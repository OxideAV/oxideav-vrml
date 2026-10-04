//! decode(encode(scene)) ≈ scene.

use oxideav_mesh3d::{
    Animation, AnimationChannel, AnimationProperty, AnimationSampler, AnimationValues, Camera,
    ImageData, Indices, Interpolation, Light, Material, Mesh, MorphTarget, Node, Primitive,
    Scene3D, Texture, TextureRef, Topology, Transform,
};
use oxideav_vrml::{VrmlDecoder, VrmlEncoder};

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(format!(
        "{}/tests/fixtures/{name}",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap()
}

fn round_trip(s: &Scene3D) -> (Scene3D, String) {
    let bytes = VrmlEncoder::new().encode_scene(s).unwrap();
    let text = String::from_utf8(bytes.clone()).unwrap();
    let back = VrmlDecoder::new()
        .decode_scene(&bytes)
        .unwrap_or_else(|e| panic!("{e}\n{text}"));
    assert!(back.validate().is_ok(), "{:?}\n{text}", back.validate());
    (back, text)
}

/// Scene summary: (triangles, lines, points, world area, world bbox).
fn summary(s: &Scene3D) -> (usize, usize, usize, f64, [f32; 6]) {
    let world = s.world_node_transforms();
    let (mut tris, mut lines, mut points, mut area) = (0, 0, 0, 0.0);
    let mut bb = [
        f32::INFINITY,
        f32::INFINITY,
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::NEG_INFINITY,
        f32::NEG_INFINITY,
    ];
    for (i, n) in s.nodes.iter().enumerate() {
        let (Some(m), Some(w)) = (n.mesh, world[i]) else {
            continue;
        };
        for p in &s.meshes[m.0 as usize].primitives {
            match p.topology {
                Topology::Lines | Topology::LineStrip | Topology::LineLoop => {
                    let n = p.indices.as_ref().map_or(p.positions.len(), Indices::len);
                    lines += match p.topology {
                        Topology::Lines => n / 2,
                        Topology::LineStrip => n.saturating_sub(1),
                        _ => n,
                    }
                }
                Topology::Points => points += p.positions.len(),
                _ => {
                    tris += p.triangle_count();
                    area += p.world_surface_area(w);
                }
            }
            for v in &p.positions {
                let wv =
                    [0, 1, 2].map(|r| w[r][0] * v[0] + w[r][1] * v[1] + w[r][2] * v[2] + w[r][3]);
                for a in 0..3 {
                    bb[a] = bb[a].min(wv[a]);
                    bb[a + 3] = bb[a + 3].max(wv[a]);
                }
            }
        }
    }
    (tris, lines, points, area, bb)
}

fn assert_close_summary(a: &Scene3D, b: &Scene3D, what: &str) {
    let (sa, sb) = (summary(a), summary(b));
    assert_eq!(
        (sa.0, sa.1, sa.2),
        (sb.0, sb.1, sb.2),
        "{what}: element counts"
    );
    assert!(
        (sa.3 - sb.3).abs() < 1e-3 * sa.3.max(1.0),
        "{what}: area {} vs {}",
        sa.3,
        sb.3
    );
    for k in 0..6 {
        if !sa.4[k].is_finite() && !sb.4[k].is_finite() {
            continue;
        }
        assert!(
            (sa.4[k] - sb.4[k]).abs() < 1e-4,
            "{what}: bbox {:?} vs {:?}",
            sa.4,
            sb.4
        );
    }
}

#[test]
fn fixtures_round_trip() {
    for name in [
        "primitives.wrl",
        "indexed.wrl",
        "lines_points.wrl",
        "grid_extrusion.wrl",
        "protos.wrl",
        "animation.wrl",
        "environment.wrl",
        "textures.wrl",
        "grouping.wrl",
    ] {
        let a = VrmlDecoder::new().decode_scene(&fixture(name)).unwrap();
        let (b, text) = round_trip(&a);
        assert_close_summary(&a, &b, name);
        assert_eq!(a.cameras.len(), b.cameras.len(), "{name}");
        assert_eq!(a.lights.len(), b.lights.len(), "{name}");
        assert_eq!(a.textures.len(), b.textures.len(), "{name}\n{text}");
        let chans = |s: &Scene3D| s.animations.iter().map(|x| x.channels.len()).sum::<usize>();
        assert_eq!(chans(&a), chans(&b), "{name}\n{text}");
        for k in [
            "vrml:worldInfo",
            "vrml:navigationInfo",
            "vrml:background",
            "vrml:fog",
        ] {
            assert_eq!(
                a.extras.contains_key(k),
                b.extras.contains_key(k),
                "{name}: {k}"
            );
        }
        // Second generation is textually stable.
        let (_, text2) = round_trip(&b);
        let (_, text3) = round_trip(&VrmlDecoder::new().decode_scene(text2.as_bytes()).unwrap());
        if text2 != text3 {
            if let Ok(dir) = std::env::var("VRML_DUMP") {
                std::fs::write(format!("{dir}/{name}.2"), &text2).unwrap();
                std::fs::write(format!("{dir}/{name}.3"), &text3).unwrap();
            }
            panic!("{name}: unstable encoding");
        }
    }
}

fn quad() -> Primitive {
    let mut p = Primitive::new(Topology::Triangles);
    p.positions = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ];
    p.normals = Some(vec![[0.0, 0.0, 1.0]; 4]);
    p.uvs = vec![vec![[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]]];
    p.colors = vec![vec![
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0, 1.0],
    ]];
    p.indices = Some(Indices::U16(vec![0, 1, 2, 0, 2, 3]));
    p
}

fn png_1x1() -> Vec<u8> {
    // Produce via the decoder's own PixelTexture path.
    let s = VrmlDecoder::new()
        .decode_scene(b"#VRML V2.0 utf8\nShape { appearance Appearance { texture PixelTexture { image 1 1 3 0x336699 } } geometry Box {} }")
        .unwrap();
    match &s.textures[0].image {
        ImageData::Source(src) => {
            let mut v = Vec::new();
            std::io::Read::read_to_end(&mut src.open().unwrap(), &mut v).unwrap();
            v
        }
        _ => unreachable!(),
    }
}

fn built_scene() -> Scene3D {
    let mut s = Scene3D::new();
    let tex_ext = s.add_texture(Texture::from_uri("wood.jpg"));
    let tex_png = s.add_texture(Texture::from_encoded("image/png", png_1x1()));
    let tex_jpg = s.add_texture(Texture::from_encoded(
        "image/jpeg",
        vec![0xFF, 0xD8, 0xFF, 0xD9],
    ));
    let mut m1 = Material::new().with_name("Paint");
    m1.base_color = [0.2, 0.4, 0.6, 1.0];
    m1.metallic = 0.0;
    m1.roughness = 0.5;
    m1.double_sided = true;
    let m1 = s.add_material(m1);
    let mut m2 = Material::new();
    m2.base_color_texture = Some(TextureRef::new(tex_ext));
    let m2 = s.add_material(m2);
    let mut m3 = Material::new();
    m3.base_color_texture = Some(TextureRef::new(tex_png));
    let m3 = s.add_material(m3);
    let mut m4 = Material::new();
    m4.base_color_texture = Some(TextureRef::new(tex_jpg));
    m4.base_color = [1.0, 1.0, 1.0, 0.5];
    m4.alpha_mode = oxideav_mesh3d::AlphaMode::Blend;
    let m4 = s.add_material(m4);

    let mut p1 = quad();
    p1.material = Some(m1);
    let mut p2 = quad();
    p2.material = Some(m2);
    p2.colors.clear();
    let mesh = s.add_mesh(
        Mesh::new(Some("Quads".to_owned()))
            .with_primitive(p1)
            .with_primitive(p2),
    );
    let mut p3 = quad();
    p3.material = Some(m3);
    p3.colors.clear();
    let mut p4 = quad();
    p4.material = Some(m4);
    p4.colors.clear();
    let mesh2 = s.add_mesh(Mesh::new(None).with_primitive(p3).with_primitive(p4));
    let mut lines = Primitive::new(Topology::LineStrip);
    lines.positions = vec![[0.0, 0.0, 0.0], [0.0, 2.0, 0.0], [1.0, 2.0, 0.0]];
    let lines_mesh = s.add_mesh(Mesh::new(None).with_primitive(lines));

    // Morph-target mesh.
    let mut mp = Primitive::new(Topology::Triangles);
    mp.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    mp.indices = Some(Indices::U16(vec![0, 1, 2]));
    let mut t = MorphTarget::new();
    t.position = Some(vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]);
    mp.targets.push(t);
    let morph_mesh = s.add_mesh(Mesh::new(None).with_primitive(mp).with_weights(vec![0.0]));

    let a = s.add_node(
        Node::new()
            .with_name("A")
            .with_mesh(mesh)
            .with_transform(Transform::Trs {
                translation: [1.0, 2.0, 3.0],
                rotation: [0.0, 0.38268343, 0.0, 0.9238795],
                scale: [1.0, 2.0, 1.0],
            }),
    );
    let b = s.add_node(Node::new().with_mesh(mesh).with_transform(Transform::Trs {
        translation: [-4.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
        scale: [1.0; 3],
    }));
    let c = s.add_node(Node::new().with_mesh(mesh2).with_transform(Transform::Trs {
        translation: [0.0, -3.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
        scale: [1.0; 3],
    }));
    let l = s.add_node(Node::new().with_mesh(lines_mesh));
    let mo = s.add_node(Node::new().with_name("Morpher").with_mesh(morph_mesh));
    s.nodes[a.0 as usize].children.push(l);
    let cam = s.add_camera(Camera::perspective(0.7, 0.1));
    let mut cn = Node::new().with_name("Cam").with_transform(Transform::Trs {
        translation: [0.0, 1.0, 5.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
        scale: [1.0; 3],
    });
    cn.camera = Some(cam);
    let cn = s.add_node(cn);
    let pl = s.add_light(Light::Point {
        color: [1.0, 0.5, 0.25],
        intensity: 0.75,
        range: Some(12.0),
    });
    let mut pn = Node::new().with_transform(Transform::Trs {
        translation: [2.0, 2.0, 2.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
        scale: [1.0; 3],
    });
    pn.light = Some(pl);
    let pn = s.add_node(pn);
    let sl = s.add_light(Light::Spot {
        color: [1.0; 3],
        intensity: 1.0,
        range: None,
        inner_cone_angle: 0.2,
        outer_cone_angle: 0.5,
    });
    let mut sn = Node::new().with_transform(Transform::Trs {
        translation: [0.0, 4.0, 0.0],
        rotation: [
            -std::f32::consts::FRAC_1_SQRT_2,
            0.0,
            0.0,
            std::f32::consts::FRAC_1_SQRT_2,
        ],
        scale: [1.0; 3],
    });
    sn.light = Some(sl);
    let sn = s.add_node(sn);
    let dl = s.add_light(Light::Directional {
        color: [1.0; 3],
        intensity: 0.5,
    });
    let mut dn = Node::new();
    dn.light = Some(dl);
    let dn = s.add_node(dn);
    s.roots = vec![a, b, c, mo, cn, pn, sn, dn];

    let mut anim = Animation::new(Some("Move".to_owned()));
    anim.channels.push(AnimationChannel::new(
        a,
        AnimationProperty::Translation,
        AnimationSampler {
            keyframes: vec![0.0, 1.0, 2.0],
            values: AnimationValues::Vec3(vec![[1.0, 2.0, 3.0], [1.0, 4.0, 3.0], [1.0, 2.0, 3.0]]),
            interpolation: Interpolation::Linear,
        },
    ));
    anim.channels.push(AnimationChannel::new(
        a,
        AnimationProperty::Rotation,
        AnimationSampler {
            keyframes: vec![0.0, 2.0],
            values: AnimationValues::Quat(vec![
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.70710677, 0.0, 0.70710677],
            ]),
            interpolation: Interpolation::Linear,
        },
    ));
    anim.channels.push(AnimationChannel::new(
        a,
        AnimationProperty::Scale,
        AnimationSampler {
            keyframes: vec![0.0, 1.0],
            values: AnimationValues::Vec3(vec![[1.0; 3], [2.0; 3]]),
            interpolation: Interpolation::Step,
        },
    ));
    anim.channels.push(AnimationChannel::new(
        mo,
        AnimationProperty::MorphWeights,
        AnimationSampler {
            keyframes: vec![0.0, 2.0],
            values: AnimationValues::Scalar(vec![0.0, 1.0]),
            interpolation: Interpolation::Linear,
        },
    ));
    s.animations.push(anim);
    s
}

#[test]
fn built_scene_round_trip() {
    let s = built_scene();
    assert!(s.validate().is_ok(), "{:?}", s.validate());
    let (b, text) = round_trip(&s);
    assert_close_summary(&s, &b, "built");
    // Shared mesh written once (DEF) and re-used (USE).
    assert!(text.contains("USE Quads_0"), "{text}");
    let quads = b
        .meshes
        .iter()
        .position(|m| m.primitives.len() == 2 && m.primitives[0].colors.len() == 1)
        .unwrap();
    assert_eq!(
        b.nodes
            .iter()
            .filter(|n| n.mesh.map(|m| m.0 as usize) == Some(quads))
            .count(),
        2
    );
    let p = &b.meshes[quads].primitives[0];
    assert_eq!(p.uvs[0].len(), 4);
    assert!(p.uvs[0].contains(&[0.0, 1.0]));
    // Base colour × COLOR_0 is baked into the VRML Color node, which
    // replaces the diffuse colour on the way back.
    assert!(
        p.colors[0].contains(&[0.0, 0.4, 0.0, 1.0]),
        "{:?}",
        p.colors[0]
    );
    let m = &b.materials[p.material.unwrap().0 as usize];
    assert!(m.double_sided);
    assert_eq!(m.base_color, [1.0; 4]);
    assert_eq!(m.extras["vrml:material"]["diffuseColor"][2], 0.6f32 as f64);
    let p2 = &b.meshes[quads].primitives[1];
    assert_eq!(
        b.materials[p2.material.unwrap().0 as usize].base_color,
        [1.0; 4]
    );
    assert!((m.roughness - 0.5).abs() < 1e-3, "{}", m.roughness);
    // Textures: URI, PNG → PixelTexture → PNG, JPEG → data: URI.
    assert_eq!(b.textures.len(), 3);
    assert!(text.contains("PixelTexture"));
    assert!(text.contains("data:image/jpeg;base64,"));
    assert!(matches!(&b.textures[0].image, ImageData::External { uri, .. } if uri == "wood.jpg"));
    let transparent = b.materials.iter().find(|m| m.base_color[3] == 0.5).unwrap();
    assert_eq!(transparent.alpha_mode, oxideav_mesh3d::AlphaMode::Blend);
    // Camera.
    let cam = b.nodes.iter().find(|n| n.camera.is_some()).unwrap();
    assert_eq!(cam.name.as_deref(), Some("Cam"));
    assert!(matches!(b.cameras[0], Camera::Perspective { yfov, .. } if (yfov - 0.7).abs() < 1e-6));
    // Lights.
    assert!(b.lights.iter().any(|l| matches!(l, Light::Point { range: Some(r), intensity, .. } if *r == 12.0 && *intensity == 0.75)));
    let spot = b
        .nodes
        .iter()
        .find(|n| {
            n.light
                .is_some_and(|l| matches!(b.lights[l.0 as usize], Light::Spot { .. }))
        })
        .unwrap();
    let world = b.world_node_transforms();
    let si = b.nodes.iter().position(|n| std::ptr::eq(n, spot)).unwrap();
    let w = world[si].unwrap();
    assert!((-w[1][2] + 1.0).abs() < 1e-5, "spot points down: {w:?}");
    assert!(b
        .lights
        .iter()
        .any(|l| matches!(l, Light::Directional { intensity, .. } if *intensity == 0.5)));
    // Animations.
    assert_eq!(b.animations.len(), 1);
    let anim = &b.animations[0];
    assert_eq!(anim.name.as_deref(), Some("Move"));
    let ai = b
        .nodes
        .iter()
        .position(|n| n.name.as_deref() == Some("A"))
        .unwrap();
    let ch = |prop| {
        anim.channels
            .iter()
            .find(|c| c.target.node.0 as usize == ai && c.target.property == prop)
            .unwrap()
    };
    let tr = ch(AnimationProperty::Translation);
    assert_eq!(tr.sampler.keyframes, [0.0, 1.0, 2.0]);
    let rot = ch(AnimationProperty::Rotation);
    let q = rot.sampler.sample(1.0).unwrap();
    let orig = s.animations[0].channels[1].sampler.sample(1.0).unwrap();
    match (q, orig) {
        (oxideav_mesh3d::SampledValue::Quat(a), oxideav_mesh3d::SampledValue::Quat(b)) => {
            let d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]).abs();
            assert!(d > 0.9999, "{a:?} vs {b:?}");
        }
        _ => panic!(),
    }
    // Step emulated with duplicated keys: value holds until the jump.
    let sc = ch(AnimationProperty::Scale);
    assert_eq!(
        sc.sampler.sample(0.9),
        Some(oxideav_mesh3d::SampledValue::Vec3([1.0; 3]))
    );
    // Morph: CoordinateInterpolator → morph target, same shape at t = 2.
    let mi = b
        .nodes
        .iter()
        .position(|n| n.name.as_deref() == Some("Morpher"))
        .unwrap();
    let mw = anim
        .channels
        .iter()
        .find(|c| c.target.node.0 as usize == mi)
        .unwrap();
    assert_eq!(mw.target.property, AnimationProperty::MorphWeights);
    let mesh = &b.meshes[b.nodes[mi].mesh.unwrap().0 as usize];
    let end = mesh.primitives[0].morphed(&[0.0, 1.0]);
    assert_eq!(end.bounding_box().unwrap().max, [2.0, 1.0, 0.0]);
}

#[test]
fn gzip_output() {
    let s = built_scene();
    let gz = VrmlEncoder::gzip().encode_scene(&s).unwrap();
    assert_eq!(&gz[..2], &[0x1F, 0x8B]);
    let back = VrmlDecoder::new().decode_scene(&gz).unwrap();
    assert_close_summary(&s, &back, "gzip");
}
