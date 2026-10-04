//! SFImage ↔ PNG bridging and `data:` URIs.
//!
//! A PixelTexture's SFImage (ISO/IEC 14772-1 §5.5) becomes an
//! in-memory 8-bit PNG (ISO/IEC 15948 / RFC 2083 layout: signature,
//! IHDR, zlib-compressed filtered scanlines in IDAT, IEND) so it can
//! travel through the mesh3d texture model as an encoded asset, and the
//! encoder turns PNG assets back into PixelTextures. Only the
//! non-interlaced 8-bit colour types are produced; the reader also
//! accepts palette images.

use crate::ast::Image;

const PNG_SIG: [u8; 8] = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];

/// Cap on decoded PNG pixels when converting to a PixelTexture.
pub(crate) const MAX_PNG_PIXELS: u64 = 16 * 1024 * 1024;

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let start = out.len();
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let crc = crc32(&out[start..]);
    out.extend_from_slice(&crc.to_be_bytes());
}

/// Encode an SFImage as PNG (top row first, as PNG requires). Returns
/// `None` for empty / inconsistent images.
pub(crate) fn image_to_png(img: &Image) -> Option<Vec<u8>> {
    let (w, h, c) = (
        img.width as usize,
        img.height as usize,
        img.components as usize,
    );
    if w == 0 || h == 0 || !(1..=4).contains(&c) || img.pixels.len() < w.checked_mul(h)? {
        return None;
    }
    let color_type = match c {
        1 => 0u8,
        2 => 4,
        3 => 2,
        _ => 6,
    };
    let mut raw = Vec::with_capacity(h * (w * c + 1));
    for row in (0..h).rev() {
        raw.push(0); // filter: None
        for x in 0..w {
            let p = img.pixels[row * w + x];
            for k in (0..c).rev() {
                raw.push((p >> (8 * k)) as u8);
            }
        }
    }
    let z = compcol::vec::compress_to_vec::<compcol::zlib::Zlib>(&raw).ok()?;
    let mut out = PNG_SIG.to_vec();
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&(w as u32).to_be_bytes());
    ihdr.extend_from_slice(&(h as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, color_type, 0, 0, 0]);
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &z);
    chunk(&mut out, b"IEND", &[]);
    Some(out)
}

/// Packs one decoded PNG pixel into an SFImage value.
type PixelFn = Box<dyn Fn(&[u8]) -> u32>;

fn paeth(a: u8, b: u8, c: u8) -> u8 {
    let p = a as i16 + b as i16 - c as i16;
    let (pa, pb, pc) = (
        (p - a as i16).abs(),
        (p - b as i16).abs(),
        (p - c as i16).abs(),
    );
    if pa <= pb && pa <= pc {
        a
    } else if pb <= pc {
        b
    } else {
        c
    }
}

/// Decode an 8-bit, non-interlaced PNG into an SFImage (bottom row
/// first). Returns `None` for anything else.
pub(crate) fn png_to_image(bytes: &[u8]) -> Option<Image> {
    if bytes.len() < 8 || bytes[..8] != PNG_SIG {
        return None;
    }
    let mut pos = 8;
    let mut ihdr: Option<(u32, u32, u8, u8, u8)> = None;
    let mut palette: Vec<[u8; 3]> = Vec::new();
    let mut trns: Vec<u8> = Vec::new();
    let mut idat = Vec::new();
    while pos + 8 <= bytes.len() {
        let len = u32::from_be_bytes(bytes[pos..pos + 4].try_into().ok()?) as usize;
        let kind = &bytes[pos + 4..pos + 8];
        let end = pos.checked_add(8)?.checked_add(len)?;
        if end + 4 > bytes.len() {
            return None;
        }
        let data = &bytes[pos + 8..end];
        match kind {
            b"IHDR" if data.len() >= 13 => {
                let w = u32::from_be_bytes(data[0..4].try_into().ok()?);
                let h = u32::from_be_bytes(data[4..8].try_into().ok()?);
                ihdr = Some((w, h, data[8], data[9], data[12]));
            }
            b"PLTE" => palette = data.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect(),
            b"tRNS" => trns = data.to_vec(),
            b"IDAT" => idat.extend_from_slice(data),
            b"IEND" => break,
            _ => {}
        }
        pos = end + 4;
    }
    let (w, h, depth, color_type, interlace) = ihdr?;
    if depth != 8 || interlace != 0 || w == 0 || h == 0 {
        return None;
    }
    if (w as u64) * (h as u64) > MAX_PNG_PIXELS {
        return None;
    }
    let channels = match color_type {
        0 => 1usize,
        2 => 3,
        3 => 1,
        4 => 2,
        6 => 4,
        _ => return None,
    };
    let (w, h) = (w as usize, h as usize);
    let stride = w * channels;
    let expected = h * (stride + 1);
    let raw =
        compcol::vec::decompress_to_vec_capped::<compcol::zlib::Zlib>(&idat, expected as u64 + 1)
            .ok()?;
    if raw.len() < expected {
        return None;
    }
    let mut img = vec![0u8; h * stride];
    for y in 0..h {
        let filter = raw[y * (stride + 1)];
        let src = &raw[y * (stride + 1) + 1..(y + 1) * (stride + 1)];
        for x in 0..stride {
            let a = if x >= channels {
                img[y * stride + x - channels]
            } else {
                0
            };
            let b = if y > 0 { img[(y - 1) * stride + x] } else { 0 };
            let c = if y > 0 && x >= channels {
                img[(y - 1) * stride + x - channels]
            } else {
                0
            };
            let v = src[x];
            img[y * stride + x] = match filter {
                0 => v,
                1 => v.wrapping_add(a),
                2 => v.wrapping_add(b),
                3 => v.wrapping_add(((a as u16 + b as u16) / 2) as u8),
                4 => v.wrapping_add(paeth(a, b, c)),
                _ => return None,
            };
        }
    }
    let (components, pixel): (u32, PixelFn) = match color_type {
        3 => {
            let has_alpha = !trns.is_empty();
            let pal = palette;
            let trns = trns;
            let f = move |px: &[u8]| {
                let i = px[0] as usize;
                let [r, g, b] = pal.get(i).copied().unwrap_or([0, 0, 0]);
                let base = (r as u32) << 16 | (g as u32) << 8 | b as u32;
                if has_alpha {
                    base << 8 | trns.get(i).copied().unwrap_or(255) as u32
                } else {
                    base
                }
            };
            (if has_alpha { 4 } else { 3 }, Box::new(f))
        }
        _ => (
            channels as u32,
            Box::new(|px: &[u8]| px.iter().fold(0u32, |acc, &b| acc << 8 | b as u32)),
        ),
    };
    let mut pixels = Vec::with_capacity(w * h);
    for y in (0..h).rev() {
        for x in 0..w {
            let off = y * stride + x * channels;
            pixels.push(pixel(&img[off..off + channels]));
        }
    }
    Some(Image {
        width: w as u32,
        height: h as u32,
        components,
        pixels,
    })
}

const B64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

pub(crate) fn base64_encode(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let n = (c[0] as u32) << 16
            | (c.get(1).copied().unwrap_or(0) as u32) << 8
            | c.get(2).copied().unwrap_or(0) as u32;
        for k in 0..4 {
            if k <= c.len() {
                out.push(B64[(n >> (18 - 6 * k)) as usize & 63] as char);
            } else {
                out.push('=');
            }
        }
    }
    out
}

pub(crate) fn base64_decode(s: &str) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(s.len() / 4 * 3);
    let mut acc = 0u32;
    let mut bits = 0;
    for b in s.bytes() {
        let v = match b {
            b'A'..=b'Z' => b - b'A',
            b'a'..=b'z' => b - b'a' + 26,
            b'0'..=b'9' => b - b'0' + 52,
            b'+' | b'-' => 62,
            b'/' | b'_' => 63,
            b'=' => break,
            b' ' | b'\n' | b'\r' | b'\t' => continue,
            _ => return None,
        };
        acc = acc << 6 | v as u32;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
        }
    }
    Some(out)
}

/// Parse an RFC 2397 `data:` URI → `(mime, bytes)`.
pub(crate) fn parse_data_uri(uri: &str) -> Option<(String, Vec<u8>)> {
    let rest = uri.strip_prefix("data:")?;
    let (meta, payload) = rest.split_once(',')?;
    let base64 = meta.ends_with(";base64");
    let mime = meta.trim_end_matches(";base64");
    let mime = if mime.is_empty() {
        "text/plain".to_owned()
    } else {
        mime.split(';').next().unwrap_or(mime).to_owned()
    };
    let bytes = if base64 {
        base64_decode(payload)?
    } else {
        percent_decode(payload)
    };
    Some((mime, bytes))
}

fn percent_decode(s: &str) -> Vec<u8> {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'%' && i + 2 < b.len() && s.is_char_boundary(i + 3) {
            if let Ok(v) = u8::from_str_radix(&s[i + 1..i + 3], 16) {
                out.push(v);
                i += 3;
                continue;
            }
        }
        out.push(b[i]);
        i += 1;
    }
    out
}

/// Build a base64 `data:` URI.
pub(crate) fn data_uri(mime: &str, bytes: &[u8]) -> String {
    format!("data:{mime};base64,{}", base64_encode(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn png_round_trip_all_component_counts() {
        for c in 1..=4u32 {
            let mask = if c == 4 {
                u32::MAX
            } else {
                (1u32 << (8 * c)) - 1
            };
            let img = Image {
                width: 3,
                height: 2,
                components: c,
                pixels: (0..6u32)
                    .map(|i| i.wrapping_mul(0x1234_5678) & mask)
                    .collect(),
            };
            let png = image_to_png(&img).unwrap();
            assert_eq!(png_to_image(&png).unwrap(), img);
        }
    }

    #[test]
    fn base64_and_data_uri() {
        for data in [&b""[..], b"a", b"ab", b"abc", b"abcd"] {
            assert_eq!(base64_decode(&base64_encode(data)).unwrap(), data);
        }
        let (mime, bytes) = parse_data_uri(&data_uri("image/png", b"xyz")).unwrap();
        assert_eq!(
            (mime.as_str(), bytes.as_slice()),
            ("image/png", &b"xyz"[..])
        );
        assert_eq!(parse_data_uri("data:,a%20b").unwrap().1, b"a b");
    }

    #[test]
    fn hostile_png_is_rejected() {
        assert!(png_to_image(b"\x89PNG\r\n\x1a\n\0\0\0\x0dIHDR").is_none());
        let mut huge = PNG_SIG.to_vec();
        chunk(&mut huge, b"IHDR", &[0, 1, 0, 0, 0, 1, 0, 0, 8, 6, 0, 0, 0]);
        assert!(png_to_image(&huge).is_none());
    }
}
