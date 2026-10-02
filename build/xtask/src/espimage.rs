// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Wrapping an image for an Espressif boot ROM.
//!
//! On the parts Hubris grew up on, flash is in the address space at reset
//! and the linked image is what goes into it, at the addresses it was
//! linked for. An Espressif part's flash is a separate SPI device, and
//! what its boot ROM wants there is an image in the ROM's own format, at
//! a fixed offset: a header, then segments each naming the address its
//! bytes belong at. The ROM copies every segment to its address, checks
//! the image, and jumps to the entry point.
//!
//! So `final.bin` is not what gets written to flash on these parts; this
//! module wraps it into what does. The format is `esp_image_header_t` and
//! `esp_image_segment_header_t` in public esp-idf
//! (components/bootloader_support/include/esp_app_format.h):
//!
//! ```text
//! header (24 bytes)
//! segment header: load address, length      \  once per segment;
//! segment data, a whole number of words     /  Hubris has one
//! zero padding, then a checksum as the last byte of a 16-byte block
//! SHA-256 of everything above
//! ```
//!
//! A Hubris image is one contiguous run from the start of its "flash"
//! region, gaps already filled, so it is always a single segment.
//!
//! This is the layout for an image that runs from the memory its segments
//! are copied to. A segment that is to execute in place from flash has
//! alignment rules of its own, which nothing here implements.

use sha2::{Digest, Sha256};

use crate::config::EspImageBoardConfig;

/// `ESP_IMAGE_HEADER_MAGIC`.
const HEADER_MAGIC: u8 = 0xE9;

/// `ESP_IMAGE_SPI_SPEED_DIV_2`, the format's zero value.
const SPI_SPEED: u8 = 0;

/// `wp_pin` when the flash pins are not assigned through eFuse.
const WP_PIN_DISABLED: u8 = 0xEE;

/// No upper bound on the chip revision the image runs on.
const MAX_CHIP_REV_ANY: u16 = 0xFFFF;

/// What the checksum is seeded with (`ESP_ROM_CHECKSUM_INITIAL`).
const CHECKSUM_SEED: u8 = 0xEF;

/// Wraps `data`, linked to sit at `load_addr` and be entered at `entry`,
/// into an image the boot ROM will load.
pub fn build(
    cfg: &EspImageBoardConfig,
    load_addr: u32,
    entry: u32,
    data: &[u8],
) -> Vec<u8> {
    let segment_len = data.len().next_multiple_of(4);
    let mut out = Vec::with_capacity(segment_len + 96);

    // esp_image_header_t
    out.push(HEADER_MAGIC);
    out.push(1); // segment_count
    out.push(cfg.flash_mode as u8);
    out.push((cfg.flash_size as u8) << 4 | SPI_SPEED);
    out.extend(entry.to_le_bytes());
    out.push(WP_PIN_DISABLED);
    out.extend([0; 3]); // spi_pin_drv
    out.extend(cfg.chip_id.to_le_bytes());
    out.push(0); // min_chip_rev
    out.extend(0u16.to_le_bytes()); // min_chip_rev_full
    out.extend(MAX_CHIP_REV_ANY.to_le_bytes());
    out.extend([0; 4]); // reserved
    out.push(1); // hash_appended

    // esp_image_segment_header_t, and the segment
    out.extend(load_addr.to_le_bytes());
    out.extend((segment_len as u32).to_le_bytes());
    out.extend(data);
    out.resize(out.len() + segment_len - data.len(), 0);

    // The checksum covers segment data only, and is the last byte of a
    // 16-byte block.
    let checksum = data.iter().fold(CHECKSUM_SEED, |sum, b| sum ^ b);
    out.resize((out.len() + 1).next_multiple_of(16) - 1, 0);
    out.push(checksum);

    let digest = Sha256::digest(&out);
    out.extend(digest);
    out
}
