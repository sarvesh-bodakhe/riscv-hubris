// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use anyhow::Context as _;
use serde::Serialize;
use std::path::Path;

use crate::config::BoardConfig;

#[derive(Debug, Serialize, Default)]
pub struct FlashConfig {
    /// The name used by probe-rs to identify the chip.
    chip: Option<String>,
    /// What to write to flash, and where, on a chip whose boot ROM loads
    /// a wrapped image from flash instead of running the linked one in
    /// place. Without it, `final.elf` goes where it is linked.
    #[serde(skip_serializing_if = "Option::is_none")]
    boot_image: Option<BootImage>,
}

#[derive(Debug, Serialize)]
pub struct BootImage {
    /// Path of the image within the archive.
    path: String,
    /// Flash address to write it at.
    address: u32,
}

/// Where a board's wrapped boot image goes in the archive.
pub const BOOT_IMAGE_ARCHIVE_PATH: &str = "img/final-esp.bin";

pub fn config(board: &str) -> anyhow::Result<FlashConfig> {
    Ok(FlashConfig {
        chip: chip_name(board)?,
        boot_image: esp_image_config(board)?.map(|esp| BootImage {
            path: BOOT_IMAGE_ARCHIVE_PATH.to_string(),
            address: esp.boot_offset,
        }),
    })
}

/// Reads a board's TOML.
fn board_config(board: &str) -> anyhow::Result<BoardConfig> {
    let board_config_path = Path::new("boards").join(format!("{board}.toml"));

    let board_config_text = std::fs::read_to_string(&board_config_path)
        .with_context(|| {
            format!(
                "can't access board config at: {}",
                board_config_path.display()
            )
        })?;

    toml::from_str(&board_config_text).with_context(|| {
        format!(
            "can't parse board config at: {}",
            board_config_path.display()
        )
    })
}

/// Returns how to wrap the image for this board's boot ROM, if it
/// declares that.
pub fn esp_image_config(
    board: &str,
) -> anyhow::Result<Option<crate::config::EspImageBoardConfig>> {
    Ok(board_config(board)?.esp_image)
}

pub fn chip_name(board: &str) -> anyhow::Result<Option<String>> {
    let board_config = board_config(board)?;

    if let Some(probe_rs) = &board_config.probe_rs {
        Ok(Some(probe_rs.chip_name.clone()))
    } else {
        // tolerate the section missing for new chips, but we can't provide a
        // chip name in this case.
        Ok(None)
    }
}
