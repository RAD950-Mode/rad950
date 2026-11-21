//! Cross-platform Live Spotting client (`POST /api/spots`, RAD950 spec v1 §6).

use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct SpotItem {
    pub spotter_call: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spotter_grid: Option<String>,
    pub tx_call: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tx_grid: Option<String>,
    pub utc_sec: u64,
    pub freq_hz: u64,
    pub band: String,
    pub snr_db: i8,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tx_pwr_code: Option<u8>,
    pub frame_type: String,
}

#[derive(Serialize)]
struct SpotBatch<'a> {
    spots: &'a [SpotItem],
}

pub fn post_spots(url: &str, api_key: &str, spots: &[SpotItem]) -> Result<usize, String> {
    if spots.is_empty() || api_key.trim().is_empty() {
        return Ok(0);
    }
    let resp = ureq::post(url)
        .set("Authorization", &format!("Bearer {}", api_key.trim()))
        .set("Content-Type", "application/json")
        .timeout(std::time::Duration::from_secs(5))
        .send_json(serde_json::to_value(SpotBatch { spots }).map_err(|e| e.to_string())?);
    match resp {
        Ok(r) if (200..300).contains(&r.status()) => Ok(spots.len()),
        Ok(r) => Err(format!("HTTP {}", r.status())),
        Err(e) => Err(e.to_string()),
    }
}
