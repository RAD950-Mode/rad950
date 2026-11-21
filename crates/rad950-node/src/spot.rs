//! Cross-platform Live Spotting (`POST /api/spots`) and ADIF Log Sync (`POST /api/upload`)
//! client for `rad950.org`.

use serde::{Deserialize, Serialize};

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

#[derive(Clone, Debug, Default, Deserialize)]
pub struct UploadResult {
    #[serde(default)]
    pub inserted: usize,
    #[serde(default)]
    pub duplicates: usize,
    #[serde(default)]
    pub newly_confirmed: usize,
}

/// Derive the `POST /api/upload` endpoint URL from the configured Spots URL
/// (e.g. `https://rad950.org/api/spots` -> `https://rad950.org/api/upload`).
pub fn derive_upload_url(spot_url: &str) -> String {
    let s = spot_url.trim().trim_end_matches('/');
    if let Some(prefix) = s.strip_suffix("/api/spots") {
        format!("{prefix}/api/upload")
    } else if s.ends_with("/api/upload") {
        s.to_string()
    } else if s.is_empty() {
        "https://rad950.org/api/upload".to_string()
    } else {
        format!("{s}/api/upload")
    }
}

/// Upload an ADIF log payload to `rad950.org/api/upload` using the user's `r950_...` Bearer API key.
pub fn upload_adif(
    upload_url: &str,
    api_key: &str,
    callsign: &str,
    filename: &str,
    adif_content: &str,
) -> Result<UploadResult, String> {
    if api_key.trim().is_empty() {
        return Err("No rad950.org API key configured in Settings".to_string());
    }
    if adif_content.trim().is_empty() {
        return Err("ADIF log is empty".to_string());
    }
    let body = serde_json::json!({
        "filename": filename,
        "format": "adif",
        "callsign": callsign,
        "content": adif_content,
    });
    let resp = ureq::post(upload_url)
        .set("Authorization", &format!("Bearer {}", api_key.trim()))
        .set("Content-Type", "application/json")
        .timeout(std::time::Duration::from_secs(10))
        .send_json(body);
    match resp {
        Ok(r) if (200..300).contains(&r.status()) => {
            r.into_json::<UploadResult>()
                .map_err(|e| format!("parse upload response: {e}"))
        }
        Ok(r) => {
            let status = r.status();
            let msg = r.into_string().unwrap_or_default();
            Err(format!("HTTP {status}: {msg}"))
        }
        Err(ureq::Error::Status(status, r)) => {
            let msg = r.into_string().unwrap_or_default();
            Err(format!("HTTP {status}: {msg}"))
        }
        Err(e) => Err(e.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derive_upload_url_maps_spots_endpoint() {
        assert_eq!(
            derive_upload_url("https://rad950.org/api/spots"),
            "https://rad950.org/api/upload"
        );
        assert_eq!(
            derive_upload_url("https://rad950.org/api/spots/"),
            "https://rad950.org/api/upload"
        );
        assert_eq!(
            derive_upload_url("http://localhost:8787/api/spots"),
            "http://localhost:8787/api/upload"
        );
    }
}
