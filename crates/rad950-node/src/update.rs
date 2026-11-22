//! Non-blocking release version check against `https://rad950.org/api/releases/latest`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde::Deserialize;

const RELEASES_LATEST_URL: &str = "https://rad950.org/api/releases/latest";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UpdateInfo {
    pub current: String,
    pub latest_tag: String,
    pub release_url: String,
}

#[derive(Deserialize)]
struct LatestReleaseResp {
    tag: Option<String>,
    release_url: Option<String>,
}

/// Parse a semver-ish string (`"v0.1.2"`, `"0.1.3-rc1"`) into `(major, minor, patch)`.
fn parse_semver(s: &str) -> Option<(u32, u32, u32)> {
    let trimmed = s.trim().trim_start_matches(['v', 'V']);
    let core = trimmed.split(['-', '+']).next()?;
    let mut parts = core.split('.');
    let maj = parts.next()?.parse().ok()?;
    let min = parts.next().unwrap_or("0").parse().ok()?;
    let pat = parts.next().unwrap_or("0").parse().ok()?;
    Some((maj, min, pat))
}

/// Returns `true` if `latest_tag` represents a strictly newer version than `current`.
pub fn is_newer_version(current: &str, latest_tag: &str) -> bool {
    match (parse_semver(current), parse_semver(latest_tag)) {
        (Some(cur), Some(lat)) => lat > cur,
        _ => false,
    }
}

/// Check `https://rad950.org/api/releases/latest` on a detached background thread.
/// Silently does nothing when offline or on any network/parse error; populates `on_update`
/// only when a strictly newer release is available.
pub fn check_for_update_async<F>(current_version: &'static str, on_update: F)
where
    F: FnOnce(UpdateInfo) + Send + 'static,
{
    let _ = std::thread::Builder::new()
        .name("rad950-update-check".to_string())
        .spawn(move || {
            let agent = ureq::AgentBuilder::new()
                .timeout_connect(Duration::from_secs(3))
                .timeout_read(Duration::from_secs(4))
                .build();
            let Ok(resp) = agent
                .get(RELEASES_LATEST_URL)
                .set("User-Agent", &format!("rad950/{current_version}"))
                .call()
            else {
                return;
            };
            let Ok(parsed) = resp.into_json::<LatestReleaseResp>() else {
                return;
            };
            let Some(tag) = parsed.tag.filter(|t| !t.trim().is_empty()) else {
                return;
            };
            if !is_newer_version(current_version, &tag) {
                return;
            }
            let release_url = parsed
                .release_url
                .filter(|u| !u.trim().is_empty())
                .unwrap_or_else(|| "https://github.com/RAD950-Mode/rad950/releases/latest".to_string());
            on_update(UpdateInfo {
                current: current_version.to_string(),
                latest_tag: tag,
                release_url,
            });
        });
}

/// Convenience helper for storing `UpdateInfo` in a shared `Mutex<Option<UpdateInfo>>`.
pub fn spawn_update_check(current_version: &'static str, target: Arc<Mutex<Option<UpdateInfo>>>) {
    check_for_update_async(current_version, move |info| {
        if let Ok(mut guard) = target.lock() {
            *guard = Some(info);
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn semver_comparison_handles_v_prefix_and_prerelease() {
        assert!(is_newer_version("0.1.2", "v0.1.3"));
        assert!(is_newer_version("0.1.2", "0.2.0"));
        assert!(is_newer_version("0.1.2", "v1.0.0"));
        assert!(!is_newer_version("0.1.2", "v0.1.2"));
        assert!(!is_newer_version("0.1.2", "v0.1.1"));
        assert!(!is_newer_version("0.2.0", "v0.1.9"));
        assert!(is_newer_version("0.1.2", "v0.1.3-beta1"));
        assert!(!is_newer_version("0.1.2", "not-a-version"));
    }
}
