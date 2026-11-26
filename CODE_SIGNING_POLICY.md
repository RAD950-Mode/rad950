# Code Signing Policy

Free code signing provided by [SignPath.io](https://about.signpath.io/), certificate by [SignPath Foundation](https://signpath.org/).

## Status

- **Current State:** Windows code signing via SignPath Foundation is being onboarded. Once active, all official Windows releases starting from the first signed tag will carry an Authenticode signature issued by SignPath Foundation.
- **License:** All signed binaries are built exclusively from MIT-licensed open-source code in [RAD950-Mode/rad950](https://github.com/RAD950-Mode/rad950) (see [LICENSE](LICENSE)).

## Scope of Signed Artifacts

Code signing applies exclusively to official Windows distribution binaries built from this repository by automated GitHub Actions workflows:

- `rad950.exe` — Windows x86_64 desktop transceiver (`crates/rad950-app`)
- `rad950-windows-x64-setup.exe` — Official per-user Windows installer (`installer/rad950.iss`)

**Excluded:** Non-Windows builds (macOS/Linux), third-party runtimes, and separately built Radtel RT-950 Pro handheld firmware images (`.ffw` / `.btf`) are not signed with the SignPath Foundation certificate.

## Team Roles & Responsibilities

In accordance with SignPath Foundation governance requirements, project roles are assigned as follows:

- **Authors / Committers:** Matt Murphy ([@mmmurf](https://github.com/mmmurf)) and open-source contributors submitting pull requests.
- **Reviewers:** Matt Murphy ([@mmmurf](https://github.com/mmmurf)) — reviews code changes, dependency updates, and CI/CD workflow changes.
- **Approvers:** Matt Murphy ([@mmmurf](https://github.com/mmmurf)) — reviews and explicitly approves signing requests in SignPath for official releases.

Multi-factor authentication (MFA) is enforced on GitHub and SignPath accounts for all maintainers and approvers.

## Build & Release Process

1. Official release binaries are compiled exclusively on GitHub-hosted Actions runners (`.github/workflows/release.yml`) triggered by official version tags (`v*`).
2. Unsigned Windows artifacts are uploaded as GitHub Actions build artifacts and submitted to SignPath via the official SignPath GitHub Action for origin verification.
3. No locally built developer binaries are submitted for signing.

## Privacy Policy

RAD-950 does not collect telemetry, usage analytics, advertising identifiers, or personal data.

- Configuration and log files (`config.toml`, `rad950.adi`, `ALL.TXT`) are stored locally on the user's machine.
- Optional amateur radio network features (live reception spotting and QSO log sync to `rad950.org`) only transmit standard amateur radio operating data (callsign, Maidenhead grid square, frequency/band, SNR, and QSO log fields) when configured by the operator.
