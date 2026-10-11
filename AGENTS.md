# Firmware versions

- When changing firmware, increment `H2_FIRMWARE_VERSION` in each affected variant's `app.c` before building the final artifacts.
- Changes to shared firmware code affect both variants unless explicitly gated to one variant.
- Bump once per completed change, not for every intermediate build or test run.
- Rebuild each affected variant's OTA artifact and verify that its header contains the new version. Report the resulting versions to the user.
