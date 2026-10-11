# H2 Internal Storage Bootloader

Gecko bootloader for the EFR32MG24 with 1536 KiB flash. It supports LZMA-compressed application updates and stores one update image in internal flash.

The [storage configuration](config/btl_storage_slot_cfg.h) enables slot 0 at `0x080C0000` with a size of **737280 bytes (720 KiB)**, ending at `0x08174000` (exclusive). Keep the bootloader storage layout consistent with the space reserved by both applications.

Initial installation places the bootloader at `0x08000000` and the application at `0x08006000`. See the [project README](../README.md) for building and the [SWD instructions](../swd_flasher/README.md) for installation.
