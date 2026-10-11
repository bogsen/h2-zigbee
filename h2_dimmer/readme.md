# Aqara H2 Dimmer

Custom Zigbee router firmware for the Aqara H2 rotary dimmer. Coupled mode controls the local output; decoupled mode sends brightness and color-temperature commands to bound lights on endpoint 11.

See the [project README](../README.md) for controls, commissioning, endpoint layout, build instructions and limitations. Variant behavior is in [app.c](app.c), with hardware assignments in [h2_variant.h](h2_variant.h).
