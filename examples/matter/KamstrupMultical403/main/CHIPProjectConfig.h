#pragma once

// Overrides CHIPDeviceConfig.h's "TEST_VENDOR"/"TEST_PRODUCT" defaults -- the
// strings a Matter controller (Apple Home, Google Home, ...) shows for
// "Fabricant"/"Modele". BasicInformation is a ServerClusterInterface-backed
// cluster that reads these compile-time macros via the transitional
// DeviceInstanceInfoProvider, not the esp-matter attribute store, so this is
// the only way to change them short of provisioning an ESP32 factory-data NVS
// partition. Included globally via CHIP_DEVICE_PROJECT_CONFIG_INCLUDE in the
// top-level CMakeLists.txt. Matter caps both strings at 32 characters.
#define CHIP_DEVICE_CONFIG_DEVICE_VENDOR_NAME "matter-esp32-mbus-adapter"
#define CHIP_DEVICE_CONFIG_DEVICE_PRODUCT_NAME "Kamstrup MULTICAL 403"
