#pragma once

#include <stdint.h>

// Manufacturer-specific "Heat Meter" cluster carrying the full meter dataset at
// higher precision than the standard Flow Measurement cluster (which is only a
// uint16 in 0.1 m^3/h). Custom cluster/attribute/device-type IDs use the
// upper-16-bits manufacturer prefix; 0xFFF1 is a Matter *test* vendor id.
// Replace 0xFFF1 with your allocated Vendor ID before shipping a real product.
#define HEAT_METER_VENDOR_PREFIX     0xFFF1u
#define HEAT_METER_DEVICE_TYPE_ID    0xFFF10001u
#define HEAT_METER_DEVICE_TYPE_VER   1
#define HEAT_METER_CLUSTER_ID        0xFFF1FC01u

// Attribute IDs within the Heat Meter cluster.
#define HM_ATTR_FLOW_ID            0x0000u  // float,  m^3/h
#define HM_ATTR_ENERGY_ID          0x0001u  // int64,  Wh
#define HM_ATTR_VOLUME_ID          0x0002u  // int64,  mL (m^3 * 1e6)
#define HM_ATTR_FLOW_TEMP_ID       0x0003u  // int32,  0.01 degC
#define HM_ATTR_RETURN_TEMP_ID     0x0004u  // int32,  0.01 degC
#define HM_ATTR_POWER_ID           0x0005u  // int64,  mW
