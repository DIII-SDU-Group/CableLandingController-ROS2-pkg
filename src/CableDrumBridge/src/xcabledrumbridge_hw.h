// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// AXI_CPU
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of mode_CPU
//        bit 1~0 - mode_CPU[1:0] (Read/Write)
//        others  - reserved
// 0x14 : reserved
// 0x18 : Data signal of duty_cycle_CPU
//        bit 6~0 - duty_cycle_CPU[6:0] (Read/Write)
//        others  - reserved
// 0x1c : reserved
// 0x20 : Data signal of man_dir_CPU
//        bit 0  - man_dir_CPU[0] (Read/Write)
//        others - reserved
// 0x24 : reserved
// 0x28 : Data signal of ref_flex_CPU
//        bit 3~0 - ref_flex_CPU[3:0] (Read/Write)
//        others  - reserved
// 0x2c : reserved
// 0x30 : Data signal of sensor_flex_CPU
//        bit 3~0 - sensor_flex_CPU[3:0] (Read)
//        others  - reserved
// 0x34 : Control signal of sensor_flex_CPU
//        bit 0  - sensor_flex_CPU_ap_vld (Read/COR)
//        others - reserved
// 0x40 : Data signal of gain_CPU
//        bit 5~0 - gain_CPU[5:0] (Read/Write)
//        others  - reserved
// 0x44 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XCABLEDRUMBRIDGE_AXI_CPU_ADDR_MODE_CPU_DATA        0x10
#define XCABLEDRUMBRIDGE_AXI_CPU_BITS_MODE_CPU_DATA        2
#define XCABLEDRUMBRIDGE_AXI_CPU_ADDR_DUTY_CYCLE_CPU_DATA  0x18
#define XCABLEDRUMBRIDGE_AXI_CPU_BITS_DUTY_CYCLE_CPU_DATA  7
#define XCABLEDRUMBRIDGE_AXI_CPU_ADDR_MAN_DIR_CPU_DATA     0x20
#define XCABLEDRUMBRIDGE_AXI_CPU_BITS_MAN_DIR_CPU_DATA     1
#define XCABLEDRUMBRIDGE_AXI_CPU_ADDR_REF_FLEX_CPU_DATA    0x28
#define XCABLEDRUMBRIDGE_AXI_CPU_BITS_REF_FLEX_CPU_DATA    4
#define XCABLEDRUMBRIDGE_AXI_CPU_ADDR_SENSOR_FLEX_CPU_DATA 0x30
#define XCABLEDRUMBRIDGE_AXI_CPU_BITS_SENSOR_FLEX_CPU_DATA 4
#define XCABLEDRUMBRIDGE_AXI_CPU_ADDR_SENSOR_FLEX_CPU_CTRL 0x34
#define XCABLEDRUMBRIDGE_AXI_CPU_ADDR_GAIN_CPU_DATA        0x40
#define XCABLEDRUMBRIDGE_AXI_CPU_BITS_GAIN_CPU_DATA        6

