#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// This file defines structure of modbus parameters which reflect correspond modbus address space
// for each modbus register type (coils, discreet inputs, holding registers, input registers)
#pragma pack(push, 1)
typedef struct
{
    uint8_t discrete_input0:1;
    uint8_t discrete_input1:1;
    uint8_t discrete_input2:1;
    uint8_t discrete_input3:1;
    uint8_t discrete_input4:1;
    uint8_t discrete_input5:1;
    uint8_t discrete_input6:1;
    uint8_t discrete_input7:1;
    uint8_t discrete_input_port1:8;
} discrete_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint8_t coils_port0;
    uint8_t coils_port1;
} coil_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    float input_data0;
    float input_data1;
    float input_data2;
} input_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint16_t holding_data0;
    uint16_t holding_data1;
    uint16_t holding_data2;
    uint16_t holding_data3;
    uint16_t holding_data4;
    uint16_t holding_data5;
    uint16_t holding_data6;
    uint16_t holding_data7;
    uint16_t holding_data8;
    uint16_t holding_data9;
    uint16_t holding_data10;
} holding_reg_params_t;
#pragma pack(pop)

extern holding_reg_params_t holding_reg_params;
extern input_reg_params_t input_reg_params;
extern coil_reg_params_t coil_reg_params;
extern discrete_reg_params_t discrete_reg_params;

#ifdef __cplusplus
}
#endif
