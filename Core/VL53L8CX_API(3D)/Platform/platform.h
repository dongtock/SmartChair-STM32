/**
  * platform.h - VL53L8CX SPI platform for STM32F411 (multi-sensor)
  */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"

typedef struct
{
    uint16_t address;
    uint8_t spi_comm_buffer[32800];

    GPIO_TypeDef *ncs_port;
    uint16_t      ncs_pin;
    GPIO_TypeDef *lpn_port;
    uint16_t      lpn_pin;
    GPIO_TypeDef *pwren_port;
    uint16_t      pwren_pin;
} VL53L8CX_Platform;

#define VL53L8CX_NB_TARGET_PER_ZONE   1U

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform);
void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs);

#endif
