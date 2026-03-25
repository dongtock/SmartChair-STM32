/**
  * platform.c - VL53L8CX SPI platform for STM32F411 (multi-sensor)
  */

#include "platform.h"
#include "main.h"

#define VL53L8CX_COMMS_CHUNK_SIZE  4096
#define SPI_WRITE_MASK(x)  (uint16_t)(x | 0x8000)
#define SPI_READ_MASK(x)   (uint16_t)(x & ~0x8000)

extern SPI_HandleTypeDef hspi1;

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
    return VL53L8CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
    return VL53L8CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                          uint8_t *p_values, uint32_t size)
{
    uint8_t status = 0;
    uint32_t i, position, data_size;
    uint16_t temp;

    for (position = 0; position < size; position += VL53L8CX_COMMS_CHUNK_SIZE)
    {
        if (size > VL53L8CX_COMMS_CHUNK_SIZE)
            data_size = ((position + VL53L8CX_COMMS_CHUNK_SIZE) > size)
                        ? (size - position) : VL53L8CX_COMMS_CHUNK_SIZE;
        else
            data_size = size;

        temp = RegisterAdress + position;
        p_platform->spi_comm_buffer[0] = SPI_WRITE_MASK(temp) >> 8;
        p_platform->spi_comm_buffer[1] = SPI_WRITE_MASK(temp) & 0xFF;

        for (i = 0; i < data_size; i++)
            p_platform->spi_comm_buffer[i + 2] = p_values[position + i];

        data_size += 2;

        HAL_GPIO_WritePin(p_platform->ncs_port, p_platform->ncs_pin, GPIO_PIN_RESET);
        status |= HAL_SPI_Transmit(&hspi1, p_platform->spi_comm_buffer, data_size, 100 * data_size);
        HAL_GPIO_WritePin(p_platform->ncs_port, p_platform->ncs_pin, GPIO_PIN_SET);
    }
    return status;
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                          uint8_t *p_values, uint32_t size)
{
    uint8_t status = 0;
    uint32_t position, data_size;
    uint16_t temp;
    uint8_t data_write[2];

    for (position = 0; position < size; position += VL53L8CX_COMMS_CHUNK_SIZE)
    {
        if (size > VL53L8CX_COMMS_CHUNK_SIZE)
            data_size = ((position + VL53L8CX_COMMS_CHUNK_SIZE) > size)
                        ? (size - position) : VL53L8CX_COMMS_CHUNK_SIZE;
        else
            data_size = size;

        temp = RegisterAdress + position;
        data_write[0] = SPI_READ_MASK(temp) >> 8;
        data_write[1] = SPI_READ_MASK(temp) & 0xFF;

        HAL_GPIO_WritePin(p_platform->ncs_port, p_platform->ncs_pin, GPIO_PIN_RESET);
        status |= HAL_SPI_Transmit(&hspi1, data_write, 2, 0x1000);
        status |= HAL_SPI_Receive(&hspi1, p_values + position, data_size, 100 * data_size);
        HAL_GPIO_WritePin(p_platform->ncs_port, p_platform->ncs_pin, GPIO_PIN_SET);
    }
    return status;
}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform)
{
    HAL_GPIO_WritePin(p_platform->ncs_port, p_platform->ncs_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p_platform->lpn_port, p_platform->lpn_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p_platform->pwren_port, p_platform->pwren_pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(p_platform->pwren_port, p_platform->pwren_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p_platform->lpn_port, p_platform->lpn_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p_platform->ncs_port, p_platform->ncs_pin, GPIO_PIN_SET);
    HAL_Delay(100);
    return 0;
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
    uint32_t i, tmp;
    for (i = 0; i < size; i = i + 4)
    {
        tmp = (buffer[i] << 24) | (buffer[i+1] << 16) | (buffer[i+2] << 8) | buffer[i+3];
        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs)
{
    HAL_Delay(TimeMs);
    return 0;
}
