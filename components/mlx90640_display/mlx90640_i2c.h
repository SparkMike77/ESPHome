#pragma once
#include <stdint.h>

// Wire.h-based I2C driver for MLX90640 (16-bit register address, big-endian 16-bit words)
void MLX90640_I2CInit(int sda, int scl, uint32_t freq);
int  MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nWords, uint16_t *data);
int  MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data);
void MLX90640_I2CFreqSet(uint32_t freq);
