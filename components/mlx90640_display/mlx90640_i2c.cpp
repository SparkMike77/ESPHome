// Wire.h is accessible here because this file compiles as component source,
// not as a PlatformIO library dependency — component sources always see the
// Arduino framework headers.
#include <Wire.h>
#include "mlx90640_i2c.h"

static TwoWire *s_wire = &Wire;

void MLX90640_I2CInit(int sda, int scl, uint32_t freq) {
    s_wire->begin(sda, scl);
    s_wire->setClock(freq);
}

void MLX90640_I2CFreqSet(uint32_t freq) {
    s_wire->setClock(freq);
}

// Read nWords big-endian 16-bit words from startAddress.
// Reads in 64-word chunks to stay within the 128-byte Wire buffer.
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t nWords, uint16_t *data) {
    uint16_t remaining = nWords;
    uint16_t addr      = startAddress;
    uint16_t *ptr      = data;

    while (remaining > 0) {
        uint16_t chunk = (remaining > 64u) ? 64u : remaining;

        s_wire->beginTransmission(slaveAddr);
        s_wire->write((uint8_t)(addr >> 8));
        s_wire->write((uint8_t)(addr & 0xFF));
        if (s_wire->endTransmission(false) != 0) return -1;

        s_wire->requestFrom((int)slaveAddr, (int)(chunk * 2));
        for (uint16_t i = 0; i < chunk; i++) {
            uint8_t msb = s_wire->read();
            uint8_t lsb = s_wire->read();
            *ptr++ = ((uint16_t)msb << 8) | lsb;
        }

        addr      += chunk;
        remaining -= chunk;
    }
    return 0;
}

// Write a single 16-bit word to writeAddress.
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
    s_wire->beginTransmission(slaveAddr);
    s_wire->write((uint8_t)(writeAddress >> 8));
    s_wire->write((uint8_t)(writeAddress & 0xFF));
    s_wire->write((uint8_t)(data >> 8));
    s_wire->write((uint8_t)(data & 0xFF));
    return (s_wire->endTransmission() != 0) ? -1 : 0;
}
