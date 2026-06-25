#pragma once
#include <stdint.h>

#define MLX90640_I2CADDR_DEFAULT 0x33
#define MLX90640_EEPROM_WORDS    832   // 0x2400..0x273F
#define MLX90640_FRAME_WORDS     834   // 768 pixels + 64 aux + 2 status words

typedef struct {
    int16_t  kVdd;
    int16_t  vdd25;
    float    KvPTAT;
    float    KtPTAT;
    uint16_t vPTAT25;
    float    alphaPTAT;
    int16_t  gainEE;
    float    tgc;
    float    cpKv;
    float    cpKta;
    uint8_t  resolutionEE;
    uint8_t  calibrationModeEE;  // 0 = chess, 1 = interleaved
    float    KsTa;
    float    ksTo[5];
    int16_t  ct[5];
    uint16_t alpha[768];
    uint8_t  alphaScale;
    int16_t  offset[768];
    int8_t   kta[768];
    uint8_t  ktaScale;
    int8_t   kv[768];
    uint8_t  kvScale;
    float    cpAlpha[2];
    int16_t  cpOffset[2];
    float    ilChessC[3];
    uint16_t brokenPixels[5];
    uint16_t outlierPixels[5];
} paramsMLX90640;

#ifdef __cplusplus
extern "C" {
#endif

// Read EEPROM (MLX90640_EEPROM_WORDS words) into eeData[].
int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t *eeData);

// Read one frame of sensor data (MLX90640_FRAME_WORDS words) into frameData[].
// Returns subpage number (0 or 1) on success, negative on error.
// Blocks until the sensor signals data-ready.
int MLX90640_GetFrameData(uint8_t slaveAddr, uint16_t *frameData);

// Parse EEPROM data into calibration parameters.
int MLX90640_ExtractParameters(uint16_t *eeData, paramsMLX90640 *params);

// Compute supply voltage from a captured frame.
float MLX90640_GetVdd(uint16_t *frameData, const paramsMLX90640 *params);

// Compute ambient (chip) temperature from a captured frame.
float MLX90640_GetTa(uint16_t *frameData, const paramsMLX90640 *params);

// Compute pixel temperatures into result[768] (°C).
// emissivity: typically 0.95 for most surfaces.
// tr: reflected temperature in °C (usually Ta - 8).
void MLX90640_CalculateTo(uint16_t *frameData, const paramsMLX90640 *params,
                           float emissivity, float tr, float *result);

// Set sensor refresh rate (0=0.5 Hz … 7=64 Hz).
int MLX90640_SetRefreshRate(uint8_t slaveAddr, uint8_t refreshRate);

// Set chess-board readout pattern (recommended).
int MLX90640_SetChessMode(uint8_t slaveAddr);

// Set interleaved readout pattern.
int MLX90640_SetInterleavedMode(uint8_t slaveAddr);

// Return current mode: 0 = chess, 1 = interleaved.
int MLX90640_GetCurMode(uint8_t slaveAddr);

// Set ADC resolution (0=16 bit … 3=19 bit).
int MLX90640_SetResolution(uint8_t slaveAddr, uint8_t resolution);

#ifdef __cplusplus
}
#endif
