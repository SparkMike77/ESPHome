// MLX90640 calibration algorithm — follows Melexis AN13877 Rev 7, Section 11.
// I²C calls go through mlx90640_i2c.h which uses Wire.h directly.
//
// EEPROM layout (word offsets from address 0x2400):
//  [0x00..0x0F]  offset row/col/global data
//  [0x10..0x1F]  alpha  row/col/global data
//  [0x20..0x2F]  CP (compensation pixel) data
//  [0x30..0x3F]  main calibration: Gain, PTAT, VDD, KsTo, TGC, …
//  [0x40..0x33F] per-pixel calibration, one 16-bit word per pixel

#include "mlx90640_api.h"
#include "mlx90640_i2c.h"
#include <math.h>
#include <string.h>

// ── Register addresses ──────────────────────────────────────────────────────
#define MLX_STATUS_REG    0x8000u
#define MLX_CTRL_REG      0x800Du
#define MLX_RAM_BASE      0x0400u
#define MLX_AUX_BASE      0x0700u
#define MLX_EE_BASE       0x2400u

// Indices into frameData[834]:
//  [0..767]   pixel RAM (0x0400..0x06FF)
//  [768..831] auxiliary RAM (0x0700..0x073F, 64 words)
//  [832]      status register snapshot
//  [833]      control register snapshot

#define FD_AUX(n)    (768 + (n))      // auxiliary word n  (0x0700+n)
#define FD_VPTAT     FD_AUX(0x20)     // 0x0720  VPTAT raw
#define FD_VBE       FD_AUX(0x00)     // 0x0700  Vbe
#define FD_GAIN      FD_AUX(0x0A)     // 0x070A  gain ADC reading
#define FD_VDD       FD_AUX(0x2A)     // 0x072A  Vdd pixel
#define FD_CP_SP0    FD_AUX(0x08)     // 0x0708  CP sub-page 0
#define FD_CP_SP1    FD_AUX(0x28)     // 0x0728  CP sub-page 1
#define FD_STATUS    832
#define FD_CTRL      833

// ── Helpers ─────────────────────────────────────────────────────────────────

static inline int16_t sign6(int v)  { return (int16_t)((v & 0x3F) - ((v & 0x20) ? 64 : 0)); }
static inline int16_t sign4(int v)  { return (int16_t)((v & 0x0F) - ((v & 0x08) ? 16 : 0)); }
static inline int16_t sign10(int v) { return (int16_t)((v & 0x3FF) - ((v & 0x200) ? 1024 : 0)); }
static inline int16_t sign8(int v)  { return (int8_t)(v & 0xFF); }

// ── Frame data ──────────────────────────────────────────────────────────────

int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t *eeData) {
    return MLX90640_I2CRead(slaveAddr, MLX_EE_BASE, MLX90640_EEPROM_WORDS, eeData);
}

int MLX90640_GetFrameData(uint8_t slaveAddr, uint16_t *frameData) {
    uint16_t statusReg;
    int retries = 200;

    // Wait for new-data bit (bit 3)
    do {
        if (MLX90640_I2CRead(slaveAddr, MLX_STATUS_REG, 1, &statusReg) != 0) return -1;
        if (--retries == 0) return -2;
    } while (!(statusReg & 0x0008u));

    // Acknowledge: clear bit 3
    if (MLX90640_I2CWrite(slaveAddr, MLX_STATUS_REG, 0x0030u) != 0) return -1;

    // Read 768 pixel words
    if (MLX90640_I2CRead(slaveAddr, MLX_RAM_BASE, 768, frameData) != 0) return -1;

    // Read 64 auxiliary words
    if (MLX90640_I2CRead(slaveAddr, MLX_AUX_BASE, 64, frameData + 768) != 0) return -1;

    // Read control register
    uint16_t ctrl;
    if (MLX90640_I2CRead(slaveAddr, MLX_CTRL_REG, 1, &ctrl) != 0) return -1;

    frameData[FD_STATUS] = statusReg;
    frameData[FD_CTRL]   = ctrl;

    return statusReg & 0x0001u;  // subpage
}

// ── VDD / Ta helpers ─────────────────────────────────────────────────────────

float MLX90640_GetVdd(uint16_t *frameData, const paramsMLX90640 *params) {
    int resRAM = (frameData[FD_CTRL] & 0x0C00u) >> 10;
    float resCorr = (float)(1 << params->resolutionEE) / (float)(1 << resRAM);
    return (resCorr * (int16_t)frameData[FD_VDD] - params->vdd25) / (float)params->kVdd + 3.3f;
}

float MLX90640_GetTa(uint16_t *frameData, const paramsMLX90640 *params) {
    float vdd = MLX90640_GetVdd(frameData, params);
    float ptat = (int16_t)frameData[FD_VPTAT];
    float vbe  = (int16_t)frameData[FD_VBE];
    float ptatArt = (ptat / (ptat * params->alphaPTAT + vbe)) * 262144.0f;
    float ta = ptatArt / (1.0f + params->KvPTAT * (vdd - 3.3f)) - (float)params->vPTAT25;
    return ta / params->KtPTAT + 25.0f;
}

// ── Parameter extraction ─────────────────────────────────────────────────────

static void extractVDD(uint16_t *ee, paramsMLX90640 *p) {
    // Word 0x33 (index 51): Kvdd[15:8] signed, Vdd25[7:0] unsigned
    int16_t kv = sign8(ee[51] >> 8);
    p->kVdd  = (int16_t)(kv * 32);
    p->vdd25 = (int16_t)(((ee[51] & 0xFF) - 256) << 5);
}

static void extractPTAT(uint16_t *ee, paramsMLX90640 *p) {
    // Word 0x32 (index 50): KvPTAT[15:10] 6-bit signed, KtPTAT[9:0] 10-bit signed
    float kv = (float)sign6((ee[50] & 0xFC00u) >> 10);
    p->KvPTAT = kv / 4096.0f;
    float kt = (float)sign10(ee[50] & 0x03FFu);
    p->KtPTAT = kt / 8.0f;
    p->vPTAT25 = ee[49];  // Word 0x31
    // alphaPTAT from word 0x10 (index 16), bits 15:12
    p->alphaPTAT = (float)((ee[16] & 0xF000u) >> 12) / 4.0f + 8.0f;
}

static void extractGain(uint16_t *ee, paramsMLX90640 *p) {
    p->gainEE = (int16_t)ee[48];  // Word 0x30
}

static void extractResolution(uint16_t *ee, paramsMLX90640 *p) {
    // Word 0x38 (index 56), bits 13:12
    p->resolutionEE = (ee[56] & 0x3000u) >> 12;
}

static void extractCalibrationMode(uint16_t *ee, paramsMLX90640 *p) {
    // Word 0x10 (index 16), bit 10: 0=chess, 1=IL
    p->calibrationModeEE = (ee[16] & 0x0400u) >> 10;
}

static void extractTgcKsTa(uint16_t *ee, paramsMLX90640 *p) {
    // Word 0x3C (index 60): KsTa[15:8] signed, TGC[7:0] signed
    p->KsTa = (float)sign8((ee[60] & 0xFF00u) >> 8) / 8192.0f;
    p->tgc  = (float)sign8(ee[60] & 0xFFu) / 32.0f;
}

static void extractKsTo(uint16_t *ee, paramsMLX90640 *p) {
    // Word 0x3F (index 63): CT step[13:12], CT3[11:8], CT2[7:4], KsTo scale[3:0]
    int step      = ((ee[63] & 0x3000u) >> 12) * 10;
    int ksToScale = (int)((ee[63] & 0x000Fu) + 8);
    float sc      = powf(2.0f, (float)ksToScale);

    p->ct[0] = -40;
    p->ct[1] = 0;
    p->ct[2] = (int16_t)(((ee[63] & 0x00F0u) >> 4) * step);
    p->ct[3] = (int16_t)((((ee[63] & 0x0F00u) >> 8) * step) + p->ct[2]);
    p->ct[4] = 400;

    // Words 0x3D..0x3E (indices 61, 62): KsTo[range 1..4]
    p->ksTo[0] = (float)sign8(ee[61] & 0xFFu) / sc;
    p->ksTo[1] = (float)sign8((ee[61] >> 8) & 0xFFu) / sc;
    p->ksTo[2] = (float)sign8(ee[62] & 0xFFu) / sc;
    p->ksTo[3] = (float)sign8((ee[62] >> 8) & 0xFFu) / sc;
    p->ksTo[4] = -0.0002f;
}

// Offset row (24) and column (32) accumulator corrections.
// Packed 4 per word (4-bit signed nibbles) starting at EE words 2..13.
static void extractOffsetRowCol(uint16_t *ee, int *accRow, int *accCol) {
    for (int i = 0; i < 6; i++) {
        int b = i * 4;
        uint16_t w = ee[2 + i];
        accRow[b+0] = sign4((w >> 12) & 0xF);
        accRow[b+1] = sign4((w >>  8) & 0xF);
        accRow[b+2] = sign4((w >>  4) & 0xF);
        accRow[b+3] = sign4( w        & 0xF);
    }
    for (int i = 0; i < 8; i++) {
        int b = i * 4;
        uint16_t w = ee[8 + i];
        accCol[b+0] = sign4((w >> 12) & 0xF);
        accCol[b+1] = sign4((w >>  8) & 0xF);
        accCol[b+2] = sign4((w >>  4) & 0xF);
        accCol[b+3] = sign4( w        & 0xF);
    }
}

// Alpha row/col accumulator corrections packed at EE words 18..31.
static void extractAlphaRowCol(uint16_t *ee, int *accRow, int *accCol) {
    for (int i = 0; i < 6; i++) {
        int b = i * 4;
        uint16_t w = ee[18 + i];
        accRow[b+0] = sign4((w >> 12) & 0xF);
        accRow[b+1] = sign4((w >>  8) & 0xF);
        accRow[b+2] = sign4((w >>  4) & 0xF);
        accRow[b+3] = sign4( w        & 0xF);
    }
    for (int i = 0; i < 8; i++) {
        int b = i * 4;
        uint16_t w = ee[24 + i];
        accCol[b+0] = sign4((w >> 12) & 0xF);
        accCol[b+1] = sign4((w >>  8) & 0xF);
        accCol[b+2] = sign4((w >>  4) & 0xF);
        accCol[b+3] = sign4( w        & 0xF);
    }
}

static void extractOffset(uint16_t *ee, paramsMLX90640 *p) {
    // Scale word at index 0: rem[3:0], col[7:4], row[11:8], global[15:12]
    int scaleGlobal = (ee[0] & 0xF000u) >> 12;
    int scaleRow    = (ee[0] & 0x0F00u) >> 8;
    int scaleCol    = (ee[0] & 0x00F0u) >> 4;
    int scaleRem    = (ee[0] & 0x000Fu);
    // Global offset reference at index 1
    int16_t offRef  = (int16_t)ee[1];

    int accRow[24], accCol[32];
    extractOffsetRowCol(ee, accRow, accCol);

    for (int i = 0; i < 768; i++) {
        int row = i / 32;
        int col = i % 32;
        int pixOsEE = sign6((ee[64 + i] & 0xFC00u) >> 10);

        int32_t raw = (int32_t)offRef
                    + accRow[row] * (1 << scaleRow)
                    + accCol[col] * (1 << scaleCol)
                    + pixOsEE    * (1 << scaleRem);
        p->offset[i] = (int16_t)(raw * (1 << scaleGlobal));
    }
}

static void extractAlpha(uint16_t *ee, paramsMLX90640 *p) {
    // Scale word at index 32: rem[3:0], col[7:4], row[11:8], scale[15:12]
    int scaleEE  = ((ee[32] & 0xF000u) >> 12) + 30;
    int scaleRow = (ee[32] & 0x0F00u) >> 8;
    int scaleCol = (ee[32] & 0x00F0u) >> 4;
    int scaleRem =  ee[32] & 0x000Fu;
    // Alpha reference at index 33
    uint16_t alphaRef = ee[33];

    int accRow[24], accCol[32];
    extractAlphaRowCol(ee, accRow, accCol);

    for (int i = 0; i < 768; i++) {
        int row = i / 32;
        int col = i % 32;
        int pixAlphaEE = (ee[64 + i] & 0x03C0u) >> 6;  // bits 9:6, unsigned

        int32_t raw = (int32_t)alphaRef
                    + accRow[row] * (1 << scaleRow)
                    + accCol[col] * (1 << scaleCol)
                    + pixAlphaEE  * (1 << scaleRem);
        p->alpha[i] = (uint16_t)(raw >> (scaleEE - 30));
    }
    p->alphaScale = (uint8_t)scaleEE;
}

static void extractKta(uint16_t *ee, paramsMLX90640 *p) {
    // Words 0x36..0x37 (indices 54, 55): Kta for four row/col parity combos
    // Word 0x38 (index 56): ktaScale1[7:4], ktaScale2[3:0]
    int8_t ktaBase[4];
    ktaBase[0] = (int8_t)((ee[54] & 0xFF00u) >> 8);   // row-even, col-even
    ktaBase[1] = (int8_t)(ee[54] & 0x00FFu);           // row-even, col-odd
    ktaBase[2] = (int8_t)((ee[55] & 0xFF00u) >> 8);   // row-odd,  col-even
    ktaBase[3] = (int8_t)(ee[55] & 0x00FFu);           // row-odd,  col-odd

    uint8_t ktaScale1 = (ee[56] & 0x00F0u) >> 4;
    uint8_t ktaScale2 =  ee[56] & 0x000Fu;

    p->ktaScale = ktaScale1;

    for (int i = 0; i < 768; i++) {
        int isOddRow = (i / 32) & 1;
        int isOddCol = (i % 32) & 1;
        int idx = isOddRow * 2 + isOddCol;

        int pixKtaEE = (int)(ee[64 + i] & 0x0030u) >> 4;
        if (pixKtaEE > 1) pixKtaEE -= 4;  // 2-bit signed

        int32_t kta = (int32_t)ktaBase[idx] * (1 << ktaScale2) + pixKtaEE;
        p->kta[i] = (int8_t)(kta / (1 << ktaScale2));
    }
}

static void extractKv(uint16_t *ee, paramsMLX90640 *p) {
    // Word 0x34 (index 52): Kv for four row/col parity combinations (4-bit signed each)
    // Word 0x38 (index 56): kvScale[11:8]
    int8_t kvBase[4];
    kvBase[0] = (int8_t)sign4((ee[52] & 0xF000u) >> 12);  // row-even, col-even
    kvBase[1] = (int8_t)sign4((ee[52] & 0x0F00u) >>  8);  // row-even, col-odd
    kvBase[2] = (int8_t)sign4((ee[52] & 0x00F0u) >>  4);  // row-odd,  col-even
    kvBase[3] = (int8_t)sign4( ee[52] & 0x000Fu);         // row-odd,  col-odd

    uint8_t kvScale = (ee[56] & 0x0F00u) >> 8;
    p->kvScale = kvScale;

    for (int i = 0; i < 768; i++) {
        int isOddRow = (i / 32) & 1;
        int isOddCol = (i % 32) & 1;
        int kvEE     = kvBase[isOddRow * 2 + isOddCol];

        int pixKvEE = (int)(ee[64 + i] & 0x000Eu) >> 1;
        if (pixKvEE > 3) pixKvEE -= 8;  // 3-bit signed

        p->kv[i] = (int8_t)(kvEE + pixKvEE);
    }
}

static void extractCP(uint16_t *ee, paramsMLX90640 *p) {
    // Alpha: word 0x39 (index 57), scale from EE[32][15:12]+27
    int alphaScaleCP = ((ee[32] & 0xF000u) >> 12) + 27;
    float alphaSP0 = (float)(ee[57] & 0x03FFu) / powf(2.0f, (float)alphaScaleCP);
    float alphaRatio = (float)sign6((ee[57] & 0xFC00u) >> 10);
    float alphaSP1   = alphaSP0 * (1.0f + alphaRatio / 128.0f);

    p->cpAlpha[0] = alphaSP0;
    p->cpAlpha[1] = alphaSP1;

    // Offset: word 0x3A (index 58)
    p->cpOffset[0] = sign10(ee[58] & 0x03FFu);
    int16_t cpOffDelta = sign6((ee[58] & 0xFC00u) >> 10);
    p->cpOffset[1] = (int16_t)(p->cpOffset[0] + cpOffDelta);

    // Kta: word 0x3B (index 59) low byte, scale from EE[56][7:4]
    int ktaScaleCP = ((ee[56] & 0x00F0u) >> 4) + 8;
    p->cpKta = (float)sign8(ee[59] & 0xFFu) / powf(2.0f, (float)ktaScaleCP);

    // Kv: word 0x3B (index 59) high byte, scale from EE[56][11:8]
    int kvScaleCP = ((ee[56] & 0x0F00u) >> 8) + 8;  // typical +8 for KvCP
    p->cpKv = (float)sign8((ee[59] >> 8) & 0xFFu) / powf(2.0f, (float)kvScaleCP);
}

static void extractILChess(uint16_t *ee, paramsMLX90640 *p) {
    // IL chess compensation (used when calibrationModeEE==0, chess)
    // Words 0x1C..0x1E (indices 28..30)
    p->ilChessC[0] = (float)sign6((ee[28] & 0x003Fu)) / 16.0f;
    p->ilChessC[1] = (float)sign6((ee[28] & 0x0FC0u) >> 6) / 2.0f;
    p->ilChessC[2] = (float)sign6((ee[28] & 0x3F00u) >> 8) / 8.0f;
}

static void extractDeviating(uint16_t *ee, paramsMLX90640 *p) {
    // Scan pixel words for outlier or broken flags.
    // Bit 0 of pixel EE word = outlier, value 0x7FFF = broken.
    int brokenCnt = 0, outlierCnt = 0;
    for (int i = 0; i < 5; i++) {
        p->brokenPixels[i]  = 0xFFFFu;
        p->outlierPixels[i] = 0xFFFFu;
    }
    for (int i = 0; i < 768; i++) {
        if (ee[64 + i] == 0x7FFFu) {
            if (brokenCnt < 5) p->brokenPixels[brokenCnt++] = (uint16_t)i;
        } else if (ee[64 + i] & 0x0001u) {
            if (outlierCnt < 5) p->outlierPixels[outlierCnt++] = (uint16_t)i;
        }
    }
}

int MLX90640_ExtractParameters(uint16_t *eeData, paramsMLX90640 *params) {
    memset(params, 0, sizeof(*params));
    extractVDD(eeData, params);
    extractPTAT(eeData, params);
    extractGain(eeData, params);
    extractResolution(eeData, params);
    extractCalibrationMode(eeData, params);
    extractTgcKsTa(eeData, params);
    extractKsTo(eeData, params);
    extractOffset(eeData, params);
    extractAlpha(eeData, params);
    extractKta(eeData, params);
    extractKv(eeData, params);
    extractCP(eeData, params);
    extractILChess(eeData, params);
    extractDeviating(eeData, params);
    return 0;
}

// ── Temperature calculation ──────────────────────────────────────────────────

void MLX90640_CalculateTo(uint16_t *frameData, const paramsMLX90640 *params,
                           float emissivity, float tr, float *result) {
    float vdd  = MLX90640_GetVdd(frameData, params);
    float ta   = MLX90640_GetTa(frameData, params);

    float ta4  = (ta  + 273.15f); ta4 *= ta4; ta4 *= ta4;
    float tr4  = (tr  + 273.15f); tr4 *= tr4; tr4 *= tr4;
    float taTr = tr4 - (tr4 - ta4) / emissivity;

    float ktaScale   = powf(2.0f, (float)params->ktaScale);
    float kvScale    = powf(2.0f, (float)params->kvScale);
    float alphaScale = powf(2.0f, (float)params->alphaScale - 27);

    // Global gain
    float gain = (float)params->gainEE / (float)(int16_t)frameData[FD_GAIN];

    // Compensation pixel (removes chip self-emission)
    float cpSP[2];
    cpSP[0] = (int16_t)frameData[FD_CP_SP0] * gain
              - (float)params->cpOffset[0]
                * (1.0f + params->cpKta * (ta - 25.0f))
                * (1.0f + params->cpKv  * (vdd - 3.3f));
    cpSP[1] = (int16_t)frameData[FD_CP_SP1] * gain
              - (float)params->cpOffset[1]
                * (1.0f + params->cpKta * (ta - 25.0f))
                * (1.0f + params->cpKv  * (vdd - 3.3f));

    for (int pix = 0; pix < 768; pix++) {
        // Which sub-page this pixel belongs to (chess mode: checkerboard pattern)
        int subpage;
        if (params->calibrationModeEE == 0) {
            subpage = ((pix / 32) + (pix % 32)) & 1;  // chess board
        } else {
            subpage = (pix / 32) & 1;                  // interleaved rows
        }

        float kta = (float)params->kta[pix] / ktaScale;
        float kv  = (float)params->kv[pix]  / kvScale;

        float pixGain = (float)(int16_t)frameData[pix] * gain;
        float pixOs   = pixGain
                      - (float)params->offset[pix]
                        * (1.0f + kta * (ta - 25.0f))
                        * (1.0f + kv  * (vdd - 3.3f));

        // Remove chip self-emission via TGC correction
        pixOs -= params->tgc * cpSP[subpage];
        pixOs /= emissivity;

        float alphaCorr = alphaScale * (float)params->alpha[pix]
                        * (1.0f + params->KsTa * (ta - 25.0f));

        if (alphaCorr == 0.0f) { result[pix] = -273.15f; continue; }

        float Sx = sqrtf(sqrtf(alphaCorr * alphaCorr * alphaCorr
                               * (pixOs + alphaCorr * taTr)));

        // Basic object temperature (valid for ct[1]..ct[2] range, i.e. 0..ct[2] °C)
        float To = sqrtf(sqrtf(
                     pixOs / (alphaCorr * (1.0f - params->ksTo[1] * 273.15f)
                              + Sx * params->ksTo[1])
                     + taTr))
                 - 273.15f;

        // Range extension for temperatures outside the basic range
        if (To < (float)params->ct[1]) {
            float Sx2 = sqrtf(sqrtf(alphaCorr * alphaCorr * alphaCorr
                        * (pixOs + alphaCorr * powf((float)params->ct[1] + 273.15f, 4.0f))));
            To = sqrtf(sqrtf(
                   pixOs / (alphaCorr * (1.0f - params->ksTo[0] * 273.15f) + Sx2 * params->ksTo[0])
                   + powf((float)params->ct[1] + 273.15f, 4.0f)))
               - 273.15f;
        } else if (To > (float)params->ct[2]) {
            float Sx2 = sqrtf(sqrtf(alphaCorr * alphaCorr * alphaCorr
                        * (pixOs + alphaCorr * powf((float)params->ct[2] + 273.15f, 4.0f))));
            To = sqrtf(sqrtf(
                   pixOs / (alphaCorr * (1.0f - params->ksTo[2] * 273.15f) + Sx2 * params->ksTo[2])
                   + powf((float)params->ct[2] + 273.15f, 4.0f)))
               - 273.15f;
        }

        result[pix] = To;
    }
}

// ── Control register helpers ─────────────────────────────────────────────────

int MLX90640_SetRefreshRate(uint8_t slaveAddr, uint8_t refreshRate) {
    uint16_t ctrl;
    if (MLX90640_I2CRead(slaveAddr, MLX_CTRL_REG, 1, &ctrl) != 0) return -1;
    ctrl = (ctrl & 0xFC7Fu) | ((uint16_t)(refreshRate & 0x07u) << 7);
    return MLX90640_I2CWrite(slaveAddr, MLX_CTRL_REG, ctrl);
}

int MLX90640_SetChessMode(uint8_t slaveAddr) {
    uint16_t ctrl;
    if (MLX90640_I2CRead(slaveAddr, MLX_CTRL_REG, 1, &ctrl) != 0) return -1;
    return MLX90640_I2CWrite(slaveAddr, MLX_CTRL_REG, ctrl | 0x1000u);
}

int MLX90640_SetInterleavedMode(uint8_t slaveAddr) {
    uint16_t ctrl;
    if (MLX90640_I2CRead(slaveAddr, MLX_CTRL_REG, 1, &ctrl) != 0) return -1;
    return MLX90640_I2CWrite(slaveAddr, MLX_CTRL_REG, ctrl & ~0x1000u);
}

int MLX90640_GetCurMode(uint8_t slaveAddr) {
    uint16_t ctrl;
    if (MLX90640_I2CRead(slaveAddr, MLX_CTRL_REG, 1, &ctrl) != 0) return -1;
    return (ctrl & 0x1000u) ? 0 : 1;  // 0=chess, 1=IL
}

int MLX90640_SetResolution(uint8_t slaveAddr, uint8_t resolution) {
    uint16_t ctrl;
    if (MLX90640_I2CRead(slaveAddr, MLX_CTRL_REG, 1, &ctrl) != 0) return -1;
    ctrl = (ctrl & 0xF3FFu) | ((uint16_t)(resolution & 0x03u) << 10);
    return MLX90640_I2CWrite(slaveAddr, MLX_CTRL_REG, ctrl);
}
