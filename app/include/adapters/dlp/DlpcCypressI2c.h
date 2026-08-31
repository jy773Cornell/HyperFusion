// Cypress USB-Serial I2C for DLPC3478 (adapters/dlp).
// Matches DLPC-API-1.12 samples/cypress_i2c.c (GPIO 5/6/9, slave 0x36>>1, 100 kHz). Adds CyClose.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    HF_CYPRESS_I2C_ID_CHARS = 256
};

typedef struct HfCypressI2cDeviceInfo
{
    char id[HF_CYPRESS_I2C_ID_CHARS];
    char model[HF_CYPRESS_I2C_ID_CHARS];
} HfCypressI2cDeviceInfo;

int HfCypressI2cEnumerate(HfCypressI2cDeviceInfo *out, int maxCount);

bool HfCypressI2cConnect(const char *deviceId);
bool HfCypressI2cRequestBus(char *error, int errorBytes);
bool HfCypressI2cRelinquishBus(void);
void HfCypressI2cRecoverTransaction(void);
bool HfCypressI2cWrite(uint32_t writeLength, uint8_t *writeData);
bool HfCypressI2cRead(uint32_t readLength, uint8_t *readData);
bool HfCypressI2cWriteRead(uint32_t writeLength, uint8_t *writeData, uint32_t readLength, uint8_t *readData);
int HfCypressI2cLastCyStatus(void);
void HfCypressI2cClose(void);

#ifdef __cplusplus
}
#endif
