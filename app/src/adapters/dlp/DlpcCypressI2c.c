// Cypress USB-Serial I2C (adapters/dlp). Same handshake as TI cypress_i2c.c; CyClose on teardown.
#include "adapters/dlp/DlpcCypressI2c.h"

#include "cyusbserial/CyUSBSerial.h"

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
__declspec(dllimport) void __stdcall Sleep(unsigned long dwMilliseconds);
__declspec(dllimport) unsigned long __stdcall GetTickCount(void);
#endif

#define REQUEST_I2C_ACCESS_GPIO 5
#define I2C_ACCESS_GRANTED_GPIO 6
#define START_I2C_TRANSACTION_GPIO 9
#define I2C_CLOCK_FREQUENCY_HZ 100000
#define DLP_I2C_SLAVE_ADDRESS (0x36 >> 1)
#define I2C_TIMEOUT_MILLISECONDS 2000
#define I2C_HANDSHAKE_TIMEOUT_MS 10000

static CY_HANDLE s_Handle = NULL;
static CY_I2C_DATA_CONFIG s_DataConfig;
static int s_Opened = 0;
static int s_LastCyStatus = 0;

static void copyUcharField(char *dst, size_t dstSize, const UCHAR *src)
{
    size_t i = 0;
    if (dst == NULL || dstSize == 0)
        return;
    if (src == NULL)
    {
        dst[0] = '\0';
        return;
    }
    for (; i + 1 < dstSize && src[i] != 0; ++i)
        dst[i] = (char)src[i];
    dst[i] = '\0';
}

static void makeDeviceId(char *dst, size_t dstSize, UINT8 deviceIdx, const UCHAR *serial)
{
    char serialBuf[HF_CYPRESS_I2C_ID_CHARS];
    copyUcharField(serialBuf, sizeof(serialBuf), serial);
    if (serialBuf[0] != '\0')
        snprintf(dst, dstSize, "%s", serialBuf);
    else
        snprintf(dst, dstSize, "i2c-%u", (unsigned)deviceIdx);
}

static bool idsMatch(const char *requested, UINT8 deviceIdx, const UCHAR *serial)
{
    char actual[HF_CYPRESS_I2C_ID_CHARS];
    if (requested == NULL || requested[0] == '\0')
        return 1;
    makeDeviceId(actual, sizeof(actual), deviceIdx, serial);
    return strcmp(requested, actual) == 0;
}

static bool getCyI2CHandle(CY_HANDLE *handle, const char *deviceId)
{
    CY_RETURN_STATUS status;
    CY_DEVICE_INFO deviceInfo;
    UINT8 numDevices = 0;
    UINT8 deviceIdx;
    UINT8 interfaceIdx;

    status = CyGetListofDevices(&numDevices);
    if ((status != CY_SUCCESS) || (numDevices == 0))
        return 0;

    for (deviceIdx = 0; deviceIdx < numDevices; deviceIdx++)
    {
        status = CyGetDeviceInfo(deviceIdx, &deviceInfo);
        if (status != CY_SUCCESS)
            continue;

        if (!idsMatch(deviceId, deviceIdx, deviceInfo.serialNum))
            continue;

        for (interfaceIdx = 0; interfaceIdx < deviceInfo.numInterfaces; interfaceIdx++)
        {
            if (deviceInfo.deviceType[interfaceIdx] != CY_TYPE_I2C)
                continue;
            /* Windows: each interface is its own device instance; interfaceNum must be 0. */
            status = CyOpen(deviceIdx, 0, handle);
            if (status == CY_SUCCESS)
                return 1;
        }
    }

    return 0;
}

static bool setGpio(UINT8 gpioNum, UINT8 value)
{
    if (!s_Opened)
        return 0;
    return CySetGpioValue(s_Handle, gpioNum, value) == CY_SUCCESS;
}

static bool getGpio(UINT8 gpioNum, UINT8 *value)
{
    if (!s_Opened)
        return 0;
    return CyGetGpioValue(s_Handle, gpioNum, value) == CY_SUCCESS;
}

int HfCypressI2cEnumerate(HfCypressI2cDeviceInfo *out, int maxCount)
{
    CY_RETURN_STATUS status;
    CY_DEVICE_INFO deviceInfo;
    UINT8 numDevices = 0;
    UINT8 deviceIdx;
    UINT8 interfaceIdx;
    int count = 0;
    int attempt = 0;

    if (out == NULL || maxCount <= 0)
        return 0;

    for (attempt = 0; attempt < 3; ++attempt)
    {
        numDevices = 0;
        status = CyGetListofDevices(&numDevices);
        if (status == CY_SUCCESS && numDevices > 0)
            break;
        Sleep(250);
    }
    if ((status != CY_SUCCESS) || (numDevices == 0))
        return 0;

    for (deviceIdx = 0; deviceIdx < numDevices && count < maxCount; deviceIdx++)
    {
        int hasI2c = 0;
        status = CyGetDeviceInfo(deviceIdx, &deviceInfo);
        if (status != CY_SUCCESS)
            continue;

        for (interfaceIdx = 0; interfaceIdx < deviceInfo.numInterfaces; interfaceIdx++)
        {
            if (deviceInfo.deviceType[interfaceIdx] == CY_TYPE_I2C)
            {
                hasI2c = 1;
                break;
            }
        }
        if (!hasI2c)
            continue;

        makeDeviceId(out[count].id, sizeof(out[count].id), deviceIdx, deviceInfo.serialNum);
        copyUcharField(out[count].model, sizeof(out[count].model), deviceInfo.productName);
        if (out[count].model[0] == '\0')
            snprintf(out[count].model, sizeof(out[count].model), "DLP3010EVM-LC");
        ++count;
    }

    return count;
}

bool HfCypressI2cConnect(const char *deviceId)
{
    CY_RETURN_STATUS status;
    CY_I2C_CONFIG i2cConfig;

    HfCypressI2cClose();

    if (!getCyI2CHandle(&s_Handle, deviceId))
        return 0;

    s_Opened = 1;

    i2cConfig.frequency = I2C_CLOCK_FREQUENCY_HZ;
    i2cConfig.slaveAddress = 0x30;
    i2cConfig.isMaster = true;
    i2cConfig.isClockStretch = false;

    status = CySetI2cConfig(s_Handle, &i2cConfig);
    if (status != CY_SUCCESS)
    {
        HfCypressI2cClose();
        return 0;
    }

    s_DataConfig.isNakBit = true;
    s_DataConfig.isStopBit = true;
    s_DataConfig.slaveAddress = DLP_I2C_SLAVE_ADDRESS;
    Sleep(100);
    return 1;
}

bool HfCypressI2cRequestBus(char *error, int errorBytes)
{
    UINT8 value = 0;
    const unsigned long start = GetTickCount();

    (void)setGpio(START_I2C_TRANSACTION_GPIO, 0);
    (void)setGpio(REQUEST_I2C_ACCESS_GPIO, 0);
    Sleep(50);

    if (!setGpio(REQUEST_I2C_ACCESS_GPIO, 1))
    {
        if (error != NULL && errorBytes > 0)
            snprintf(error, (size_t)errorBytes, "GPIO %u (I2C request) set failed.", REQUEST_I2C_ACCESS_GPIO);
        return 0;
    }

    /* TI sample compares time() to I2C_TIMEOUT_MILLISECONDS as if it were seconds.
     * Wait 10 s wall time and treat any non-zero grant as success. */
    while ((GetTickCount() - start) < I2C_HANDSHAKE_TIMEOUT_MS)
    {
        if (getGpio(I2C_ACCESS_GRANTED_GPIO, &value) && value != 0)
        {
            if (!setGpio(START_I2C_TRANSACTION_GPIO, 1))
            {
                if (error != NULL && errorBytes > 0)
                    snprintf(error, (size_t)errorBytes,
                             "GPIO %u (start transaction) set failed.",
                             START_I2C_TRANSACTION_GPIO);
                return 0;
            }

            CyI2cReset(s_Handle, false);
            CyI2cReset(s_Handle, true);
            /* DLPC can still be booting after PROJ_ON; TI sample has no delay but we need one. */
            Sleep(500);
            return 1;
        }
        Sleep(10);
    }

    if (error != NULL && errorBytes > 0)
    {
        snprintf(error, (size_t)errorBytes,
                 "EVM MCU did not grant I2C (GPIO%u=%u after %u ms). Power-cycle the EVM, then retry.",
                 I2C_ACCESS_GRANTED_GPIO, (unsigned)value, I2C_HANDSHAKE_TIMEOUT_MS);
    }
    return 0;
}

bool HfCypressI2cRelinquishBus(void)
{
    return setGpio(REQUEST_I2C_ACCESS_GPIO, 0) && setGpio(START_I2C_TRANSACTION_GPIO, 0);
}

void HfCypressI2cRecoverTransaction(void)
{
    if (!s_Opened)
        return;
    CyI2cReset(s_Handle, false);
    CyI2cReset(s_Handle, true);
    (void)setGpio(START_I2C_TRANSACTION_GPIO, 0);
    Sleep(20);
    (void)setGpio(START_I2C_TRANSACTION_GPIO, 1);
    Sleep(50);
}

bool HfCypressI2cWrite(uint32_t writeLength, uint8_t *writeData)
{
    CY_DATA_BUFFER writeBuffer;
    CY_RETURN_STATUS status;

    if (!s_Opened)
        return 0;

    writeBuffer.buffer = writeData;
    writeBuffer.length = writeLength;
    writeBuffer.transferCount = 0;

    status = CyI2cWrite(s_Handle, &s_DataConfig, &writeBuffer, I2C_TIMEOUT_MILLISECONDS);
    s_LastCyStatus = (int)status;
    if (status != CY_SUCCESS)
    {
        CyI2cReset(s_Handle, false);
        CyI2cReset(s_Handle, true);
        return 0;
    }
    return 1;
}

bool HfCypressI2cRead(uint32_t readLength, uint8_t *readData)
{
    CY_DATA_BUFFER readBuffer;
    CY_RETURN_STATUS status;

    if (!s_Opened)
        return 0;

    readBuffer.buffer = readData;
    readBuffer.length = readLength;
    readBuffer.transferCount = 0;

    status = CyI2cRead(s_Handle, &s_DataConfig, &readBuffer, I2C_TIMEOUT_MILLISECONDS);
    s_LastCyStatus = (int)status;
    /* Cypress often returns IO_TIMEOUT even when bytes arrived. Require the payload. */
    if ((status == CY_SUCCESS || status == CY_ERROR_IO_TIMEOUT)
        && readBuffer.transferCount >= readLength)
        return 1;

    CyI2cReset(s_Handle, false);
    CyI2cReset(s_Handle, true);
    return 0;
}

bool HfCypressI2cWriteRead(uint32_t writeLength, uint8_t *writeData, uint32_t readLength, uint8_t *readData)
{
    /* TI cypress_i2c.c: write with STOP, then a separate read (not repeated-start). */
    s_DataConfig.isStopBit = true;
    s_DataConfig.isNakBit = true;
    if (!HfCypressI2cWrite(writeLength, writeData))
        return 0;
    return HfCypressI2cRead(readLength, readData);
}

int HfCypressI2cLastCyStatus(void)
{
    return s_LastCyStatus;
}

void HfCypressI2cClose(void)
{
    if (!s_Opened)
        return;

    (void)HfCypressI2cRelinquishBus();
    (void)CyClose(s_Handle);
    s_Handle = NULL;
    s_Opened = 0;
}
