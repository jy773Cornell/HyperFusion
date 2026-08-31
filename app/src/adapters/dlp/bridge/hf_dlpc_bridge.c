// 32-bit DLPC3478 I2C helper (adapters/dlp). Links TI x86 cyusbserial. No light until ARM/PATTERN.
#include "adapters/dlp/DlpcCypressI2c.h"

#include "dlpc_common.h"
#include "dlpc34xx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
__declspec(dllimport) void __stdcall Sleep(unsigned long dwMilliseconds);
#endif

static uint8_t s_writeBuffer[1024 + 8];
static uint8_t s_readBuffer[256 + 8];
static int s_busHeld = 0;
static int s_libraryReady = 0;

static uint32_t writeI2c(uint16_t writeLength,
                         uint8_t *writeBuffer,
                         DLPC_COMMON_CommandProtocolData_s *protocolData)
{
    (void)protocolData;
    if (!HfCypressI2cWrite(writeLength, writeBuffer))
        return FAIL;
    return SUCCESS;
}

static uint32_t readI2c(uint16_t writeLength,
                        uint8_t *writeBuffer,
                        uint16_t readLength,
                        uint8_t *readBuffer,
                        DLPC_COMMON_CommandProtocolData_s *protocolData)
{
    (void)protocolData;
    if (!HfCypressI2cWriteRead(writeLength, writeBuffer, readLength, readBuffer))
        return FAIL;
    return SUCCESS;
}

static void initLibrary(void)
{
    if (s_libraryReady)
        return;
    DLPC_COMMON_InitCommandLibrary(s_writeBuffer,
                                   (uint16_t)sizeof(s_writeBuffer),
                                   s_readBuffer,
                                   (uint16_t)sizeof(s_readBuffer),
                                   writeI2c,
                                   readI2c);
    s_libraryReady = 1;
}

static void replyOk(void)
{
    fputs("OK\n", stdout);
    fflush(stdout);
}

static void replyErr(const char *message)
{
    fprintf(stdout, "ERR %s\n", message ? message : "error");
    fflush(stdout);
}

static int check(uint32_t status, const char *what)
{
    if (status == SUCCESS)
        return 1;
    fprintf(stdout, "ERR %s failed (status %u)\n", what, (unsigned)status);
    fflush(stdout);
    return 0;
}

static void teardownUsb(void)
{
    if (s_busHeld)
    {
        (void)HfCypressI2cRelinquishBus();
        s_busHeld = 0;
    }
    HfCypressI2cClose();
}

static void cmdEnum(void)
{
    HfCypressI2cDeviceInfo devices[8];
    const int count = HfCypressI2cEnumerate(devices, 8);
    int i = 0;
    fprintf(stdout, "OK %d\n", count < 0 ? 0 : count);
    for (; i < count; ++i)
        fprintf(stdout, "DEV %s\t%s\n", devices[i].id, devices[i].model);
    fflush(stdout);
}

static int readChipId(DLPC34XX_ControllerDeviceId_e *chip)
{
    int attempt = 0;
    for (attempt = 0; attempt < 3; ++attempt)
    {
        *chip = (DLPC34XX_ControllerDeviceId_e)0;
        if (DLPC34XX_ReadControllerDeviceId(chip) == SUCCESS && (unsigned)*chip != 0)
            return 1;
        HfCypressI2cRecoverTransaction();
        Sleep(150);
    }
    return 0;
}

static void cmdConnect(const char *deviceId)
{
    DLPC34XX_ControllerDeviceId_e chip = DLPC34XX_CDI_DLPC3430;
    char handshakeErr[192];
    int cycle = 0;
    const char *id = (deviceId != NULL && deviceId[0] != '\0') ? deviceId : NULL;

    initLibrary();

    for (cycle = 0; cycle < 2; ++cycle)
    {
        teardownUsb();
        s_busHeld = 0;
        handshakeErr[0] = '\0';

        if (cycle > 0)
            Sleep(400);

        if (!HfCypressI2cConnect(id))
        {
            replyErr("Cypress USB-Serial I2C open failed. Close the TI DLP GUI and retry.");
            return;
        }

        if (!HfCypressI2cRequestBus(handshakeErr, (int)sizeof(handshakeErr)))
        {
            teardownUsb();
            if (cycle == 0)
                continue;
            replyErr(handshakeErr[0] != '\0' ? handshakeErr : "I2C bus handshake failed.");
            return;
        }
        s_busHeld = 1;

        if (readChipId(&chip))
        {
            fprintf(stdout, "OK chip=0x%X\n", (unsigned)chip);
            fflush(stdout);
            return;
        }
    }

    {
        char i2cErr[192];
        snprintf(i2cErr, sizeof(i2cErr),
                 "ReadControllerDeviceId failed (cypress status %d). Confirm SW_ONOFF ON and PROJ_ON LED D3.",
                 HfCypressI2cLastCyStatus());
        teardownUsb();
        replyErr(i2cErr);
    }
}

/* DLPA2005 IDAC: 2528 mA at 0x3FF (~2.47 mA/LSB). GUI values are milliamps. */
static uint16_t milliampToIdac(int ma)
{
    long code = 0;
    if (ma <= 0)
        return 0;
    code = ((long)ma * 1023 + 1264) / 2528;
    if (code > 1023)
        code = 1023;
    return (uint16_t)code;
}

static int writeLedCurrents(int redMa, int greenMa, int blueMa)
{
    const uint16_t r = milliampToIdac(redMa);
    const uint16_t g = milliampToIdac(greenMa);
    const uint16_t b = milliampToIdac(blueMa);

    if (!check(DLPC34XX_WriteLedOutputControlMethod(DLPC34XX_LCM_MANUAL),
               "WriteLedOutputControlMethod"))
        return 0;
    return check(DLPC34XX_WriteRgbLedCurrent(r, g, b), "WriteRgbLedCurrent");
}

static void cmdArm(int redMa, int greenMa, int blueMa)
{
    if (!writeLedCurrents(redMa, greenMa, blueMa))
        return;
    if (!check(DLPC34XX_WriteDisplayImageCurtain(DLPC34XX_ICE_ENABLE, DLPC34XX_C_BLACK),
               "WriteDisplayImageCurtain"))
        return;
    if (!check(DLPC34XX_WriteRgbLedEnable(1, 1, 1), "WriteRgbLedEnable"))
        return;
    replyOk();
}

static void cmdLed(int redMa, int greenMa, int blueMa)
{
    if (!writeLedCurrents(redMa, greenMa, blueMa))
        return;
    replyOk();
}

static void cmdBlank(void)
{
    if (!check(DLPC34XX_WriteDisplayImageCurtain(DLPC34XX_ICE_ENABLE, DLPC34XX_C_BLACK),
               "WriteDisplayImageCurtain"))
        return;
    if (!check(DLPC34XX_WriteRgbLedEnable(0, 0, 0), "WriteRgbLedEnable"))
        return;
    replyOk();
}

static int writeGeometry(uint16_t startPixel)
{
    const uint16_t width = DLP3010_WIDTH;
    const uint16_t height = DLP3010_HEIGHT;
    uint16_t cropWidth = width;
    if (startPixel >= width)
        startPixel = 0;
    cropWidth = (uint16_t)(width - startPixel);
    if (!check(DLPC34XX_WriteInputImageSize(width, height), "WriteInputImageSize"))
        return 0;
    if (!check(DLPC34XX_WriteImageCrop(startPixel, 0, cropWidth, height), "WriteImageCrop"))
        return 0;
    return check(DLPC34XX_WriteDisplaySize(0, 0, cropWidth, height), "WriteDisplaySize");
}

static int writeFppVerticalLines(unsigned width, int phaseDeg)
{
    DLPC34XX_VerticalLines_s lines;
    uint16_t offset = 0;
    int inverted = 0;

    memset(&lines, 0, sizeof(lines));
    if (width < 1)
        width = 1;
    if (width > 255)
        width = 255;

    if (phaseDeg == 90 || phaseDeg == 270)
        offset = (uint16_t)(width / 2);
    if (phaseDeg == 180 || phaseDeg == 270)
        inverted = 1;

    if (!writeGeometry(offset))
        return 0;

    lines.Border = DLPC34XX_BE_DISABLE;
    if (inverted)
    {
        lines.BackgroundColor = DLPC34XX_C_WHITE;
        lines.ForegroundColor = DLPC34XX_C_BLACK;
    }
    else
    {
        lines.BackgroundColor = DLPC34XX_C_BLACK;
        lines.ForegroundColor = DLPC34XX_C_WHITE;
    }
    lines.ForegroundLineWidth = (uint8_t)width;
    lines.BackgroundLineWidth = (uint8_t)width;
    return check(DLPC34XX_WriteVerticalLines(&lines), "WriteVerticalLines");
}

static int writePattern(const char *name)
{
    if (_strnicmp(name, "FPP lines ", 10) == 0)
    {
        unsigned width = 0;
        char extra[16];
        int phase = 0;
        extra[0] = '\0';
        if (sscanf(name + 10, "%u %15s", &width, extra) < 1)
        {
            replyErr("FPP lines needs a stripe width");
            return 0;
        }
        if (_stricmp(extra, "90") == 0)
            phase = 90;
        else if (_stricmp(extra, "180") == 0 || _stricmp(extra, "inv") == 0)
            phase = 180;
        else if (_stricmp(extra, "270") == 0)
            phase = 270;
        return writeFppVerticalLines(width, phase);
    }
    if (_stricmp(name, "Checkerboard") == 0)
    {
        DLPC34XX_Checkerboard_s checker;
        memset(&checker, 0, sizeof(checker));
        checker.Border = DLPC34XX_BE_ENABLE;
        checker.BackgroundColor = DLPC34XX_C_BLACK;
        checker.ForegroundColor = DLPC34XX_C_WHITE;
        checker.HorizontalCheckerCount = 16;
        checker.VerticalCheckerCount = 9;
        return check(DLPC34XX_WriteCheckerboard(&checker), "WriteCheckerboard");
    }
    if (_stricmp(name, "Horizontal ramp") == 0)
    {
        return check(DLPC34XX_WriteHorizontalRamp(DLPC34XX_BE_ENABLE, DLPC34XX_C_WHITE, 0, 255),
                     "WriteHorizontalRamp");
    }
    if (_stricmp(name, "Vertical ramp") == 0)
    {
        return check(DLPC34XX_WriteVerticalRamp(DLPC34XX_BE_ENABLE, DLPC34XX_C_WHITE, 0, 255),
                     "WriteVerticalRamp");
    }
    if (_stricmp(name, "Solid field") == 0)
        return check(DLPC34XX_WriteSolidField(DLPC34XX_BE_ENABLE, DLPC34XX_C_WHITE), "WriteSolidField");
    if (_stricmp(name, "Color bars") == 0)
        return check(DLPC34XX_WriteColorbars(DLPC34XX_BE_ENABLE), "WriteColorbars");

    replyErr("Unknown test pattern");
    return 0;
}

static void cmdPattern(const char *name)
{
    if (!writeGeometry(0))
        return;
    if (!writePattern(name))
        return;
    if (!check(DLPC34XX_WriteOperatingModeSelect(DLPC34XX_OM_TEST_PATTERN_GENERATOR),
               "WriteOperatingModeSelect"))
        return;
    if (!check(DLPC34XX_WriteDisplayImageCurtain(DLPC34XX_ICE_DISABLE, DLPC34XX_C_BLACK),
               "WriteDisplayImageCurtain"))
        return;
    if (!check(DLPC34XX_WriteRgbLedEnable(1, 1, 1), "WriteRgbLedEnable"))
        return;
    replyOk();
}

static void cmdDisconnect(void)
{
    if (s_busHeld)
    {
        (void)DLPC34XX_WriteDisplayImageCurtain(DLPC34XX_ICE_ENABLE, DLPC34XX_C_BLACK);
        (void)DLPC34XX_WriteRgbLedEnable(0, 0, 0);
    }
    teardownUsb();
    replyOk();
}

static char *trim(char *s)
{
    char *end = NULL;
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')
        ++s;
    if (*s == '\0')
        return s;
    end = s + strlen(s) - 1;
    while (end > s && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n'))
    {
        *end = '\0';
        --end;
    }
    return s;
}

int main(void)
{
    char line[512];

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    while (fgets(line, (int)sizeof(line), stdin) != NULL)
    {
        char *cmd = trim(line);
        if (cmd[0] == '\0')
            continue;

        if (_stricmp(cmd, "ENUM") == 0)
        {
            cmdEnum();
        }
        else if (_strnicmp(cmd, "CONNECT", 7) == 0)
        {
            char *id = trim(cmd + 7);
            cmdConnect(id);
        }
        else if (_strnicmp(cmd, "ARM ", 4) == 0)
        {
            int r = 0, g = 0, b = 0;
            if (sscanf(cmd + 4, "%d %d %d", &r, &g, &b) != 3)
                replyErr("ARM needs red green blue mA");
            else
                cmdArm(r, g, b);
        }
        else if (_strnicmp(cmd, "LED ", 4) == 0)
        {
            int r = 0, g = 0, b = 0;
            if (sscanf(cmd + 4, "%d %d %d", &r, &g, &b) != 3)
                replyErr("LED needs red green blue mA");
            else
                cmdLed(r, g, b);
        }
        else if (_strnicmp(cmd, "PATTERN ", 8) == 0)
        {
            cmdPattern(trim(cmd + 8));
        }
        else if (_stricmp(cmd, "BLANK") == 0)
        {
            cmdBlank();
        }
        else if (_stricmp(cmd, "DISCONNECT") == 0)
        {
            cmdDisconnect();
        }
        else if (_stricmp(cmd, "QUIT") == 0)
        {
            cmdDisconnect();
            break;
        }
        else
        {
            replyErr("unknown command");
        }
    }

    teardownUsb();
    return 0;
}
