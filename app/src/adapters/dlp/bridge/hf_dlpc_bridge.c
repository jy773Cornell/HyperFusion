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

#ifndef DLP3010_WIDTH
#define DLP3010_WIDTH 1280
#define DLP3010_HEIGHT 720
#endif

static int writeGeometry(uint16_t startPixel);
static void cmdSplash(int index);

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

static void cmdVideo(void)
{
    /* HDMI / external video port — unused by the hybrid FPP burst. */
    if (!writeGeometry(0))
        return;
    if (!check(DLPC34XX_WriteOperatingModeSelect(DLPC34XX_OM_EXTERNAL_VIDEO_PORT),
               "WriteOperatingModeSelect"))
        return;
    if (!check(DLPC34XX_WriteDisplayImageCurtain(DLPC34XX_ICE_DISABLE, DLPC34XX_C_BLACK),
               "WriteDisplayImageCurtain"))
        return;
    if (!check(DLPC34XX_WriteRgbLedEnable(1, 1, 1), "WriteRgbLedEnable"))
        return;
    replyOk();
}

static void cmdSplash(int index)
{
    /* Display-splash (DLPU075 Table 3-32). Not Splash Pattern — that needs
     * Trigger/Pattern Config or the DMD stays black. */
    DLPC34XX_SplashScreenHeader_s header;
    uint16_t inW = DLP3010_WIDTH;
    uint16_t inH = DLP3010_HEIGHT;

    if (index < 0 || index > 3)
    {
        replyErr("SPLASH index must be 0-3");
        return;
    }

    memset(&header, 0, sizeof(header));
    if (DLPC34XX_ReadSplashScreenHeader((uint8_t)index, &header) == SUCCESS
        && header.WidthInPixels > 0 && header.HeightInPixels > 0)
    {
        inW = header.WidthInPixels;
        inH = header.HeightInPixels;
    }

    (void)DLPC34XX_WriteImageFreeze(1);
    if (!check(DLPC34XX_WriteSplashScreenSelect((uint8_t)index), "WriteSplashScreenSelect"))
        return;
    if (!check(DLPC34XX_WriteInputImageSize(inW, inH), "WriteInputImageSize"))
        return;
    if (!check(DLPC34XX_WriteImageCrop(0, 0, inW, inH), "WriteImageCrop"))
        return;
    /* Scale splash to full DMD. Factory slots 2–3 are 854×480; we flash 640×360. */
    if (!check(DLPC34XX_WriteDisplaySize(0, 0, DLP3010_WIDTH, DLP3010_HEIGHT),
               "WriteDisplaySize"))
        return;
    if (!check(DLPC34XX_WriteOperatingModeSelect(DLPC34XX_OM_SPLASH_SCREEN),
               "WriteOperatingModeSelect"))
        return;
    if (!check(DLPC34XX_WriteSplashScreenExecute(), "WriteSplashScreenExecute"))
        return;
    if (!check(DLPC34XX_WriteDisplayImageCurtain(DLPC34XX_ICE_DISABLE, DLPC34XX_C_BLACK),
               "WriteDisplayImageCurtain"))
        return;
    if (!check(DLPC34XX_WriteRgbLedEnable(1, 1, 1), "WriteRgbLedEnable"))
        return;
    Sleep(800);
    fprintf(stdout, "OK splash=%d %ux%u\n", index, (unsigned)inW, (unsigned)inH);
    fflush(stdout);
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
    unsigned fg;
    unsigned bg;
    unsigned quarter;
    int inverted = 0;

    memset(&lines, 0, sizeof(lines));
    if (width < 1)
        width = 1;
    if (width > 255)
        width = 255;

    /* TPG VerticalLines always starts at x=0. Image crop does not pan the bars,
     * so 0° and 90° were identical. Encode phase with duty + color swap:
     *   0°   W width / B width
     *   90°  W width/2 / B (width + width/2)   same period, edges moved
     *   180° B width / W width
     *   270° B width/2 / W (width + width/2) */
    if (phaseDeg == 180 || phaseDeg == 270)
        inverted = 1;
    quarter = width / 2;
    if ((phaseDeg == 90 || phaseDeg == 270) && quarter >= 1)
    {
        fg = quarter;
        bg = width + quarter;
    }
    else
    {
        fg = width;
        bg = width;
    }
    if (fg < 1)
        fg = 1;
    if (bg < 1)
        bg = 1;
    if (fg > 255)
        fg = 255;
    if (bg > 255)
        bg = 255;

    if (!writeGeometry(0))
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
    lines.ForegroundLineWidth = (uint8_t)fg;
    lines.BackgroundLineWidth = (uint8_t)bg;
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
    if (_strnicmp(name, "Splash ", 7) == 0)
    {
        int index = 0;
        if (sscanf(name + 7, "%d", &index) != 1)
        {
            replyErr("Splash needs index 0-3");
            return;
        }
        cmdSplash(index);
        return;
    }
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
        else if (_stricmp(cmd, "VIDEO") == 0)
        {
            cmdVideo();
        }
        else if (_strnicmp(cmd, "SPLASH ", 7) == 0)
        {
            int index = 0;
            if (sscanf(cmd + 7, "%d", &index) != 1)
                replyErr("SPLASH needs index 0-3");
            else
                cmdSplash(index);
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
