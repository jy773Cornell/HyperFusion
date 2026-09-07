// DLP3010EVM-LC projector types (backend/fpp).
// Isolated from HSI lighthouse. Connect arms (LEDs + black curtain). No light until Show.
#pragma once

#include <string>
#include <vector>

#include <QString>

namespace hf::dlp
{
enum class DlpProjectorState
{
    Disconnected,
    Connected,
    Armed,
    Projecting,
    Fault
};

enum class DlpErrorCode
{
    None,
    NotAvailable,
    InvalidState,
    SdkError,
    InternalError
};

struct DlpError
{
    DlpErrorCode code = DlpErrorCode::None;
    std::string message;
    bool fatal = false;
};

struct DlpDeviceInfo
{
    QString id;
    QString model;
    QString displayName() const
    {
        if (model.isEmpty())
            return id;
        if (id.isEmpty())
            return model;
        return model + QStringLiteral(" ") + id;
    }
};

struct DlpProjectorSettings
{
    QString deviceId;
    int ledRedMa = 2400;
    int ledGreenMa = 2400;
    int ledBlueMa = 2400;
    QString testPattern = QStringLiteral("FPP HDMI");
};

inline bool isHdmiPspScanningPattern(const QString &name)
{
    return QString::compare(name, QStringLiteral("FPP HDMI"), Qt::CaseInsensitive) == 0
           || QString::compare(name, QStringLiteral("FPP HDMI PSP"), Qt::CaseInsensitive) == 0;
}

inline bool isFppScanningPattern(const QString &name)
{
    return QString::compare(name, QStringLiteral("FPP"), Qt::CaseInsensitive) == 0
           || QString::compare(name, QStringLiteral("FPP scanning"), Qt::CaseInsensitive) == 0
           || QString::compare(name, QStringLiteral("FPP NO REBUILD"), Qt::CaseInsensitive) == 0
           || isHdmiPspScanningPattern(name);
}

enum class FppScanStepKind
{
    AmbientBlank,
    Pattern
};

struct FppScanStep
{
    FppScanStepKind kind = FppScanStepKind::Pattern;
    const char *patternName = nullptr;
    const char *label = nullptr;
    /// 1280×720 PNG under calibration/multiview/patterns/psp (HDMI only).
    const char *hdmiFile = nullptr;
};

/// HDMI 26-frame sine PSP (u then v). JSON names must stay PSP* so decode uses psp.py.
inline constexpr FppScanStep kFppScanningSteps[] = {
    {FppScanStepKind::AmbientBlank, nullptr, "Black", "black.png"},
    {FppScanStepKind::Pattern, "PSP white", "White", "white.png"},
    {FppScanStepKind::Pattern, "PSP sine 1 0", "1-period sine 0", "sine_1_0.png"},
    {FppScanStepKind::Pattern, "PSP sine 1 90", "1-period sine 90", "sine_1_90.png"},
    {FppScanStepKind::Pattern, "PSP sine 1 180", "1-period sine 180", "sine_1_180.png"},
    {FppScanStepKind::Pattern, "PSP sine 1 270", "1-period sine 270", "sine_1_270.png"},
    {FppScanStepKind::Pattern, "PSP sine 8 0", "8-period sine 0", "sine_8_0.png"},
    {FppScanStepKind::Pattern, "PSP sine 8 90", "8-period sine 90", "sine_8_90.png"},
    {FppScanStepKind::Pattern, "PSP sine 8 180", "8-period sine 180", "sine_8_180.png"},
    {FppScanStepKind::Pattern, "PSP sine 8 270", "8-period sine 270", "sine_8_270.png"},
    {FppScanStepKind::Pattern, "PSP sine 80 0", "80-period sine 0", "sine_80_0.png"},
    {FppScanStepKind::Pattern, "PSP sine 80 90", "80-period sine 90", "sine_80_90.png"},
    {FppScanStepKind::Pattern, "PSP sine 80 180", "80-period sine 180", "sine_80_180.png"},
    {FppScanStepKind::Pattern, "PSP sine 80 270", "80-period sine 270", "sine_80_270.png"},
    {FppScanStepKind::Pattern, "PSP sine v 1 0", "1-period sine v 0", "sine_v_1_0.png"},
    {FppScanStepKind::Pattern, "PSP sine v 1 90", "1-period sine v 90", "sine_v_1_90.png"},
    {FppScanStepKind::Pattern, "PSP sine v 1 180", "1-period sine v 180", "sine_v_1_180.png"},
    {FppScanStepKind::Pattern, "PSP sine v 1 270", "1-period sine v 270", "sine_v_1_270.png"},
    {FppScanStepKind::Pattern, "PSP sine v 8 0", "8-period sine v 0", "sine_v_8_0.png"},
    {FppScanStepKind::Pattern, "PSP sine v 8 90", "8-period sine v 90", "sine_v_8_90.png"},
    {FppScanStepKind::Pattern, "PSP sine v 8 180", "8-period sine v 180", "sine_v_8_180.png"},
    {FppScanStepKind::Pattern, "PSP sine v 8 270", "8-period sine v 270", "sine_v_8_270.png"},
    {FppScanStepKind::Pattern, "PSP sine v 80 0", "80-period sine v 0", "sine_v_80_0.png"},
    {FppScanStepKind::Pattern, "PSP sine v 80 90", "80-period sine v 90", "sine_v_80_90.png"},
    {FppScanStepKind::Pattern, "PSP sine v 80 180", "80-period sine v 180", "sine_v_80_180.png"},
    {FppScanStepKind::Pattern, "PSP sine v 80 270", "80-period sine v 270", "sine_v_80_270.png"},
};

inline constexpr int kFppScanningStepCount =
    static_cast<int>(sizeof(kFppScanningSteps) / sizeof(kFppScanningSteps[0]));
inline constexpr int kFppScanningDwellMs = 600;
/// Pin capture: skip the in-flight exposure, then take the next new BFS frame.
inline constexpr int kFppCaptureMinNewFrames = 2;
/// Extra BFS frames after the first new frame so the projected pattern can settle.
inline constexpr int kFppCaptureStabilizeFrames = 5;
inline constexpr int kFppCaptureFrameWaitMs = 2000;
inline constexpr int kFppCaptureStabilizeWaitMs = 4000;

inline int clampLedMilliamp(int ma, int maxMa)
{
    if (maxMa < 0)
        maxMa = 0;
    if (ma < 0)
        return 0;
    if (ma > maxMa)
        return maxMa;
    return ma;
}
} // namespace hf::dlp
