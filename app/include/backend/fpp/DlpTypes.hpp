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
    int ledRedMa = 30;
    int ledGreenMa = 30;
    int ledBlueMa = 30;
    QString testPattern = QStringLiteral("FPP scanning");
};

inline bool isFppScanningPattern(const QString &name)
{
    return QString::compare(name, QStringLiteral("FPP scanning"), Qt::CaseInsensitive) == 0;
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
};

/// GUI test sequence. Camera capture is not wired yet.
/// 8 px fringe 90/270: TPG has no sine; square-wave pan by half-stripe (period/4).
inline constexpr FppScanStep kFppScanningSteps[] = {
    {FppScanStepKind::AmbientBlank, nullptr, "Black"},
    {FppScanStepKind::Pattern, "Solid field", "White"},
    {FppScanStepKind::Pattern, "Horizontal ramp", "Coarse absolute code"},
    {FppScanStepKind::Pattern, "FPP lines 128", "128-px code"},
    {FppScanStepKind::Pattern, "FPP lines 64", "64-px code"},
    {FppScanStepKind::Pattern, "FPP lines 32", "32-px code"},
    {FppScanStepKind::Pattern, "FPP lines 16", "16-px code"},
    {FppScanStepKind::Pattern, "FPP lines 8", "8 px fringe 0 deg"},
    {FppScanStepKind::Pattern, "FPP lines 8 90", "8 px fringe 90 deg"},
    {FppScanStepKind::Pattern, "FPP lines 8 180", "8 px fringe 180 deg"},
    {FppScanStepKind::Pattern, "FPP lines 8 270", "8 px fringe 270 deg"},
};

inline constexpr int kFppScanningStepCount =
    static_cast<int>(sizeof(kFppScanningSteps) / sizeof(kFppScanningSteps[0]));
inline constexpr int kFppScanningDwellMs = 600;

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
