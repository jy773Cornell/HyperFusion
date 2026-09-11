// HDMI PSP pattern paths (backend/fpp). Resolves calibration/multiview/fpp_cal/patterns/psp. No device I/O.
#pragma once

#include <QString>

namespace hf::dlp
{
/// Directory that contains black.png / white.png / sine_*.png (1280×720).
[[nodiscard]] QString resolveHdmiPspPatternDir();
[[nodiscard]] QString hdmiPspPatternFile(const char *fileName);
} // namespace hf::dlp
