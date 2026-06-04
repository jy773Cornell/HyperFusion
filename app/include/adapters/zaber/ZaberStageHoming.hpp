#pragma once

#include "backend/StageTypes.hpp"

#ifdef HF_HAVE_ZML
namespace zaber::motion::ascii
{
class Device;
} // namespace zaber::motion::ascii
#endif

class ZaberStageHoming
{
public:
#ifdef HF_HAVE_ZML
    // Initial connect homing (+20 mm pre-move when referenced, then home sensor).
    static bool performLocalizationHoming(zaber::motion::ascii::Device &device, StageError &error);
    // Manual re-home: ZML lockstep home only (no localization pre-move).
    static bool performSimpleHoming(zaber::motion::ascii::Device &device, StageError &error);
#endif
};
