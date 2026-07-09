#pragma once

#include "backend/stage/StageTypes.hpp"

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
    // Initial connect homing: ZML lockstep home to sensor.
    static bool performLocalizationHoming(zaber::motion::ascii::Device &device, StageError &error);
    // Manual re-home / disconnect homing: same lockstep home to sensor.
    static bool performSimpleHoming(zaber::motion::ascii::Device &device, StageError &error);
#endif
};
