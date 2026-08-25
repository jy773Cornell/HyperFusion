// Linear-stage homing (adapter): lockstep home to the home sensor.
// Connect, manual, capture, and disconnect homes all share this path.
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
    // Initial connect homing: cruise near home, then lockstep home to sensor.
    static bool performLocalizationHoming(zaber::motion::ascii::Device &device, StageError &error);
    // Manual re-home / disconnect / capture homing: same gentle lockstep home.
    static bool performSimpleHoming(zaber::motion::ascii::Device &device, StageError &error);
#endif
};
