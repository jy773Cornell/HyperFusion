#pragma once

#include "backend/stage/StageTypes.hpp"

#ifdef HF_HAVE_ZML
namespace zaber::motion::ascii
{
class Connection;
} // namespace zaber::motion::ascii
#endif

class ZaberStageConfigurator
{
public:
#ifdef HF_HAVE_ZML
    static bool prepare(zaber::motion::ascii::Connection &connection,
                        StageTopology &topology,
                        StageError &error,
                        double motionAccelerationMmPerSec2);
    static bool enableLockstep(zaber::motion::ascii::Connection &connection, StageTopology &topology, StageError &error);
#endif
};
