#pragma once

#include "backend/stage/IStageController.hpp"

#include "adapters/zaber/ZaberStageProfile.hpp"

#include <optional>

#ifdef HF_HAVE_ZML
#include <zaber/motion/ascii/connection.h>
#endif

class ZaberStageController : public IStageController
{
public:
    ZaberStageController();
    ~ZaberStageController() override;

    std::string name() const override;
    StageState state() const override;
    StageTopology topology() const override;

    bool connect(const StageConnectSettings &settings, StageError &error) override;
    bool homeWithLocalization(StageError &error) override;
    bool home(StageError &error) override;
    void stopMotion(bool waitUntilIdle = true) override;
    void disconnect() override;

    bool moveRelativeMm(double distanceMm, double speedMmPerSec, StageError &error) override;
    bool moveAbsoluteMm(double positionMm,
                        double speedMmPerSec,
                        bool waitUntilIdle,
                        StageError &error) override;
    bool moveVelocityMm(double velocityMmPerSec, StageError &error) override;
    bool getPrimaryPositionMm(double &positionMm, StageError &error) override;

private:
    bool ensureReadyForMotion(StageError &error) const;
    bool ensureCanReadPosition(StageError &error) const;
#ifdef HF_HAVE_ZML
    std::optional<zaber::motion::ascii::Connection> connection_;
#endif
    StageState state_ = StageState::Disconnected;
    StageTopology topology_;
    double motionAccelerationMmPerSec2_ = zaber_stage::kDefaultMotionAccelerationMmPerSec2;
};
