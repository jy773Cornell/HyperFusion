#pragma once

#include "core/IStageController.hpp"

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
    void disconnect() override;

private:
#ifdef HF_HAVE_ZML
    std::optional<zaber::motion::ascii::Connection> connection_;
#endif
    StageState state_ = StageState::Disconnected;
    StageTopology topology_;
};
