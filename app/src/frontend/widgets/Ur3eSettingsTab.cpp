// UR3e robot settings tab UI layout (wiring handled by Ur3ePanelController).
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/Ur3eJointBarWidget.hpp"
#include "frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp"

#include "backend/HyperFusionConfig.hpp"

#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>

namespace
{
constexpr int kUr3eJointCount = 6;
constexpr const char *kUr3eJointNames[kUr3eJointCount] = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
};

// UR3e joint limits (degrees) from ur_description/config/ur3e/joint_limits.yaml and UR3e spec.
// Most joints: ±360° (PolyScope default safety allows up to ±363°). Elbow: ±180° in ROS/MoveIt
// (hardware ±160°; URDF uses ±180° to avoid planning discontinuities).
constexpr double kUr3eJointMinDeg[kUr3eJointCount] = {
    -360.0, // shoulder_pan_joint
    -360.0, // shoulder_lift_joint
    -180.0, // elbow_joint
    -360.0, // wrist_1_joint
    -360.0, // wrist_2_joint
    -360.0, // wrist_3_joint (continuous in URDF; ±360° for UI)
};
constexpr double kUr3eJointMaxDeg[kUr3eJointCount] = {
    360.0,
    360.0,
    180.0,
    360.0,
    360.0,
    360.0,
};

void configureJointBar(ui::Ur3eJointBarWidget *bar, const int jointIndex)
{
    bar->setEnabled(false);
    const double degToRad = M_PI / 180.0;
    bar->setRangeRadians(kUr3eJointMinDeg[jointIndex] * degToRad,
                         kUr3eJointMaxDeg[jointIndex] * degToRad);
    bar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}
} // namespace

QWidget *MainWindow::createUr3eSettingsTab()
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;

    auto *scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto *page = new QWidget();
    scrollArea->setWidget(page);

    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);

    auto *connBox = new QGroupBox(QStringLiteral("UR3e connection"), page);
    auto *connForm = new QFormLayout(connBox);
    ur3eRobotIpEdit_ = new QLineEdit(connBox);
    ur3eRobotIpEdit_->setPlaceholderText(QStringLiteral("Robot controller IP / hostname"));
    ur3eRobotIpEdit_->setText(cfg.robotIp);
    ur3eConnectBtn_ = new QPushButton(QStringLiteral("Connect"), connBox);
    ur3eDisconnectBtn_ = new QPushButton(QStringLiteral("Disconnect"), connBox);
    ur3eDisconnectBtn_->setEnabled(false);
    connForm->addRow(QStringLiteral("Controller"), ur3eRobotIpEdit_);
    connForm->addRow(QStringLiteral(""), ur3eConnectBtn_);
    connForm->addRow(QStringLiteral(""), ur3eDisconnectBtn_);

    auto *jointsBox = new QGroupBox(QStringLiteral("Joints"), page);
    auto *jointsLayout = new QVBoxLayout(jointsBox);
    jointsLayout->setContentsMargins(8, 8, 8, 8);
    jointsLayout->setSpacing(6);

    auto *jointGrid = new QGridLayout();
    jointGrid->setHorizontalSpacing(8);
    jointGrid->setVerticalSpacing(2);
    jointGrid->setColumnStretch(0, 1);
    jointGrid->setColumnStretch(1, 2);

    auto *nameHeader = new QLabel(QStringLiteral("Joint Name"), jointsBox);
    auto *valueHeader = new QLabel(QStringLiteral("Target (| = current)"), jointsBox);
    QFont headerFont = nameHeader->font();
    headerFont.setBold(true);
    nameHeader->setFont(headerFont);
    valueHeader->setFont(headerFont);
    jointGrid->addWidget(nameHeader, 0, 0);
    jointGrid->addWidget(valueHeader, 0, 1);

    for (int jointIndex = 0; jointIndex < kUr3eJointCount; ++jointIndex)
    {
        auto *nameLabel = new QLabel(QString::fromUtf8(kUr3eJointNames[jointIndex]), jointsBox);
        nameLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

        ur3eJointBars_[jointIndex] = new ui::Ur3eJointBarWidget(jointsBox);
        configureJointBar(ur3eJointBars_[jointIndex], jointIndex);

        jointGrid->addWidget(nameLabel, jointIndex + 1, 0);
        jointGrid->addWidget(ur3eJointBars_[jointIndex], jointIndex + 1, 1);
    }
    jointsLayout->addLayout(jointGrid);

    ur3eMoveBtn_ = new QPushButton(QStringLiteral("Move"), jointsBox);
    ur3eStopMotionBtn_ = new QPushButton(QStringLiteral("Stop"), jointsBox);
    ur3eSyncJointsBtn_ = new QPushButton(QStringLiteral("Sync Position"), jointsBox);
    ur3eStartRvizBtn_ = new QPushButton(QStringLiteral("Start RViz"), jointsBox);
    ur3eStartMoveItBtn_ = new QPushButton(QStringLiteral("Start MoveIt"), jointsBox);
    ur3eMoveBtn_->setEnabled(false);
    ur3eStopMotionBtn_->setEnabled(false);
    ur3eSyncJointsBtn_->setEnabled(false);
    ur3eStartRvizBtn_->setEnabled(false);
    ur3eStartMoveItBtn_->setEnabled(false);
    ur3eStartRvizBtn_->setToolTip(
        QStringLiteral("Open RViz to visualize the live robot only (no motion planning)."));
    ur3eMoveBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ur3eStopMotionBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ur3eSyncJointsBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ur3eStartRvizBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ur3eStartMoveItBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    auto *motionRow = new QHBoxLayout();
    motionRow->setSpacing(8);
    motionRow->addWidget(ur3eMoveBtn_, 1);
    motionRow->addWidget(ur3eStopMotionBtn_, 1);
    jointsLayout->addLayout(motionRow);

    auto *auxRow = new QHBoxLayout();
    auxRow->setSpacing(8);
    auxRow->addWidget(ur3eSyncJointsBtn_, 1);
    auxRow->addWidget(ur3eStartRvizBtn_, 1);
    auxRow->addWidget(ur3eStartMoveItBtn_, 1);
    jointsLayout->addLayout(auxRow);

    ur3eHemisphereScanSettings_ = new ui::Ur3eHemisphereScanSettingsWidget(page);

    ur3ePosePollTimer_ = new QTimer(this);
    ur3ePosePollTimer_->setInterval(500);

    layout->addWidget(connBox);
    layout->addWidget(jointsBox);
    layout->addWidget(ur3eHemisphereScanSettings_);
    layout->addStretch();

    ur3eSettingsPage_ = page;
    return scrollArea;
}
