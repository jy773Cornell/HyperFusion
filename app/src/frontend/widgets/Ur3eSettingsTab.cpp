// 3D Scanning settings: nested UR3e robot panel + BFS camera settings (UI
// only).
#include "frontend/widgets/BfsCameraSettingsWidget.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/MainWindowTabHelpers.hpp"
#include "frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp"
#include "frontend/widgets/Ur3eJointBarWidget.hpp"

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
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>

namespace {
constexpr int kUr3eJointCount = 6;
/// Short UI labels (full ROS names in tooltips) so the narrow settings column fits.
constexpr const char *kUr3eJointLabels[kUr3eJointCount] = {
    "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3",
};
constexpr const char *kUr3eJointTooltips[kUr3eJointCount] = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
};

// UR3e joint limits (degrees) from ur_description/config/ur3e/joint_limits.yaml
// and UR3e spec. Most joints: ±360° (PolyScope default safety allows up to
// ±363°). Elbow: ±180° in ROS/MoveIt (hardware ±160°; URDF uses ±180° to avoid
// planning discontinuities).
constexpr double kUr3eJointMinDeg[kUr3eJointCount] = {
    -360.0, // shoulder_pan_joint
    -360.0, // shoulder_lift_joint
    -180.0, // elbow_joint
    -360.0, // wrist_1_joint
    -360.0, // wrist_2_joint
    -360.0, // wrist_3_joint (continuous in URDF; ±360° for UI)
};
constexpr double kUr3eJointMaxDeg[kUr3eJointCount] = {
    360.0, 360.0, 180.0, 360.0, 360.0, 360.0,
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
    auto *outerPage = new QWidget(this);
    ui::applyWhiteSettingsBackground(outerPage);
    auto *outerLayout = new QVBoxLayout(outerPage);
    outerLayout->setContentsMargins(0, 0, 0, 0);

    scanningSettingsTabs_ = new QTabWidget(outerPage);

    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;

    auto *scrollArea = new QScrollArea(scanningSettingsTabs_);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    auto *page = new QWidget();
    page->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    scrollArea->setWidget(page);
    ui::applyWhiteSettingsScrollBackground(scrollArea);

    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(6);

    auto *connBox = new QGroupBox(QStringLiteral("Connection"), page);
    auto *connLayout = new QVBoxLayout(connBox);
    connLayout->setContentsMargins(8, 8, 8, 8);
    connLayout->setSpacing(6);

    auto *ipRow = new QHBoxLayout();
    ipRow->setSpacing(6);
    auto *ipLabel = new QLabel(QStringLiteral("IP"), connBox);
    ur3eRobotIpEdit_ = new QLineEdit(connBox);
    ur3eRobotIpEdit_->setPlaceholderText(QStringLiteral("192.168.1.10"));
    ur3eRobotIpEdit_->setText(cfg.robotIp);
    ur3eRobotIpEdit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ipRow->addWidget(ipLabel, 0);
    ipRow->addWidget(ur3eRobotIpEdit_, 1);
    connLayout->addLayout(ipRow);

    ur3eConnectBtn_ = new QPushButton(QStringLiteral("Connect"), connBox);
    ur3eDisconnectBtn_ = new QPushButton(QStringLiteral("Disconnect"), connBox);
    ur3eDisconnectBtn_->setEnabled(false);
    ur3eConnectBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ur3eDisconnectBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto *connBtnRow = new QHBoxLayout();
    connBtnRow->setSpacing(6);
    connBtnRow->addWidget(ur3eConnectBtn_, 1);
    connBtnRow->addWidget(ur3eDisconnectBtn_, 1);
    connLayout->addLayout(connBtnRow);

    auto *jointsBox = new QGroupBox(QStringLiteral("Joints"), page);
    auto *jointsLayout = new QVBoxLayout(jointsBox);
    jointsLayout->setContentsMargins(8, 8, 8, 8);
    jointsLayout->setSpacing(4);

    auto *jointGrid = new QGridLayout();
    jointGrid->setHorizontalSpacing(6);
    jointGrid->setVerticalSpacing(2);
    jointGrid->setColumnStretch(0, 0);
    jointGrid->setColumnStretch(1, 1);

    auto *nameHeader = new QLabel(QStringLiteral("Joint"), jointsBox);
    auto *valueHeader = new QLabel(QStringLiteral("Target"), jointsBox);
    QFont headerFont = nameHeader->font();
    headerFont.setBold(true);
    nameHeader->setFont(headerFont);
    valueHeader->setFont(headerFont);
    jointGrid->addWidget(nameHeader, 0, 0);
    jointGrid->addWidget(valueHeader, 0, 1);

    for (int jointIndex = 0; jointIndex < kUr3eJointCount; ++jointIndex)
    {
        auto *nameLabel =
            new QLabel(QString::fromUtf8(kUr3eJointLabels[jointIndex]), jointsBox);
        nameLabel->setToolTip(QString::fromUtf8(kUr3eJointTooltips[jointIndex]));
        nameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        nameLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

        ur3eJointBars_[jointIndex] = new ui::Ur3eJointBarWidget(jointsBox);
        configureJointBar(ur3eJointBars_[jointIndex], jointIndex);

        jointGrid->addWidget(nameLabel, jointIndex + 1, 0);
        jointGrid->addWidget(ur3eJointBars_[jointIndex], jointIndex + 1, 1);
    }
    jointsLayout->addLayout(jointGrid);

    ur3eMoveBtn_ = new QPushButton(QStringLiteral("Move"), jointsBox);
    ur3eStopMotionBtn_ = new QPushButton(QStringLiteral("Stop"), jointsBox);
    ur3eSyncJointsBtn_ = new QPushButton(QStringLiteral("Sync"), jointsBox);
    ur3eStartRvizBtn_ = new QPushButton(QStringLiteral("RViz"), jointsBox);
    ur3eStartMoveItBtn_ = new QPushButton(QStringLiteral("MoveIt"), jointsBox);
    ur3eMoveBtn_->setEnabled(false);
    ur3eStopMotionBtn_->setEnabled(false);
    ur3eSyncJointsBtn_->setEnabled(false);
    ur3eStartRvizBtn_->setEnabled(false);
    ur3eStartMoveItBtn_->setEnabled(false);
    ur3eSyncJointsBtn_->setToolTip(QStringLiteral("Sync Position — copy live joints into the targets."));
    ur3eStartRvizBtn_->setToolTip(QStringLiteral(
        "Open RViz to visualize the live robot only (no motion planning)."));
    ur3eStartMoveItBtn_->setToolTip(QStringLiteral("Start MoveIt motion planning."));
    ur3eMoveBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ur3eStopMotionBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ur3eSyncJointsBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ur3eStartRvizBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ur3eStartMoveItBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    auto *motionRow = new QHBoxLayout();
    motionRow->setSpacing(6);
    motionRow->addWidget(ur3eMoveBtn_, 1);
    motionRow->addWidget(ur3eStopMotionBtn_, 1);
    jointsLayout->addLayout(motionRow);

    auto *auxRow = new QHBoxLayout();
    auxRow->setSpacing(6);
    auxRow->addWidget(ur3eSyncJointsBtn_, 1);
    auxRow->addWidget(ur3eStartRvizBtn_, 1);
    auxRow->addWidget(ur3eStartMoveItBtn_, 1);
    jointsLayout->addLayout(auxRow);

    ur3eHemisphereScanSettings_ = new ui::Ur3eHemisphereScanSettingsWidget(page);
    ur3eHemisphereScanSettings_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    ur3ePosePollTimer_ = new QTimer(this);
    ur3ePosePollTimer_->setInterval(500);

    layout->addWidget(connBox);
    layout->addWidget(jointsBox);
    layout->addWidget(ur3eHemisphereScanSettings_);
    layout->addStretch();

    ur3eSettingsPage_ = page;

    scanningSettingsTabs_->addTab(scrollArea, QStringLiteral("UR3e"));
    bfsCameraSettings_ = new ui::BfsCameraSettingsWidget(scanningSettingsTabs_);
    scanningSettingsTabs_->addTab(bfsCameraSettings_, QStringLiteral("BFS"));

    outerLayout->addWidget(scanningSettingsTabs_, 1);
    return outerPage;
}
