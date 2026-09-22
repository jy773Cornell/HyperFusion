// DLP Multiview tab: worker, connect (auto-arm), Blank, test pattern (frontend).
#pragma once

#include "backend/fpp/DlpTypes.hpp"

#include <QObject>
#include <QTimer>

#include <memory>

class MainWindow;

namespace ui
{
class DlpHdmiPatternWindow;
}

namespace hf::dlp
{
class DlpProjectorWorker;

class DlpPanelController : public QObject
{
    Q_OBJECT

public:
    explicit DlpPanelController(MainWindow *host, QObject *parent = nullptr);
    ~DlpPanelController() override;

    void initializeWorker();
    void shutdownSync();
    void wireSettingsTabConnections();
    [[nodiscard]] bool isConnected() const;
    /// Snapshot of Multiview DLP UI settings (LED mA, device id).
    [[nodiscard]] DlpProjectorSettings currentSettings() const;
    /// One FPP sequence step on the EVM (blocks). For pin capture, not the GUI loop.
    [[nodiscard]] bool showFppScanStepSync(int stepIndex, QString *errorOut = nullptr);
    /// Apply LED currents synchronously for a capture step.
    [[nodiscard]] bool applyLedCurrentsSync(const DlpProjectorSettings &settings,
                                            QString *errorOut = nullptr);
    [[nodiscard]] bool blankSync(QString *errorOut = nullptr);

private:
    void onRefreshClicked();
    void onConnectClicked();
    void onDisconnectClicked();
    void onBlankClicked();
    void onTestPatternClicked();
    void onSettingsEdited();
    void applyLedCurrentsFromUi();
    void onStateChanged(DlpProjectorState state);
    void onError(const DlpError &error);
    void onDevices(const std::vector<DlpDeviceInfo> &devices);
    void onWorkerLog(const QString &message);
    [[nodiscard]] DlpProjectorSettings settingsFromUi() const;
    [[nodiscard]] bool showHdmiPngOnGui(int stepIndex, DlpError &error);

    MainWindow *host_ = nullptr;
    std::unique_ptr<DlpProjectorWorker> worker_;
    std::unique_ptr<ui::DlpHdmiPatternWindow> hdmiWindow_;
    QTimer *ledApplyTimer_ = nullptr;
};
} // namespace hf::dlp
