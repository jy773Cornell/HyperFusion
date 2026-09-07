// Fullscreen HDMI PSP image on the projector QScreen (frontend). GUI thread only.
#pragma once

#include <QWidget>

class QLabel;
class QScreen;

namespace ui
{
class DlpHdmiPatternWindow : public QWidget
{
    Q_OBJECT

public:
    explicit DlpHdmiPatternWindow(QWidget *parent = nullptr);

    /// Paint one 1280×720 PNG on the EVM display. Blocking-safe on the GUI thread.
    [[nodiscard]] bool showPng(const QString &path, QString *errorOut = nullptr);
    void hidePattern();

private:
    [[nodiscard]] QScreen *pickProjectorScreen() const;

    QLabel *label_ = nullptr;
};
} // namespace ui
