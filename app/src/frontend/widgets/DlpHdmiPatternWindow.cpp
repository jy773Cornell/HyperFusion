// Fullscreen HDMI PSP viewer (frontend). Picks the non-primary 1280×720 screen.
#include "frontend/widgets/DlpHdmiPatternWindow.hpp"

#include "backend/HyperFusionConfig.hpp"

#include <QGuiApplication>
#include <QLabel>
#include <QPixmap>
#include <QScreen>
#include <QVBoxLayout>

namespace ui
{
DlpHdmiPatternWindow::DlpHdmiPatternWindow(QWidget *parent)
    : QWidget(parent, Qt::FramelessWindowHint | Qt::Tool | Qt::WindowStaysOnTopHint)
{
    setObjectName(QStringLiteral("DlpHdmiPatternWindow"));
    setWindowTitle(QStringLiteral("DLP HDMI"));
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    label_ = new QLabel(this);
    label_->setAlignment(Qt::AlignCenter);
    label_->setScaledContents(true);
    label_->setStyleSheet(QStringLiteral("background: black;"));
    layout->addWidget(label_);
}

QScreen *DlpHdmiPatternWindow::pickProjectorScreen() const
{
    const QList<QScreen *> screens = QGuiApplication::screens();
    if (screens.isEmpty())
        return nullptr;

    const int configured = hf::hardwareConfig().dlp.hdmiScreenIndex;
    if (configured >= 0 && configured < screens.size())
        return screens.at(configured);

    for (QScreen *screen : screens)
    {
        if (screen == nullptr)
            continue;
        const QSize size = screen->geometry().size();
        if (size.width() == 1280 && size.height() == 720)
            return screen;
    }

    QScreen *primary = QGuiApplication::primaryScreen();
    if (screens.size() > 1)
    {
        for (QScreen *screen : screens)
        {
            if (screen != nullptr && screen != primary)
                return screen;
        }
    }
    return primary != nullptr ? primary : screens.front();
}

bool DlpHdmiPatternWindow::showPng(const QString &path, QString *errorOut)
{
    QPixmap pix(path);
    if (pix.isNull())
    {
        if (errorOut != nullptr)
            *errorOut = QStringLiteral("Failed to load HDMI pattern %1").arg(path);
        return false;
    }

    QScreen *screen = pickProjectorScreen();
    if (screen == nullptr)
    {
        if (errorOut != nullptr)
            *errorOut = QStringLiteral("No Qt screen for HDMI PSP.");
        return false;
    }

    setScreen(screen);
    setGeometry(screen->geometry());
    label_->setPixmap(pix);
    show();
    raise();
    return true;
}

void DlpHdmiPatternWindow::hidePattern()
{
    hide();
    if (label_ != nullptr)
        label_->clear();
}
} // namespace ui
