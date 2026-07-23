// Shared UI helpers for MainWindow settings tabs (buttons, icons, profile filters).
#include "frontend/widgets/MainWindowTabHelpers.hpp"

#include <QColor>
#include <QIcon>
#include <QPainter>
#include <QPalette>
#include <QPixmap>
#include <QPolygon>
#include <QPushButton>
#include <QScrollArea>
#include <QSize>
#include <QStyle>
#include <QToolButton>
#include <QWidget>

namespace ui
{
namespace
{
QPushButton *makeCaptureWhiteButton(QWidget *parent, const QString &label, const QIcon &icon = {})
{
    auto *button = new QPushButton(label, parent);
    if (!icon.isNull())
    {
        button->setIcon(icon);
        button->setIconSize(QSize(14, 14));
    }
    button->setMinimumHeight(36);
    button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    button->setStyleSheet(QStringLiteral(
        "QPushButton {"
        "  background-color: #ffffff;"
        "  color: #222222;"
        "  border: 1px solid #b0b0b0;"
        "  border-radius: 2px;"
        "  padding: 4px 12px;"
        "  min-width: 48px;"
        "}"
        "QPushButton:hover { background-color: #f0f0f0; }"
        "QPushButton:pressed { background-color: #e0e0e0; }"
        "QPushButton:disabled { color: #999999; background-color: #f5f5f5; }"));
    return button;
}
} // namespace

QIcon makeHomeIcon(const int size)
{
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QColor fill(70, 70, 70);
    const QPen outline(QColor(40, 40, 40), 1.2);
    painter.setPen(outline);
    painter.setBrush(fill);

    const int pad = 2;
    const int roofPeakX = size / 2;
    const int roofPeakY = pad;
    const int roofLeftX = pad + 1;
    const int roofRightX = size - pad - 1;
    const int roofBaseY = size / 2 - 1;
    const QPolygon roof{{roofPeakX, roofPeakY}, {roofLeftX, roofBaseY}, {roofRightX, roofBaseY}};
    painter.drawPolygon(roof);

    const QRect body(pad + 3, roofBaseY, size - 2 * (pad + 3), size - roofBaseY - pad);
    painter.drawRect(body);

    painter.setBrush(QColor(230, 230, 230));
    painter.setPen(Qt::NoPen);
    painter.drawRect(body.center().x() - 2, body.bottom() - 5, 4, 5);

    return QIcon(pixmap);
}

QToolButton *makeStageToolButton(QWidget *parent, const int standardPixmapIcon, const QString &tooltip)
{
    auto *button = new QToolButton(parent);
    button->setIcon(parent->style()->standardIcon(static_cast<QStyle::StandardPixmap>(standardPixmapIcon)));
    button->setToolTip(tooltip);
    button->setAutoRaise(true);
    button->setIconSize(QSize(28, 28));
    button->setMinimumSize(44, 44);
    return button;
}

QIcon makeRecorderStopIcon(const int size)
{
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(30, 30, 30));
    painter.drawRect(2, 2, size - 4, size - 4);
    return QIcon(pixmap);
}

QIcon makeRecorderPreviewIcon(const int size)
{
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(70, 170, 85));
    const QPolygon triangle{{3, 2}, {3, size - 2}, {size - 2, size / 2}};
    painter.drawPolygon(triangle);
    return QIcon(pixmap);
}

QIcon makeRecorderRecordIcon(const int size)
{
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(220, 45, 45));
    painter.drawEllipse(2, 2, size - 4, size - 4);
    return QIcon(pixmap);
}

QPushButton *makeCaptureCompactWhiteButton(QWidget *parent, const QString &label)
{
    auto *button = makeCaptureWhiteButton(parent, label);
    button->setMinimumHeight(22);
    button->setMaximumHeight(22);
    button->setFixedWidth(40);
    button->setStyleSheet(QStringLiteral(
        "QPushButton {"
        "  background-color: #ffffff;"
        "  color: #222222;"
        "  border: 1px solid #b0b0b0;"
        "  border-radius: 2px;"
        "  padding: 1px 6px;"
        "  min-height: 22px;"
        "  max-height: 22px;"
        "  min-width: 40px;"
        "  font-size: 11px;"
        "}"
        "QPushButton:hover { background-color: #f0f0f0; }"
        "QPushButton:pressed { background-color: #e0e0e0; }"
        "QPushButton:disabled { color: #999999; background-color: #f5f5f5; }"));
    return button;
}

QPushButton *makeRecorderButton(QWidget *parent, const QIcon &icon, const QString &label)
{
    auto *button = makeCaptureWhiteButton(parent, label, icon);
    button->setIconSize(QSize(12, 12));
    button->setMinimumHeight(26);
    button->setMaximumHeight(26);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    button->setStyleSheet(QStringLiteral(
        "QPushButton {"
        "  background-color: #ffffff;"
        "  color: #222222;"
        "  border: 1px solid #b0b0b0;"
        "  border-radius: 2px;"
        "  padding: 2px 8px;"
        "  min-height: 26px;"
        "  max-height: 26px;"
        "}"
        "QPushButton:hover { background-color: #f0f0f0; }"
        "QPushButton:pressed { background-color: #e0e0e0; }"
        "QPushButton:disabled { color: #999999; background-color: #f5f5f5; }"));
    return button;
}

void applyWhiteSettingsBackground(QWidget *widget)
{
    if (widget == nullptr)
        return;
    widget->setAutoFillBackground(true);
    QPalette palette = widget->palette();
    palette.setColor(QPalette::Window, Qt::white);
    palette.setColor(QPalette::Base, Qt::white);
    widget->setPalette(palette);
}

void applyWhiteSettingsScrollBackground(QScrollArea *scroll)
{
    if (scroll == nullptr)
        return;
    applyWhiteSettingsBackground(scroll);
    applyWhiteSettingsBackground(scroll->viewport());
    applyWhiteSettingsBackground(scroll->widget());
}

bool lumoProfileMatchesFx10eSlot(const QString &name)
{
    if (name.contains(QStringLiteral("SWIR"), Qt::CaseInsensitive))
        return false;
    return name.contains(QStringLiteral("FX10"), Qt::CaseInsensitive)
           || name.contains(QStringLiteral("Pleora"), Qt::CaseInsensitive);
}

bool lumoProfileMatchesSwir3Slot(const QString &name)
{
    return name.contains(QStringLiteral("SWIR"), Qt::CaseInsensitive)
           || name.contains(QStringLiteral("NI"), Qt::CaseInsensitive);
}
} // namespace ui
