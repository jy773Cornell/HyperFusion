// Shared UI helpers for MainWindow settings tabs (buttons, icons, profile filters).
#pragma once

#include <QString>

class QIcon;
class QPushButton;
class QToolButton;
class QWidget;

namespace ui
{
QIcon makeHomeIcon(int size = 22);
QToolButton *makeStageToolButton(QWidget *parent, int standardPixmapIcon, const QString &tooltip);

QIcon makeRecorderStopIcon(int size = 14);
QIcon makeRecorderPreviewIcon(int size = 14);
QIcon makeRecorderRecordIcon(int size = 14);
QPushButton *makeCaptureCompactWhiteButton(QWidget *parent, const QString &label);
QPushButton *makeRecorderButton(QWidget *parent, const QIcon &icon, const QString &label);

bool lumoProfileMatchesFx10eSlot(const QString &name);
bool lumoProfileMatchesSwir3Slot(const QString &name);
} // namespace ui
