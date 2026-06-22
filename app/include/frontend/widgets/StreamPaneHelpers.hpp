// Shared helpers for stream tab pane layout (group boxes and placeholders).
#pragma once

#include <QString>

class QGroupBox;
class QLabel;
class QWidget;

namespace ui
{
QGroupBox *createStreamPane(QWidget *owner, const QString &title, QWidget *contentWidget);
QGroupBox *createPreviewPane(QWidget *owner, const QString &title, QLabel *&labelOut);
QGroupBox *createWaterfallPane(QWidget *owner, const QString &title, class WaterfallDisplayWidget *&widgetOut);

void setPreviewDisconnectedText(QLabel *label, const QString &paneTitle, const QString &cameraName);
void setWaterfallDisconnectedText(WaterfallDisplayWidget *widget,
                                const QString &paneTitle,
                                const QString &cameraName);
} // namespace ui
