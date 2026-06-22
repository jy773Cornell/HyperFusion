// Stream tab pane layout helpers.
#include "frontend/widgets/StreamPaneHelpers.hpp"

#include "frontend/widgets/WaterfallDisplayWidget.hpp"

#include <QFrame>
#include <QGroupBox>
#include <QLabel>
#include <QVBoxLayout>

namespace ui
{
QGroupBox *createStreamPane(QWidget *owner, const QString &title, QWidget *contentWidget)
{
    auto *box = new QGroupBox(title, owner);
    auto *layout = new QVBoxLayout(box);

    auto *placeholder = new QFrame(box);
    placeholder->setFrameShape(QFrame::StyledPanel);
    placeholder->setMinimumSize(320, 200);
    placeholder->setStyleSheet(QStringLiteral("background-color: #111111;"));

    auto *placeholderLayout = new QVBoxLayout(placeholder);
    placeholderLayout->setContentsMargins(6, 6, 6, 6);
    placeholderLayout->addWidget(contentWidget, 1);

    layout->addWidget(placeholder, 1);
    return box;
}

QGroupBox *createPreviewPane(QWidget *owner, const QString &title, QLabel *&labelOut)
{
    auto *box = new QGroupBox(title, owner);
    auto *layout = new QVBoxLayout(box);

    auto *placeholder = new QFrame(box);
    placeholder->setFrameShape(QFrame::StyledPanel);
    placeholder->setMinimumSize(320, 200);
    placeholder->setStyleSheet(QStringLiteral("background-color: #111111;"));

    labelOut = new QLabel(QStringLiteral("No stream"), placeholder);
    labelOut->setAlignment(Qt::AlignCenter);
    labelOut->setStyleSheet(QStringLiteral("color: #7ec8ff; background-color: transparent;"));
    labelOut->setWordWrap(true);

    auto *placeholderLayout = new QVBoxLayout(placeholder);
    placeholderLayout->setContentsMargins(6, 6, 6, 6);
    placeholderLayout->addWidget(labelOut, 1);

    layout->addWidget(placeholder, 1);
    return box;
}

QGroupBox *createWaterfallPane(QWidget *owner, const QString &title, WaterfallDisplayWidget *&widgetOut)
{
    widgetOut = new WaterfallDisplayWidget(owner);
    return createStreamPane(owner, title, widgetOut);
}

void setPreviewDisconnectedText(QLabel *label, const QString &paneTitle, const QString &cameraName)
{
    if (label == nullptr)
        return;

    label->setText(QStringLiteral("%1 %2 (disconnected)").arg(cameraName, paneTitle));
}

void setWaterfallDisconnectedText(WaterfallDisplayWidget *widget,
                                  const QString &paneTitle,
                                  const QString &cameraName)
{
    if (widget == nullptr)
        return;

    widget->clearDisplay(QStringLiteral("%1 %2 (disconnected)").arg(cameraName, paneTitle));
}
} // namespace ui
