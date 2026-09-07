// Recording-complete summary: one page per mode/camera (frontend/ui).
// Each page: selectable thumbnails on the left, large viewer on the right.
// Dialog is resizable; the large viewer does not open a click-to-zoom preview.
#pragma once

#include "backend/camera/CaptureWriterTypes.hpp"

#include <QDialog>
#include <QString>

class CaptureSessionSummaryDialog final : public QDialog
{
    Q_OBJECT

public:
    explicit CaptureSessionSummaryDialog(QWidget *parent = nullptr);

    static void execForSession(QWidget *parent,
                               const CaptureWriterSessionSummary &summary,
                               const QString &extraDetails);
};
