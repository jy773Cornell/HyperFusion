// Recording-complete summary: one page per illumination mode / camera.
// Resizable dialog; click a thumbnail to open a full image preview.
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
