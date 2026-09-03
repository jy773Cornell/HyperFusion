// Recording-complete summary dialog (frontend/ui): tab per mode/camera.
// Resizable; click a thumbnail to open a full-size image preview.
#include "frontend/widgets/CaptureSessionSummaryDialog.hpp"

#include <QCursor>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPixmap>
#include <QScrollArea>
#include <QSizePolicy>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <utility>
#include <vector>

CaptureSessionSummaryDialog::CaptureSessionSummaryDialog(QWidget *parent)
    : QDialog(parent)
{
}

namespace
{
constexpr int kCellImageWidth = 320;
constexpr int kCellImageHeight = 140;
constexpr int kSpectraImageWidth = 680;
constexpr int kSpectraImageHeight = 300;
constexpr int kDialogWidth = 752;
constexpr int kDialogHeight = 860;

struct StreamImageSection
{
    QString title;
    QStringList paths;
};

int streamSortRank(const QString &relativeRoot)
{
    const QString key = relativeRoot.trimmed().toLower().replace(QLatin1Char('\\'), QLatin1Char('/'));
    if (key == QLatin1String("reflectance/fx10e"))
        return 0;
    if (key == QLatin1String("reflectance/swir3"))
        return 1;
    if (key == QLatin1String("transmittance/fx10e"))
        return 2;
    if (key == QLatin1String("transmittance/swir3"))
        return 3;
    return 100;
}

QDir resolvePreprocessedDir(const QString &sessionDirectory, const QString &relativeRoot)
{
    const QDir session(sessionDirectory);
    if (!relativeRoot.trimmed().isEmpty())
    {
        const QDir nested(session.filePath(relativeRoot + QStringLiteral("/preprocessed")));
        if (nested.exists())
            return nested;
    }
    return QDir(session.filePath(QStringLiteral("preprocessed")));
}

QString firstExisting(const QDir &dir, const QStringList &nameFilters)
{
    if (!dir.exists())
        return {};
    const QStringList names = dir.entryList(nameFilters, QDir::Files, QDir::Name);
    if (names.isEmpty())
        return {};
    return dir.filePath(names.front());
}

QStringList collectMatching(const QDir &dir, const QStringList &nameFilters)
{
    QStringList paths;
    if (!dir.exists())
        return paths;
    const QStringList names = dir.entryList(nameFilters, QDir::Files, QDir::Name);
    for (const QString &name : names)
        paths.push_back(dir.filePath(name));
    return paths;
}

std::vector<StreamImageSection> collectStreamImageSections(const QString &sessionDirectory,
                                                           const QString &relativeRoot)
{
    const QDir preprocessed = resolvePreprocessedDir(sessionDirectory, relativeRoot);
    const QDir segmentation(preprocessed.filePath(QStringLiteral("segmentation")));
    const QDir preview(QDir(sessionDirectory).filePath(QStringLiteral("preview")));

    std::vector<StreamImageSection> sections;

    StreamImageSection darkRef;
    darkRef.title = QStringLiteral("Dark reference");
    const QString dark = firstExisting(preprocessed, {QStringLiteral("DARKREF_*_ref_plot.png")});
    if (!dark.isEmpty())
        darkRef.paths.push_back(dark);
    sections.push_back(std::move(darkRef));

    StreamImageSection whiteRef;
    whiteRef.title = QStringLiteral("White reference");
    const QString white = firstExisting(preprocessed, {QStringLiteral("WHITEREF_*_ref_plot.png")});
    if (!white.isEmpty())
        whiteRef.paths.push_back(white);
    sections.push_back(std::move(whiteRef));

    StreamImageSection rgb;
    rgb.title = QStringLiteral("Processed RGB");
    QStringList rgbPaths = collectMatching(preprocessed, {QStringLiteral("*_rgb.png")});
    if (rgbPaths.isEmpty() && preview.exists() && !relativeRoot.trimmed().isEmpty())
    {
        QString slug = relativeRoot.trimmed().toLower();
        slug.replace(QLatin1Char('\\'), QLatin1Char('/'));
        slug.replace(QLatin1Char('/'), QLatin1Char('_'));
        slug.replace(QLatin1Char(' '), QLatin1Char('_'));
        const QString previewRgb = preview.filePath(slug + QStringLiteral("_rgb.png"));
        if (QFileInfo::exists(previewRgb))
            rgbPaths.push_back(previewRgb);
    }
    rgb.paths = std::move(rgbPaths);
    sections.push_back(std::move(rgb));

    StreamImageSection overlay;
    overlay.title = QStringLiteral("Segmentation overview");
    const QString overlayPath = segmentation.filePath(QStringLiteral("overlay.png"));
    if (QFileInfo::exists(overlayPath))
        overlay.paths.push_back(overlayPath);
    sections.push_back(std::move(overlay));

    StreamImageSection spectra;
    spectra.title = QStringLiteral("ROI spectra");
    const QString spectraPath = segmentation.filePath(QStringLiteral("roi_spectra_plot.png"));
    if (QFileInfo::exists(spectraPath))
        spectra.paths.push_back(spectraPath);
    sections.push_back(std::move(spectra));

    return sections;
}

void showImagePreview(QWidget *parent, const QString &path)
{
    QDialog preview(parent);
    preview.setWindowTitle(QFileInfo(path).fileName());
    preview.setMinimumSize(480, 360);
    preview.resize(960, 720);
    preview.setSizeGripEnabled(true);

    auto *layout = new QVBoxLayout(&preview);
    layout->setContentsMargins(8, 8, 8, 8);
    auto *scroll = new QScrollArea(&preview);
    scroll->setWidgetResizable(true);
    auto *image = new QLabel(scroll);
    image->setAlignment(Qt::AlignCenter);
    const QPixmap pixmap(path);
    if (pixmap.isNull())
        image->setText(path);
    else
        image->setPixmap(pixmap);
    scroll->setWidget(image);
    layout->addWidget(scroll, 1);

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Close, &preview);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &preview, &QDialog::reject);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &preview, &QDialog::accept);
    layout->addWidget(buttons);
    preview.exec();
}

class ClickableImageLabel final : public QLabel
{
public:
    ClickableImageLabel(QWidget *parent, QString path)
        : QLabel(parent)
        , path_(std::move(path))
    {
        setCursor(Qt::PointingHandCursor);
    }

protected:
    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton && !path_.isEmpty())
            showImagePreview(window(), path_);
        QLabel::mouseReleaseEvent(event);
    }

private:
    QString path_;
};

QLabel *resizedImageLabel(QWidget *parent, const QString &path, const int maxW, const int maxH)
{
    auto *label = new ClickableImageLabel(parent, path);
    label->setAlignment(Qt::AlignCenter);
    label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    QPixmap pixmap(path);
    if (pixmap.isNull())
    {
        label->setText(QFileInfo(path).fileName());
        return label;
    }
    pixmap = pixmap.scaled(maxW, maxH, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    label->setPixmap(pixmap);
    label->setToolTip(QObject::tr("Click to open preview\n%1").arg(path));
    return label;
}

QWidget *makeSectionCard(QWidget *parent,
                         const StreamImageSection &section,
                         const int maxW,
                         const int maxH)
{
    auto *card = new QFrame(parent);
    card->setFrameShape(QFrame::StyledPanel);
    card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto *layout = new QVBoxLayout(card);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(2);

    auto *title = new QLabel(section.title, card);
    QFont titleFont = title->font();
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    if (section.paths.isEmpty())
    {
        auto *missing = new QLabel(QStringLiteral("Not available."), card);
        missing->setAlignment(Qt::AlignCenter);
        missing->setStyleSheet(QStringLiteral("color: #777777;"));
        layout->addWidget(missing, 1);
        return card;
    }

    for (const QString &path : section.paths)
        layout->addWidget(resizedImageLabel(card, path, maxW, maxH), 1, Qt::AlignCenter);
    return card;
}

QWidget *makeStreamPage(QWidget *parent,
                        const QString &detail,
                        const std::vector<StreamImageSection> &sections)
{
    auto *page = new QWidget(parent);
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);

    auto *detailLabel = new QLabel(detail, page);
    detailLabel->setWordWrap(true);
    layout->addWidget(detailLabel);

    // Expected order: dark, white, rgb, overlay, spectra.
    StreamImageSection dark;
    StreamImageSection white;
    StreamImageSection rgb;
    StreamImageSection overlay;
    StreamImageSection spectra;
    dark.title = QStringLiteral("Dark reference");
    white.title = QStringLiteral("White reference");
    rgb.title = QStringLiteral("Processed RGB");
    overlay.title = QStringLiteral("Segmentation overview");
    spectra.title = QStringLiteral("ROI spectra");
    if (sections.size() > 0)
        dark = sections[0];
    if (sections.size() > 1)
        white = sections[1];
    if (sections.size() > 2)
        rgb = sections[2];
    if (sections.size() > 3)
        overlay = sections[3];
    if (sections.size() > 4)
        spectra = sections[4];

    layout->addWidget(makeSectionCard(page, spectra, kSpectraImageWidth, kSpectraImageHeight), 5);

    auto *lowerHost = new QWidget(page);
    auto *lowerGrid = new QGridLayout(lowerHost);
    lowerGrid->setContentsMargins(0, 0, 0, 0);
    lowerGrid->setHorizontalSpacing(4);
    lowerGrid->setVerticalSpacing(4);
    lowerGrid->setColumnStretch(0, 1);
    lowerGrid->setColumnStretch(1, 1);
    lowerGrid->setRowStretch(0, 1);
    lowerGrid->setRowStretch(1, 1);
    lowerGrid->addWidget(makeSectionCard(lowerHost, dark, kCellImageWidth, kCellImageHeight), 0, 0);
    lowerGrid->addWidget(makeSectionCard(lowerHost, white, kCellImageWidth, kCellImageHeight), 0, 1);
    lowerGrid->addWidget(makeSectionCard(lowerHost, rgb, kCellImageWidth, kCellImageHeight), 1, 0);
    lowerGrid->addWidget(makeSectionCard(lowerHost, overlay, kCellImageWidth, kCellImageHeight), 1, 1);
    layout->addWidget(lowerHost, 4);
    return page;
}
} // namespace

void CaptureSessionSummaryDialog::execForSession(QWidget *parent,
                                                 const CaptureWriterSessionSummary &summary,
                                                 const QString &extraDetails)
{
    CaptureSessionSummaryDialog dialog(parent);
    dialog.setWindowTitle(QObject::tr("Recording complete"));
    dialog.setMinimumSize(640, 560);
    dialog.resize(kDialogWidth, kDialogHeight);
    dialog.setSizeGripEnabled(true);

    auto *layout = new QVBoxLayout(&dialog);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(4);
    auto *lead = new QLabel(QObject::tr("Capture scan sequence finished successfully."), &dialog);
    lead->setWordWrap(true);
    layout->addWidget(lead);

    if (!summary.sessionDirectory.isEmpty())
    {
        auto *sessionLabel = new QLabel(summary.sessionDirectory, &dialog);
        sessionLabel->setWordWrap(true);
        sessionLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
        layout->addWidget(sessionLabel);
    }

    if (!extraDetails.trimmed().isEmpty())
    {
        auto *extra = new QLabel(extraDetails, &dialog);
        extra->setWordWrap(true);
        extra->setTextInteractionFlags(Qt::TextSelectableByMouse);
        layout->addWidget(extra);
    }

    auto *tabs = new QTabWidget(&dialog);
    std::vector<const CaptureWriterStreamSummary *> ordered;
    for (auto it = summary.streams.cbegin(); it != summary.streams.cend(); ++it)
        ordered.push_back(&it.value());
    std::sort(ordered.begin(), ordered.end(),
              [](const CaptureWriterStreamSummary *a, const CaptureWriterStreamSummary *b) {
                  const int ra = streamSortRank(a->relativeRoot);
                  const int rb = streamSortRank(b->relativeRoot);
                  if (ra != rb)
                      return ra < rb;
                  return a->relativeRoot.compare(b->relativeRoot, Qt::CaseInsensitive) < 0;
              });

    for (const CaptureWriterStreamSummary *stream : ordered)
    {
        const QString title = stream->relativeRoot.isEmpty() ? stream->baseName : stream->relativeRoot;
        const QString detail = QStringLiteral("%1 sample frames").arg(stream->frameCount);
        const auto sections =
            collectStreamImageSections(summary.sessionDirectory, stream->relativeRoot);
        tabs->addTab(makeStreamPage(&dialog, detail, sections), title);
    }

    if (tabs->count() == 0)
    {
        auto *empty = new QLabel(QObject::tr("No capture streams to summarize."), &dialog);
        empty->setAlignment(Qt::AlignCenter);
        layout->addWidget(empty, 1);
    }
    else
    {
        layout->addWidget(tabs, 1);
    }

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok, &dialog);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    layout->addWidget(buttons);

    dialog.exec();
}
