// Recording-complete summary dialog (frontend/ui): tab per mode/camera.
// Each page is a master-detail gallery: clickable thumbs left, large image right.
#include "frontend/widgets/CaptureSessionSummaryDialog.hpp"

#include <QCursor>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPixmap>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSizePolicy>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

CaptureSessionSummaryDialog::CaptureSessionSummaryDialog(QWidget *parent)
    : QDialog(parent)
{
}

namespace
{
constexpr int kThumbWidth = 176;
constexpr int kThumbHeight = 110;
constexpr int kSidebarWidth = 204;
constexpr int kDialogWidth = 1080;
constexpr int kDialogHeight = 720;

struct GalleryItem
{
    QString title;
    QString path;
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

void appendIfExists(std::vector<GalleryItem> *items, const QString &title, const QString &path)
{
    if (items == nullptr || path.isEmpty() || !QFileInfo::exists(path))
        return;
    items->push_back(GalleryItem{title, path});
}

std::vector<GalleryItem> collectStreamGalleryItems(const QString &sessionDirectory,
                                                   const QString &relativeRoot)
{
    const QDir preprocessed = resolvePreprocessedDir(sessionDirectory, relativeRoot);
    const QDir segmentation(preprocessed.filePath(QStringLiteral("segmentation")));
    const QDir preview(QDir(sessionDirectory).filePath(QStringLiteral("preview")));

    std::vector<GalleryItem> items;
    appendIfExists(&items, QStringLiteral("ROI spectra"),
                   segmentation.filePath(QStringLiteral("roi_spectra_plot.png")));

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
    if (rgbPaths.size() == 1)
        appendIfExists(&items, QStringLiteral("Processed RGB"), rgbPaths.front());
    else
    {
        for (const QString &path : rgbPaths)
            appendIfExists(&items, QFileInfo(path).completeBaseName(), path);
    }

    appendIfExists(&items, QStringLiteral("Segmentation overlay"),
                   segmentation.filePath(QStringLiteral("overlay.png")));
    appendIfExists(&items, QStringLiteral("Dark reference"),
                   firstExisting(preprocessed, {QStringLiteral("DARKREF_*_ref_plot.png")}));
    appendIfExists(&items, QStringLiteral("White reference"),
                   firstExisting(preprocessed, {QStringLiteral("WHITEREF_*_ref_plot.png")}));
    return items;
}

class ScaledPixmapLabel final : public QLabel
{
public:
    explicit ScaledPixmapLabel(QWidget *parent)
        : QLabel(parent)
    {
        setAlignment(Qt::AlignCenter);
        setMinimumSize(240, 180);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        setStyleSheet(QStringLiteral("background: #111111; color: #aaaaaa;"));
    }

    void setSourcePath(const QString &path)
    {
        path_ = path;
        source_ = path.isEmpty() ? QPixmap() : QPixmap(path);
        if (source_.isNull())
        {
            setPixmap(QPixmap());
            setText(path.isEmpty() ? QStringLiteral("Not available.") : QFileInfo(path).fileName());
            setToolTip({});
            return;
        }
        setText({});
        setToolTip(path);
        updateScaled();
    }

protected:
    void resizeEvent(QResizeEvent *event) override
    {
        QLabel::resizeEvent(event);
        updateScaled();
    }

private:
    void updateScaled()
    {
        if (source_.isNull())
            return;
        const QSize avail = size();
        if (avail.width() < 8 || avail.height() < 8)
            return;
        setPixmap(source_.scaled(avail, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

    QString path_;
    QPixmap source_;
};

class ThumbnailCard final : public QFrame
{
public:
    ThumbnailCard(QWidget *parent, const GalleryItem &item)
        : QFrame(parent)
        , path_(item.path)
    {
        setObjectName(QStringLiteral("SummaryThumb"));
        setCursor(Qt::PointingHandCursor);
        setFixedWidth(kSidebarWidth - 20);
        setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

        auto *layout = new QVBoxLayout(this);
        layout->setContentsMargins(4, 4, 4, 4);
        layout->setSpacing(2);

        auto *image = new QLabel(this);
        image->setAlignment(Qt::AlignCenter);
        image->setFixedSize(kThumbWidth, kThumbHeight);
        image->setStyleSheet(QStringLiteral("background: #1a1a1a;"));
        const QPixmap pixmap(item.path);
        if (pixmap.isNull())
            image->setText(QStringLiteral("—"));
        else
            image->setPixmap(pixmap.scaled(kThumbWidth, kThumbHeight, Qt::KeepAspectRatio,
                                           Qt::SmoothTransformation));
        layout->addWidget(image);

        auto *caption = new QLabel(item.title, this);
        caption->setAlignment(Qt::AlignCenter);
        caption->setWordWrap(true);
        QFont captionFont = caption->font();
        captionFont.setPointSize(qMax(8, captionFont.pointSize() - 1));
        caption->setFont(captionFont);
        layout->addWidget(caption);

        setToolTip(item.path);
        setSelected(false);
    }

    const QString &path() const { return path_; }

    void setSelected(const bool on)
    {
        setStyleSheet(on ? QStringLiteral(
                               "QFrame#SummaryThumb { border: 2px solid #2b7de9; background: #e8f1fc; }")
                         : QStringLiteral(
                               "QFrame#SummaryThumb { border: 1px solid #c8c8c8; background: #f4f4f4; }"));
    }

    std::function<void()> onClicked;

protected:
    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton && onClicked)
            onClicked();
        QFrame::mouseReleaseEvent(event);
    }

private:
    QString path_;
};

class SummaryGalleryPage final : public QWidget
{
public:
    SummaryGalleryPage(QWidget *parent, const QString &detail, const std::vector<GalleryItem> &items)
        : QWidget(parent)
    {
        auto *root = new QVBoxLayout(this);
        root->setContentsMargins(4, 4, 4, 4);
        root->setSpacing(4);

        if (!detail.trimmed().isEmpty())
        {
            auto *detailLabel = new QLabel(detail, this);
            detailLabel->setWordWrap(true);
            root->addWidget(detailLabel);
        }

        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        auto *row = new QWidget(this);
        row->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto *rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(8);

        auto *sideScroll = new QScrollArea(row);
        sideScroll->setWidgetResizable(true);
        sideScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        sideScroll->setFixedWidth(kSidebarWidth);
        sideScroll->setFrameShape(QFrame::NoFrame);

        auto *sideHost = new QWidget(sideScroll);
        auto *sideLayout = new QVBoxLayout(sideHost);
        sideLayout->setContentsMargins(2, 2, 2, 2);
        sideLayout->setSpacing(6);

        thumbs_.reserve(static_cast<int>(items.size()));
        for (int i = 0; i < static_cast<int>(items.size()); ++i)
        {
            auto *thumb = new ThumbnailCard(sideHost, items[i]);
            const int index = i;
            thumb->onClicked = [this, index]() { select(index); };
            sideLayout->addWidget(thumb, 0, Qt::AlignHCenter);
            thumbs_.push_back(thumb);
        }
        sideLayout->addStretch(1);
        sideScroll->setWidget(sideHost);
        rowLayout->addWidget(sideScroll, 0);

        auto *mainFrame = new QFrame(row);
        mainFrame->setFrameShape(QFrame::StyledPanel);
        mainFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto *mainLayout = new QVBoxLayout(mainFrame);
        mainLayout->setContentsMargins(4, 4, 4, 4);
        main_ = new ScaledPixmapLabel(mainFrame);
        mainLayout->addWidget(main_, 1);
        rowLayout->addWidget(mainFrame, 1);

        root->addWidget(row, 1);

        if (!items.empty())
            select(0);
        else
            main_->setSourcePath({});
    }

private:
    void select(const int index)
    {
        if (index < 0 || index >= thumbs_.size())
            return;
        for (int i = 0; i < thumbs_.size(); ++i)
            thumbs_[i]->setSelected(i == index);
        main_->setSourcePath(thumbs_[index]->path());
    }

    ScaledPixmapLabel *main_ = nullptr;
    QVector<ThumbnailCard *> thumbs_;
};

QWidget *makeGalleryPage(QWidget *parent, const QString &detail, const std::vector<GalleryItem> &items)
{
    return new SummaryGalleryPage(parent, detail, items);
}
} // namespace

void CaptureSessionSummaryDialog::execForSession(QWidget *parent,
                                                 const CaptureWriterSessionSummary &summary,
                                                 const QString &extraDetails)
{
    CaptureSessionSummaryDialog dialog(parent);
    dialog.setWindowTitle(QObject::tr("Recording complete"));
    dialog.setWindowFlags(dialog.windowFlags() | Qt::WindowMinMaxButtonsHint);
    dialog.setSizeGripEnabled(true);
    dialog.setMinimumSize(640, 420);
    dialog.resize(kDialogWidth, kDialogHeight);

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
    tabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
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
        const auto items = collectStreamGalleryItems(summary.sessionDirectory, stream->relativeRoot);
        tabs->addTab(makeGalleryPage(&dialog, detail, items), title);
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
