// Application branding: window icon and display metadata (frontend layer).
#include "frontend/utils/AppBranding.hpp"

#include <QApplication>
#include <QIcon>

namespace hf::ui
{
void applyApplicationBranding(QApplication &app)
{
    app.setApplicationName(QStringLiteral("HyperFusion"));
    app.setOrganizationName(QStringLiteral("HyperFusion"));
    app.setApplicationDisplayName(QStringLiteral("HyperFusion"));

    const QIcon icon(QStringLiteral(":/HyperFusionLogo.png"));
    if (!icon.isNull())
        app.setWindowIcon(icon);
}

} // namespace hf::ui
