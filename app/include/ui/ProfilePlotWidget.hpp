// DN profile plot (wavelength or spatial) with fixed 12-bit Y axis and nine ticks.
#pragma once

#include <QString>
#include <QWidget>

#include <cstdint>
#include <vector>

namespace ui
{
class ProfilePlotWidget final : public QWidget
{
    Q_OBJECT

public:
    enum class Mode
    {
        Wavelength,
        Spatial
    };

    explicit ProfilePlotWidget(Mode mode, QWidget *parent = nullptr);

    void setProfile(const std::vector<std::uint16_t> &dnValues, int xMaxInclusive);
    /// Wavelength mode: optional nm lookup per band index for X tick labels (size >= band count).
    void setWavelengthAxis(const std::vector<double> &wavelengthNmByBand);
    void setRgbBandMarkers(int redBand, int greenBand, int blueBand);
    void clearDisplay(const QString &message = QString());

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void drawGridAndAxes(QPainter &painter, const QRect &plotRect) const;
    void drawProfile(QPainter &painter, const QRect &plotRect) const;
    void drawBandMarkers(QPainter &painter, const QRect &plotRect) const;
    static std::vector<double> nineTickValues(double minValue, double maxValue);

    Mode mode_;
    QString disconnectedMessage_;
    std::vector<std::uint16_t> dnValues_;
    int xMaxInclusive_ = 0;
    bool hasProfile_ = false;
    int redBandMarker_ = -1;
    int greenBandMarker_ = -1;
    int blueBandMarker_ = -1;
    std::vector<double> wavelengthNmByBand_;
};
} // namespace ui
