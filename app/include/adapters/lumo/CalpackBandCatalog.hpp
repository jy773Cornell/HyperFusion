// Spectral band catalog loaded from Specim .scp calibration packs (ZIP → wlcal1b.wls).
// Used to populate RGB band combo boxes in the camera settings UI.
#pragma once

#include <QString>

#include <cstdint>
#include <string>
#include <vector>

struct SpectralBand
{
    int index = 0;
    double wavelengthNm = 0.0;
    double fwhmNm = 0.0;
};

class CalpackBandCatalog
{
public:
    /// spectralBinning: 1, 2, 4, or 8 — selects spectral/wlcal{b}b.wls inside the .scp ZIP.
    static bool loadFromCalpack(const QString &calpackPath,
                                int spectralBinning,
                                std::vector<SpectralBand> &bands,
                                std::string &errorMessage);

    static QString formatBandLabel(const SpectralBand &band);
    static int comboIndexForBand(const std::vector<SpectralBand> &bands, int bandIndex);
};
