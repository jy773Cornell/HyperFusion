// Auto hemisphere scan execute helpers (separate from semi-fixed ring execute).

#include "backend/3dscanning/Ur3eAutoHemisphereScanExecute.hpp"

namespace hf::ur3e
{
// Mode helpers are header-inline. Auto pin/wrist session loop stays in
// Ur3ePanelController::startHemisphereScanExecute so existing Capture integration
// is unchanged. Semi-fixed uses Ur3eSemiFixedScanExecute::runSemiFixedScanExecute.
} // namespace hf::ur3e
