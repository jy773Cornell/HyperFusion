// One-time UR3e WSL network setup (elevated PowerShell) and stale-process cleanup.
#pragma once

#include <QString>

namespace hf::ur3e
{
/// Absolute path to setup_wsl_robot_network.ps1, or empty if not found.
QString resolveWslNetworkSetupScriptPath();

/// Request Administrator approval and run network setup (idempotent). Returns false if denied or failed.
bool runElevatedWslNetworkSetup(QString *detail = nullptr);

/// Kill leftover UR3e / ur_control WSL processes (non-elevated).
bool runStaleUr3eProcessCleanup(QString *detail = nullptr);

/// Network setup (elevated) then stale-process cleanup. Safe to call once per app start.
bool runUr3eStartupSetupOnce(QString *detail = nullptr);

} // namespace hf::ur3e
