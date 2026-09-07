# One-time Windows + WSL network setup for UR3e real-robot control.
# The ROS driver runs in WSL; the robot must reach the PC reverse ports on the LAN NIC.
# Idempotent - safe to re-run. Does not run wsl --shutdown unless -ShutdownWsl is passed.
# Usage (from repo):
#   .\app\sidecars\ur3e\scripts\setup_wsl_robot_network.ps1
# Firewall rules require an elevated (Administrator) PowerShell.
param(
    [switch]$SkipFirewall,
    [switch]$ShutdownWsl,
    [int[]]$ReversePorts = @(50001, 50002, 50003, 50004)
)

$ErrorActionPreference = "Stop"

function Test-IsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Ensure-WslMirroredNetworking {
    $wslConfigPath = Join-Path $env:USERPROFILE ".wslconfig"
    $desiredLines = @(
        "networkingMode=mirrored",
        "localhostForwarding=true"
    )
    $sectionHeader = "[wsl2]"
    $headerComment = "# HyperFusion UR3e - allow robot (External Control) to reach the WSL ROS driver."

    $needsRewrite = $true
    if (Test-Path -LiteralPath $wslConfigPath) {
        $lines = Get-Content -LiteralPath $wslConfigPath
        $hasMirrored = $false
        $hasLocalhostForwarding = $false
        foreach ($line in $lines) {
            if ($line -match '^\s*networkingMode\s*=\s*mirrored\s*$') { $hasMirrored = $true }
            if ($line -match '^\s*localhostForwarding\s*=\s*true\s*$') { $hasLocalhostForwarding = $true }
        }
        $needsRewrite = -not ($hasMirrored -and $hasLocalhostForwarding)
    }

    if (-not $needsRewrite) {
        Write-Host "==> $wslConfigPath already has mirrored networking settings"
        return $false
    }

    $otherLines = @()
    if (Test-Path -LiteralPath $wslConfigPath) {
        $skipWsl2Body = $false
        foreach ($line in (Get-Content -LiteralPath $wslConfigPath)) {
            if ($line -match '^\s*\[wsl2\]\s*$') {
                $skipWsl2Body = $true
                continue
            }
            if ($skipWsl2Body) {
                if ($line -match '^\s*\[') { $skipWsl2Body = $false }
                elseif ($line -match '^\s*(networkingMode|localhostForwarding)\s*=') { continue }
                else { continue }
            }
            if ($line -match '^\s*# HyperFusion UR3e') { continue }
            if ($line.Trim().Length -gt 0) { $otherLines += $line }
        }
    }

  @(
        $headerComment
        $sectionHeader
        $desiredLines
        ""
        $otherLines
    ) | Set-Content -LiteralPath $wslConfigPath -Encoding utf8
    Write-Host "==> Updated $wslConfigPath (networkingMode=mirrored, localhostForwarding=true)"
    return $true
}

function Remove-UrPortProxyRules {
    param(
        [int[]]$Ports,
        [string]$ListenAddress = "0.0.0.0"
    )

    # Mirrored WSL shares the Windows LAN IP with the ROS driver. Portproxy to
    # 127.0.0.1 steals :50001-50004 on the LAN NIC and never reaches reverse_ip
    # listeners — External Control / controller manager hang and Connect stays disabled.
    if (-not (Test-IsAdministrator)) {
        Write-Warning @'
Cannot remove stale portproxy rules (needs Administrator PowerShell).
If Connect never enables with mirrored WSL, run as admin:
  netsh interface portproxy delete v4tov4 listenaddress=0.0.0.0 listenport=50002
(repeat for 50001, 50003, 50004)
'@
        return
    }

    foreach ($port in $Ports) {
        $existing = netsh interface portproxy show v4tov4 | Select-String -Pattern "^\s*$ListenAddress\s+$port\s+"
        if (-not $existing) {
            continue
        }
        netsh interface portproxy delete v4tov4 `
            listenaddress=$ListenAddress `
            listenport=$port | Out-Null
        Write-Host "==> Removed stale port proxy: ${ListenAddress}:$port (not used with mirrored WSL)"
    }
}

function Ensure-UrReverseFirewallRules {
    param([int[]]$Ports)

    if ($SkipFirewall) {
        Write-Host "==> Skipping firewall rules (-SkipFirewall)"
        return
    }

    if (-not (Test-IsAdministrator)) {
        Write-Warning @'
Firewall rules were not added (requires Administrator PowerShell).
Re-run as admin, or run manually for each port (50001-50004):
  New-NetFirewallRule -DisplayName "UR Reverse PORT" -Direction Inbound -Protocol TCP -LocalPort PORT -Action Allow
'@
        return
    }

    foreach ($port in $Ports) {
        $ruleName = "HyperFusion UR Reverse $port"
        $existing = Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue
        if ($existing) {
            Write-Host "==> Firewall rule already exists: $ruleName"
            continue
        }
        New-NetFirewallRule `
            -DisplayName $ruleName `
            -Direction Inbound `
            -Protocol TCP `
            -LocalPort $port `
            -Action Allow `
            -Profile Any | Out-Null
        Write-Host "==> Added firewall rule: $ruleName (TCP $port inbound)"
    }
}

Write-Host "==> HyperFusion UR3e - WSL robot network setup"
$wslConfigChanged = Ensure-WslMirroredNetworking
Ensure-UrReverseFirewallRules -Ports $ReversePorts
# Mirrored mode: driver binds reverse_ip directly — do not add portproxy.
Remove-UrPortProxyRules -Ports $ReversePorts

if ($ShutdownWsl -and $wslConfigChanged) {
    Write-Host "==> Shutting down WSL (applies .wslconfig changes)..."
    & wsl.exe --shutdown
    Write-Host "==> WSL shutdown complete. Re-open Ubuntu before connecting to the robot."
} elseif ($ShutdownWsl -and -not $wslConfigChanged) {
    Write-Host "==> Skipping WSL shutdown (.wslconfig unchanged)."
} elseif ($wslConfigChanged) {
    Write-Host ""
    Write-Host "IMPORTANT: Restart WSL so mirrored networking takes effect:"
    Write-Host "  wsl --shutdown"
    Write-Host "Then reopen Ubuntu and verify:"
    Write-Host "  ip -4 addr show"
    Write-Host "  ping -c 2 192.168.1.10"
}

Write-Host ""
Write-Host "Teach pendant External Control: set remote PC IP to your robot-LAN adapter (e.g. 192.168.1.20)."
Write-Host "Verify from Windows (after driver is running):"
Write-Host "  Test-NetConnection -ComputerName 192.168.1.20 -Port 50002"
Write-Host "Done."
