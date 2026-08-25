// One-time UR3e WSL network setup (elevated PowerShell) and stale-process cleanup.
#include "backend/multiview/Ur3eWslSetup.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eWslPathUtil.hpp"

#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QThread>

#ifdef Q_OS_WIN
#include <windows.h>
#include <shellapi.h>

#include <string>
#endif

namespace hf::ur3e
{
namespace
{
QString wslDistroArgument()
{
    return hf::hardwareConfig().ur3e.wslDistro.trimmed();
}

QStringList wslBashArguments(const QString &script)
{
    QStringList arguments;
    const QString distro = wslDistroArgument();
    if (!distro.isEmpty())
        arguments << QStringLiteral("-d") << distro;
    arguments << QStringLiteral("--") << QStringLiteral("bash") << QStringLiteral("-lc") << script;
    return arguments;
}

bool runWslBashScript(const QString &script, const int timeoutMs, QString *detail)
{
    QProcess process;
    process.setProgram(QStringLiteral("wsl.exe"));
    process.setArguments(wslBashArguments(script));
    process.start();
    if (!process.waitForStarted(10000))
    {
        if (detail != nullptr)
            *detail = QStringLiteral("Failed to start wsl.exe: %1").arg(process.errorString());
        return false;
    }
    if (!process.waitForFinished(timeoutMs))
    {
        process.kill();
        process.waitForFinished(2000);
        if (detail != nullptr)
            *detail = QStringLiteral("WSL command timed out.");
        return false;
    }
    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
    {
        const QString stderrText = QString::fromUtf8(process.readAllStandardError()).trimmed();
        if (detail != nullptr)
        {
            *detail = stderrText.isEmpty()
                          ? QStringLiteral("WSL command failed (exit %1).").arg(process.exitCode())
                          : stderrText;
        }
        return false;
    }
    return true;
}

bool waitForWslReady(QString *detail)
{
    for (int attempt = 0; attempt < 30; ++attempt)
    {
        if (runWslBashScript(QStringLiteral("exit 0"), 15000, nullptr))
            return true;
        QThread::msleep(1000);
    }

    if (detail != nullptr)
        *detail = QStringLiteral("WSL did not become ready after network setup.");
    return false;
}
} // namespace

QString resolveWslNetworkSetupScriptPath()
{
    const QString resourcesRoot = resolveUr3eResourcesWindowsPath();
    if (resourcesRoot.isEmpty())
        return {};

    const QString scriptPath =
        QDir(resourcesRoot).filePath(QStringLiteral("scripts/setup_wsl_robot_network.ps1"));
    if (!QFileInfo::exists(scriptPath))
        return {};

    return QFileInfo(scriptPath).absoluteFilePath();
}

bool runElevatedWslNetworkSetup(QString *detail)
{
    const QString scriptPath = resolveWslNetworkSetupScriptPath();
    if (scriptPath.isEmpty())
    {
        if (detail != nullptr)
            *detail = QStringLiteral("UR3e network setup script not found (resources/ur3e/scripts).");
        return false;
    }

#ifdef Q_OS_WIN
    const QString args = QStringLiteral("-NoProfile -ExecutionPolicy Bypass -File \"%1\" -ShutdownWsl")
                             .arg(scriptPath);

    SHELLEXECUTEINFOW shellInfo = {};
    shellInfo.cbSize = sizeof(shellInfo);
    shellInfo.fMask = SEE_MASK_NOCLOSEPROCESS | SEE_MASK_NOASYNC;
    shellInfo.lpVerb = L"runas";
    shellInfo.lpFile = L"powershell.exe";
    const std::wstring wideArgs = args.toStdWString();
    shellInfo.lpParameters = wideArgs.c_str();
    shellInfo.nShow = SW_SHOWMINNOACTIVE;

    if (!ShellExecuteExW(&shellInfo))
    {
        const DWORD errorCode = GetLastError();
        if (detail != nullptr)
        {
            if (errorCode == static_cast<DWORD>(ERROR_CANCELLED))
                *detail = QStringLiteral("UR3e network setup cancelled (Administrator approval required).");
            else
                *detail = QStringLiteral("Failed to launch elevated network setup (error %1).").arg(errorCode);
        }
        return false;
    }

    if (shellInfo.hProcess != nullptr)
    {
        WaitForSingleObject(shellInfo.hProcess, 180000);
        CloseHandle(shellInfo.hProcess);
    }

    if (detail != nullptr)
        *detail = QStringLiteral("UR3e WSL network setup completed.");
    return true;
#else
    if (detail != nullptr)
        *detail = QStringLiteral("UR3e network setup is only supported on Windows.");
    return false;
#endif
}

bool runStaleUr3eProcessCleanup(QString *detail)
{
    const QString repoLinux = resolveUr3eRepoLinuxPath();
    if (repoLinux.isEmpty())
    {
        if (detail != nullptr)
            *detail = QStringLiteral("Could not locate resources/ur3e for stale-process cleanup.");
        return false;
    }

    const QString cleanup =
        QStringLiteral("bash '%1/scripts/kill_stale_ur_ros.sh'").arg(repoLinux);

    if (!runWslBashScript(cleanup, 20000, detail))
        return false;

    if (detail != nullptr)
        *detail = QStringLiteral("Stale UR3e WSL processes cleaned.");
    return true;
}

bool runUr3eStartupSetupOnce(QString *detail)
{
    const bool mockHardware = hf::hardwareConfig().ur3e.useMockHardware;

    QString networkDetail;
    bool networkOk = true;
    if (!mockHardware)
        networkOk = runElevatedWslNetworkSetup(&networkDetail);
    else if (detail != nullptr)
        networkDetail = QStringLiteral("Skipping UR3e LAN network setup (simulation mode).");

    QString wslReadyDetail;
    bool wslReady = true;
    if (mockHardware)
    {
        // Simulation does not need mirrored networking or the long post-shutdown retry loop.
        wslReady = runWslBashScript(QStringLiteral("exit 0"), 8000, &wslReadyDetail);
    }
    else
    {
        wslReady = waitForWslReady(&wslReadyDetail);
    }

    // Stale-process cleanup runs in Ur3eServerManager::beginAsyncCleanup before sidecar launch.

    if (detail != nullptr)
    {
        QStringList parts;
        if (!networkDetail.isEmpty())
            parts << networkDetail;
        if (!wslReady && !wslReadyDetail.isEmpty())
            parts << wslReadyDetail;
        *detail = parts.join(QStringLiteral(" "));
    }

    return networkOk && wslReady;
}

} // namespace hf::ur3e
