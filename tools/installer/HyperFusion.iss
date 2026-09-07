; HyperFusion Windows installer (Inno Setup 6).
; Compiled by tools/package_release.ps1 when ISCC.exe is on the machine.
; Copies the app, then runs Install-HyperFusion.ps1 which skips APIs already present.

#define MyAppName "HyperFusion"
#define MyAppVersion "0.1"
#define MyAppPublisher "HyperFusion"
#define MyAppExeName "app.exe"

[Setup]
AppId={{A7C4E2B1-9F18-4D3A-8C11-5E8B2A1D4C09}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={sd}\HyperFusion
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
OutputDir=..
OutputBaseFilename=HyperFusion-Setup
Compression=lzma
SolidCompression=yes
PrivilegesRequired=admin
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
WizardStyle=modern

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Files]
Source: "payload\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "vendor\*"; DestDir: "{tmp}\hf-vendor"; Flags: ignoreversion recursesubdirs createallsubdirs skipifsourcedoesntexist
Source: "Install-HyperFusion.ps1"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; GroupDescription: "Additional icons:"

[Run]
Filename: "powershell.exe"; \
  Parameters: "-NoProfile -ExecutionPolicy Bypass -File ""{app}\Install-HyperFusion.ps1"" -InstallDir ""{app}"" -VendorDir ""{tmp}\hf-vendor"" -SkipPayloadCopy"; \
  Flags: waituntilterminated; \
  StatusMsg: "Checking vendor APIs, Python, WSL, GSAM2, and UR3e..."
Filename: "{app}\{#MyAppExeName}"; Description: "Launch HyperFusion"; Flags: nowait postinstall skipifsilent
