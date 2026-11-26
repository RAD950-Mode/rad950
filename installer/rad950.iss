; RAD-950 Desktop Windows Installer (Inno Setup 6)
; Per-user installation (no admin UAC prompt required), Start Menu & optional Desktop shortcut.

#ifndef AppVersion
  #define AppVersion "0.1.5"
#endif

[Setup]
AppId={{8D6E4C9A-7B2F-4F90-9A15-950RAD950APP}}
AppName=RAD-950
AppVersion={#AppVersion}
AppPublisher=RAD950.org
AppPublisherURL=https://rad950.org
AppSupportURL=https://github.com/RAD950-Mode/rad950/issues
DefaultDirName={localappdata}\Programs\RAD-950
DefaultGroupName=RAD-950
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
OutputDir=..\dist-installer
OutputBaseFilename=rad950-windows-x64-setup
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
ArchitecturesInstallIn64BitMode=x64compatible

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Additional shortcuts:"

[Files]
Source: "..\dist\rad950.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\README.md"; DestDir: "{app}"; Flags: ignoreversion skipifsourcedoesntexist

[Icons]
Name: "{group}\RAD-950"; Filename: "{app}\rad950.exe"; WorkingDir: "{app}"
Name: "{group}\Uninstall RAD-950"; Filename: "{uninstallexe}"
Name: "{autodesktop}\RAD-950"; Filename: "{app}\rad950.exe"; WorkingDir: "{app}"; Tasks: desktopicon

[Run]
Filename: "{app}\rad950.exe"; Description: "Launch RAD-950"; Flags: nowait postinstall skipifsilent
