<#
  Dev helper for the communicator prop firmware (Windows / ESP-IDF 6.0.1 via EIM).
  Activates the IDF environment, forces UTF-8 (avoids a component-manager emoji
  crash), and runs a build/flash/monitor action against the project.

  Usage (from anywhere):
    pwsh firmware/communicator/tools/dev.ps1 build
    pwsh firmware/communicator/tools/dev.ps1 flash   -Port COM7
    pwsh firmware/communicator/tools/dev.ps1 bf      -Port COM7   # build + flash
    pwsh firmware/communicator/tools/dev.ps1 monitor -Port COM7
#>
param(
  [ValidateSet("build", "flash", "bf", "monitor", "reconfigure")]
  [string]$Action = "build",
  [string]$Port = "COM7"
)

& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"
$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONUTF8 = "1"

$proj = Split-Path $PSScriptRoot -Parent

switch ($Action) {
  "build"       { idf.py -C $proj build }
  "flash"       { idf.py -C $proj -p $Port flash }
  "bf"          { idf.py -C $proj build; if ($LASTEXITCODE -eq 0) { idf.py -C $proj -p $Port flash } }
  "monitor"     { idf.py -C $proj -p $Port monitor }
  "reconfigure" { idf.py -C $proj reconfigure }
}
