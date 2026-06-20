<#
  Dev helper for the communicator prop firmware (Windows / ESP-IDF 6.0.1 via EIM).
  Activates the IDF environment, forces UTF-8 (avoids a component-manager emoji
  crash), and runs a build/flash/monitor action against the project.

  Usage (from anywhere):
    pwsh firmware/communicator/tools/dev.ps1 build
    pwsh firmware/communicator/tools/dev.ps1 flash   -Port COM7
    pwsh firmware/communicator/tools/dev.ps1 bf      -Port COM7   # build + flash
    pwsh firmware/communicator/tools/dev.ps1 bfw     -Port COM7   # build + flash + wait until API answers
    pwsh firmware/communicator/tools/dev.ps1 monitor -Port COM7

  After flashing, drive/inspect the UI with tools/prop.py (see the communicator-ui skill):
    python tools/prop.py shot out.png --screen spectrum --wait
#>
param(
  [ValidateSet("build", "flash", "bf", "bfw", "monitor", "reconfigure")]
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
  "bfw"         { idf.py -C $proj build; if ($LASTEXITCODE -eq 0) { idf.py -C $proj -p $Port flash; if ($LASTEXITCODE -eq 0) { python "$PSScriptRoot\prop.py" wait } } }
  "monitor"     { idf.py -C $proj -p $Port monitor }
  "reconfigure" { idf.py -C $proj reconfigure }
}
