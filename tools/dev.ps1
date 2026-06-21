<#
  Dev helper for the communicator prop firmware (Windows / ESP-IDF 6.0.1 via EIM).
  Activates the IDF environment, forces UTF-8 (avoids a component-manager emoji
  crash), and runs a build/flash/monitor action against the project.

  Usage (from anywhere):
    pwsh tools/dev.ps1 build
    pwsh tools/dev.ps1 flash   -Port COM7
    pwsh tools/dev.ps1 bf      -Port COM7   # build + flash
    pwsh tools/dev.ps1 bfw     -Port COM7   # build + flash + wait until API answers
    pwsh tools/dev.ps1 monitor -Port COM7
    pwsh tools/dev.ps1 ota                  # build + OTA push to mDNS host
    pwsh tools/dev.ps1 ota    -DeviceHost 172.17.2.167   # explicit IP

  After flashing, drive/inspect the UI with tools/prop.py (see the communicator-ui skill):
    python tools/prop.py shot out.png --screen spectrum --wait
#>
param(
  [ValidateSet("build", "flash", "bf", "bfw", "monitor", "reconfigure", "ota")]
  [string]$Action = "build",
  [string]$Port = $null,
  [string]$DeviceHost = "comm-unit-7.local",
  [string]$Token = "prop-ota-2024"
)

& "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"
$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONUTF8 = "1"

if ($null -eq $Port -or $Port -eq "") {
  try {
    $portDevice = Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -like "*CH340*" -or $_.Caption -like "*CH340*" -or $_.Name -like "*CH341*" } | Select-Object -First 1
    if ($portDevice -and $portDevice.Name -match '\((COM\d+)\)') {
        $Port = $Matches[1]
        Write-Host "Auto-detected CH340 port: $Port"
    } else {
        $Port = "COM7"
    }
  } catch {
    $Port = "COM7"
  }
}

$proj = Split-Path $PSScriptRoot -Parent

switch ($Action) {
  "build"       { idf.py -C $proj build }
  "flash"       { idf.py -C $proj -p $Port flash }
  "bf"          { idf.py -C $proj build; if ($LASTEXITCODE -eq 0) { idf.py -C $proj -p $Port flash } }
  "bfw"         { idf.py -C $proj build; if ($LASTEXITCODE -eq 0) { idf.py -C $proj -p $Port flash; if ($LASTEXITCODE -eq 0) { python "$PSScriptRoot\prop.py" wait } } }
  "monitor"     { idf.py -C $proj -p $Port monitor }
  "reconfigure" { idf.py -C $proj reconfigure }
  "ota" {
    idf.py -C $proj build
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    $bin = "$proj\build\communicator.bin"
    $url = "http://$DeviceHost/ota?token=$Token"
    Write-Host "OTA: pushing $bin -> $url"
    $resp = Invoke-WebRequest -Method Post -InFile $bin -Uri $url -TimeoutSec 120 -UseBasicParsing
    Write-Host "OTA: $($resp.Content)"
    Write-Host "OTA: device is rebooting into new firmware"
  }
}
