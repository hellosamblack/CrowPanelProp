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
  [ValidateSet("build", "flash", "bf", "bfw", "monitor", "reconfigure", "ota","list" )]
  [string]$Action = "build",
  [string]$Port = $null,
  [string]$DeviceHost = "comm-unit-7.local",
  [string]$Token = "prop-ota-2024"
)

function Get-IDFActivationScript {
  # 1. Check if idf.py is already available in PATH
  if (Get-Command "idf.py" -ErrorAction SilentlyContinue) {
    Write-Host "idf.py is already available in PATH."
    return $null
  }

  # 2. Check the default path first
  $defaultPath = "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"
  if (Test-Path $defaultPath) {
    Write-Host "Found default IDF activation script: $defaultPath"
    return $defaultPath
  }

  # 3. Check eim_idf.json in common locations
  $eimPaths = @(
    "C:\Espressif\tools\eim_idf.json",
    "$env:USERPROFILE\Espressif\tools\eim_idf.json",
    "$env:USERPROFILE\.espressif\tools\eim_idf.json",
    "$env:APPDATA\.espressif\tools\eim_idf.json"
  )
  foreach ($eimPath in $eimPaths) {
    if (Test-Path $eimPath) {
      try {
        $eim = Get-Content $eimPath -Raw | ConvertFrom-Json
        $idf = $null
        if ($eim.idfSelectedId) {
          $idf = $eim.idfInstalled | Where-Object { $_.id -eq $eim.idfSelectedId } | Select-Object -First 1
        }
        if (-not $idf) {
          $idf = $eim.idfInstalled | Select-Object -First 1
        }
        if ($idf -and $idf.activationScript -and (Test-Path $idf.activationScript)) {
          Write-Host "Found IDF activation script from '$eimPath': $($idf.activationScript)"
          return $idf.activationScript
        }
      } catch {
        # Ignore JSON parsing errors
      }
    }
  }

  # 4. Scan C:\Espressif\tools and other directories for Microsoft.*.PowerShell_profile.ps1
  $searchDirs = @(
    "C:\Espressif\tools",
    "$env:USERPROFILE\Espressif\tools",
    "$env:USERPROFILE\.espressif\tools",
    "$env:APPDATA\.espressif\tools"
  )
  foreach ($dir in $searchDirs) {
    if (Test-Path $dir) {
      $profileScripts = Get-ChildItem -Path $dir -Filter "*PowerShell_profile.ps1" -ErrorAction SilentlyContinue
      if ($profileScripts) {
        $candidate = ($profileScripts | Sort-Object Name -Descending | Select-Object -First 1).FullName
        Write-Host "Found IDF activation script in '$dir': $candidate"
        return $candidate
      }
    }
  }

  return $null
}

$activationScript = Get-IDFActivationScript
if ($activationScript) {
  . $activationScript
} else {
  if (-not (Get-Command "idf.py" -ErrorAction SilentlyContinue)) {
    Write-Warning "Could not find an ESP-IDF activation script, and idf.py is not in PATH."
  }
}
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
  "list" {
    Write-Host "Available serial ports:"
    Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match '\(COM\d+\)' } | ForEach-Object {
      if ($_.Name -match '\((COM\d+)\)') {
        Write-Host "  $($Matches[1]) - $($_.Caption)"
      }
    }
    python -m serial.tools.list_ports -v
  }
}
