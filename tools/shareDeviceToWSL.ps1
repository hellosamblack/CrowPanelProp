$vidpid = "1a86:7522"

$usbList = usbipd list

$deviceLine = $usbList | Where-Object { $_ -match $vidpid }

if (-not $deviceLine) {
    Write-Host "Device $vidpid not found."
    exit 1
}

# Extract BUSID (first column)
$busId = ($deviceLine -split "\s+")[0]

Write-Host "Detected BUSID: $busId"

# Get usbipd list output
$usbList = usbipd list

# Find the line for our device
$deviceLine = $usbList | Where-Object { $_ -match "^\s*$busId\s" }

if (-not $deviceLine) {
    Write-Host "Device with BUSID $busId not found."
    exit 1
}

Write-Host "Found device: $deviceLine"

# Determine current state
if ($deviceLine -match "Not shared") {
    Write-Host "Binding device..."
    usbipd bind --busid $busId
}
elseif ($deviceLine -match "Shared") {
    Write-Host "Device already shared."
}
elseif ($deviceLine -match "Attached") {
    Write-Host "Device already attached to WSL."
    exit 0
}

# Attach to WSL
Write-Host "Attaching device to WSL..."
usbipd attach --wsl --busid $busId

Write-Host "Done."