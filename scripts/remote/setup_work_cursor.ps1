#Requires -Version 5.1
# Unit Windows: write SSH config and test connection to home Linux.
# Usage: powershell -ExecutionPolicy Bypass -File scripts\remote\setup_work_cursor.ps1

$ErrorActionPreference = "Stop"

$ScriptDir = $PSScriptRoot
$ConfigExample = Join-Path $ScriptDir "work_config.env.example"
$WorkConfig = Join-Path $ScriptDir "work_config.env"

if (-not (Test-Path $WorkConfig)) {
    Copy-Item $ConfigExample $WorkConfig
    Write-Host "Created work_config.env - verify TAILSCALE_HOST then re-run."
}

$config = @{}
Get-Content $WorkConfig | ForEach-Object {
    $line = $_.Trim()
    if ($line.StartsWith("#") -or $line.Length -eq 0) { return }
    $parts = $line.Split("=", 2)
    if ($parts.Count -eq 2) {
        $config[$parts[0].Trim()] = $parts[1].Trim()
    }
}

$HostAlias = if ($config["SSH_HOST_ALIAS"]) { $config["SSH_HOST_ALIAS"] } else { "dummy-home" }
$TailscaleHost = if ($config["TAILSCALE_HOST"]) { $config["TAILSCALE_HOST"] } else { "lxx01" }
$LinuxUser = if ($config["LINUX_USER"]) { $config["LINUX_USER"] } else { "lxx" }
$RemoteDir = if ($config["REMOTE_PROJECT_DIR"]) { $config["REMOTE_PROJECT_DIR"] } else { "/home/lxx/Projects/dummyResearch" }

$SshDir = Join-Path $env:USERPROFILE ".ssh"
$SshConfig = Join-Path $SshDir "config"
$IdentityFile = Join-Path $SshDir "id_ed25519"

if (-not (Test-Path $IdentityFile)) {
    Write-Host "Missing SSH key: $IdentityFile"
    Write-Host "Run: ssh-keygen -t ed25519 -C $env:USERNAME"
    exit 1
}

New-Item -ItemType Directory -Force -Path $SshDir | Out-Null

$block = @"
# --- dummyResearch remote (Cursor SSH) ---
Host $HostAlias
    HostName $TailscaleHost
    User $LinuxUser
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
    StrictHostKeyChecking accept-new
    ServerAliveInterval 30
    ServerAliveCountMax 6
    ConnectTimeout 30
# --- end dummyResearch ---

"@

$content = ""
if (Test-Path $SshConfig) {
    $content = Get-Content $SshConfig -Raw
    if ($content -match "dummyResearch remote") {
        $content = [regex]::Replace($content, '# --- dummyResearch remote[\s\S]*?# --- end dummyResearch ---\r?\n?', '')
    }
}

if ($content.Length -gt 0 -and -not $content.EndsWith([Environment]::NewLine)) {
    $content = $content + [Environment]::NewLine
}
$content = $content + $block
[System.IO.File]::WriteAllText($SshConfig, $content.TrimEnd() + [Environment]::NewLine, [System.Text.UTF8Encoding]::new($false))

Write-Host "OK: wrote $SshConfig"
Write-Host "     Host $HostAlias -> ${LinuxUser}@${TailscaleHost}"

$tailscale = Get-Command tailscale -ErrorAction SilentlyContinue
if (-not $tailscale) {
    Write-Host "WARN: Tailscale not found. Install from https://tailscale.com/download/windows"
} else {
    Write-Host "Tailscale status:"
    & tailscale status 2>&1 | Select-Object -First 10
}

Write-Host ""
Write-Host "Testing SSH (host key + public key auth)..."
$sshOpts = @(
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "PasswordAuthentication=no",
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=15"
)
try {
    $out = & ssh @sshOpts $HostAlias "hostname; test -d $RemoteDir && echo DIR_OK || echo DIR_MISSING" 2>&1
    $text = ($out | Out-String).Trim()
    Write-Host $text
    if ($LASTEXITCODE -eq 0) {
        Write-Host "OK: SSH connected"
    } elseif ($text -match "Permission denied") {
        Write-Host "FAIL: public key not authorized on home Linux yet"
        Write-Host "      At home run: bash scripts/remote/setup_home_ssh.sh"
    } elseif ($text -match "Host key verification failed") {
        Write-Host "FAIL: host key issue - re-run this script (config uses StrictHostKeyChecking accept-new)"
    } else {
        Write-Host "FAIL: SSH exit code $LASTEXITCODE"
    }
} catch {
    Write-Host "FAIL: $($_.Exception.Message)"
}

Write-Host ""
Write-Host "Cursor next steps:"
Write-Host "  1. Install extension: Remote - SSH"
Write-Host "  2. Ctrl+Shift+P -> Remote-SSH: Connect to Host -> $HostAlias"
Write-Host "  3. Open Folder -> $RemoteDir"
Write-Host "  See docs/CURSOR_REMOTE.md"
