param()

$ErrorActionPreference = "Stop"
$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$venvPython = Join-Path $root '.venv\Scripts\python.exe'
$requirements = Join-Path $root 'requirements.txt'
$cache = Join-Path $root 'data\mujoco_menagerie_cache'

if (-not (Test-Path -LiteralPath $venvPython)) {
    $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if (-not $pythonCommand) { throw 'Python 3.11 or newer is required.' }
    & $pythonCommand.Source -m venv (Join-Path $root '.venv')
    if (-not $?) { throw 'Could not create the project virtual environment.' }
    if ($LASTEXITCODE -ne 0) { throw "python -m venv exited with code $LASTEXITCODE." }
}

New-Item -ItemType Directory -Path $cache -Force | Out-Null
& $venvPython -m pip install -r $requirements
if (-not $?) { throw 'Could not install project dependencies.' }
if ($LASTEXITCODE -ne 0) { throw "pip install exited with code $LASTEXITCODE." }

$env:MENAGERIE_CACHE_DIR = $cache
& $venvPython -c "import mujoco_menagerie as mm; mm.prefetch(['robotstudio_so101'])"
if (-not $?) { throw 'Could not download the SO-101 demonstration model.' }
if ($LASTEXITCODE -ne 0) { throw "model prefetch exited with code $LASTEXITCODE." }

Write-Host "Setup complete. Run .\scripts\run.ps1"
