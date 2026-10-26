param()

$ErrorActionPreference = "Stop"
$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$python = Join-Path $root '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) { throw 'Run .\scripts\setup.ps1 first.' }
$env:MENAGERIE_CACHE_DIR = Join-Path $root 'data\mujoco_menagerie_cache'
& $python (Join-Path $root 'app.py')
if (-not $?) { throw 'mujoco-cn did not start successfully.' }
if ($LASTEXITCODE -ne 0) { throw "mujoco-cn exited with code $LASTEXITCODE." }
