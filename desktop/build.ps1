param([string]$FirmwarePackage = (Join-Path $PSScriptRoot '..\firmware-package'))
$ErrorActionPreference = 'Stop'
$python = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
$release = Join-Path $PSScriptRoot 'release\Tiller'
$workerBuild = Join-Path $PSScriptRoot 'release\worker-build'
New-Item -ItemType Directory -Force $release | Out-Null
& $python -m PyInstaller --noconfirm --onedir --name tiller-worker --distpath $workerBuild --workpath (Join-Path $PSScriptRoot 'build\worker') --specpath (Join-Path $PSScriptRoot 'build') --collect-all esptool --collect-all esp_pylib --recursive-copy-metadata esptool (Join-Path $PSScriptRoot 'worker\worker.py')
if ($LASTEXITCODE -ne 0) { throw 'Worker build failed.' }
& dotnet publish (Join-Path $PSScriptRoot 'Tiller.App') -c Release -r win-x64 --self-contained true -p:PublishSingleFile=true -p:IncludeNativeLibrariesForSelfExtract=true -o $release --nologo
if ($LASTEXITCODE -ne 0) { throw 'Desktop build failed.' }
foreach ($directory in @('worker','package','source','licenses')) { New-Item -ItemType Directory -Force (Join-Path $release $directory) | Out-Null }
Get-ChildItem -LiteralPath (Join-Path $workerBuild 'tiller-worker') | Copy-Item -Destination (Join-Path $release 'worker') -Force -Recurse
Copy-Item -LiteralPath (Join-Path $FirmwarePackage 'manifest.json') -Destination (Join-Path $release 'package') -Force
Copy-Item -LiteralPath (Join-Path $FirmwarePackage 'firmware') -Destination (Join-Path $release 'package') -Force -Recurse
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'README.md') -Destination (Join-Path $release 'START-HERE.md') -Force
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'COPYING-worker.txt') -Destination (Join-Path $release 'licenses') -Force
Copy-Item -LiteralPath (Join-Path $PSScriptRoot '..\LICENSE') -Destination (Join-Path $release 'licenses\LICENSE.txt') -Force
Copy-Item -LiteralPath (Join-Path $PSScriptRoot '..\THIRD-PARTY.md') -Destination (Join-Path $release 'licenses') -Force
# Ship source without caches, compiled artifacts, personal logs, backups or stock dumps.
foreach ($directory in @('Tiller.App','worker','tests')) {
    $target = Join-Path (Join-Path $release 'source') $directory
    New-Item -ItemType Directory -Force $target | Out-Null
    Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot $directory) -File |
        Where-Object { $_.Extension -in @('.cs','.csproj','.py','.txt','.manifest') } |
        Copy-Item -Destination $target -Force
}
foreach ($name in @('build.ps1','README.md','COPYING-worker.txt')) { Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $release 'source') -Force }
Write-Output "Built: $release"
