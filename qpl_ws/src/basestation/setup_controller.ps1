# Install UV
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
$env:Path = "$env:USERPROFILE\.local\bin;$env:Path"

# Install Python 3.10 (for consistency)
uv python install 3.10

# Delete any prior venv
if (Test-Path -Path "venv") {
    Remove-Item -Recurse -Force "venv"
}

# Create virtual environment, activate, and ensure pip is installed
python3.10 -m venv venv
.\venv\Scripts\Activate.ps1
.\venv\Scripts\python.exe -m ensurepip --upgrade

# Install requirements
uv pip install pynput==1.8.1
uv pip install pydualsense==0.7.5

Rename-Item -Path ".\venv\Lib" -NewName "libtemp"
Rename-Item -Path ".\venv\libtemp" -NewName "lib"
