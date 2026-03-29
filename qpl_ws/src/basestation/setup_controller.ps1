# Install UV
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# Install Python 3.10 (for consistency)
uv python install 3.10

# Delete any prior venv
if (Test-Path -Path "venv") {
    Remove-Item -Recurse -Force "venv"
}

# Create virtual environment and activate
python3.10 -m venv venv
venv\Scripts\activate

# Install requirements
pip install pynput==1.8.1
pip install pydualsense==0.7.5
