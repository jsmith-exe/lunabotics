# Install UV
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# Install Python 3.10 (for consistency)
uv python install 3.10

# Create virtual environment and activate
python3.10 -m venv venv
venv\Scripts\activate

# Install requirements
uv pip install -r .\requirements.txt
