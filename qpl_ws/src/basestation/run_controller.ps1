$controller_fwd = Start-Process wsl -ArgumentList 'bash -i -c "qpl_controller_fwd"' -PassThru
.\venv\Scripts\activate
$controller = Start-Process python -ArgumentList '.\basestation\main.py' -NoNewWindow -PassThru

Write-Host "Press Enter to stop..."
Read-Host

# Kill by process object
$controller_fwd.Kill()
$controller.Kill()
