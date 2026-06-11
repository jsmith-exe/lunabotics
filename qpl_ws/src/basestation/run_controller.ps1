$controller_fwd = Start-Process wsl -ArgumentList 'bash -i -c "qpl_controller_fwd; read -p \"Press Enter to close...\""' -PassThru
.\venv\Scripts\Activate.ps1
$controller = Start-Process python -ArgumentList '-m basestation.main' -NoNewWindow -PassThru

Write-Host "Press Enter to stop..."
Read-Host

# Kill by process object
$controller_fwd.Kill()
$controller.Kill()
