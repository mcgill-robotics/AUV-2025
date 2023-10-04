for /f "delims=[] tokens=2" %%a in ('ping -4 -n 1 %ComputerName% ^| findstr [') do set NetworkIP=%%a
docker-compose -f compose-windows.yml run -e DISPLAY=%NetworkIP%:0.0 clarke