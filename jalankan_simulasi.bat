@echo off
title Garuda Eye Control Center
color 0A

echo ====================================================
echo   MENGAKTIFKAN SISTEM PERTAHANAN GARUDA EYE
echo ====================================================

echo [1/3] Memeriksa dependensi Python...
python -m pip install -r requirements.txt

echo [2/3] Menjalankan Backend Flask di jendela baru...
start "Garuda Eye Backend" cmd /k "python main.py"

echo Menunggu server inisialisasi (3 detik)...
timeout /t 3 /nobreak > nul

echo [3/3] Membuka Dashboard Tactical...
start index.html

echo SISTEM AKTIF. Silakan periksa browser Anda.