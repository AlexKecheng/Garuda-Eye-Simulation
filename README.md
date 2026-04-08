# Garuda Eye - Air Defense Simulation

Sistem Simulasi Taktik Integrasi Arhanud berbasis Python dan Three.js untuk mendukung pengambilan keputusan melalui sistem K4IPP.

## Fitur Utama
- **Sensor Fusion**: Integrasi data Radar dan Kamera.
- **Fuzzy-SAW Decision Support**: Prioritas ancaman menggunakan logika fuzzy.
- **3D Tactical Visualization**: Visualisasi real-time menggunakan Three.js.
- **AAR (After Action Review)**: Laporan efektivitas operasi.

## Cara Menjalankan

### 1. Backend (Python)
Pastikan telah menginstal dependency:
```bash
pip install flask flask-cors scikit-fuzzy
```
Jalankan server:
```bash
python main.py
```

### 2. Frontend
Buka file `index.html` di browser Anda.

## Teknologi
- Python (Flask, Scikit-Fuzzy)
- JavaScript (Three.js, Leaflet.js)