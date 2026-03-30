/*
================================================================================
JUDUL TUGAS AKHIR:
Rancang Bangun Sistem Simulasi Taktik Integrasi Arhanud Berbasis Python pada 
Taktik Integrasi Satuan Arhanud dalam Opsratgab/Opshanudnas Guna Mendukung 
Pengambilan Keputusan Dandahanud Melalui Sistem K4IPP
================================================================================
I. PEMISAHAN KOMPONEN

A. Komponen Hardware (Prototipe yang Disimulasikan):
   - Radar Surveillance: 3 unit radar pencari dengan jangkauan 20km, disebar dalam formasi segitiga.
   - Missile Launcher: 3 unit peluncur rudal, terintegrasi dengan masing-masing radar.
   - Objek Vital (Obvit): 1 unit gedung pusat (IKN) yang menjadi titik pusat pertahanan.
   - Rintangan Medan: 1 unit gunung sebagai penghalang fisik Line-of-Sight.

B. Fitur Software (Logika & Algoritma yang Diimplementasikan):
   - Sensor Fusion: Korelasi data dari berbagai sensor (simulasi) untuk membentuk satu gambar situasi.
   - Target Prioritization: Algoritma skoring dinamis berdasarkan Tipe, Jarak ke Obvit, dan Kecepatan Mendekat (Closing Speed).
   - Probabilty of Kill (Pk): Peluang hancur target yang bervariasi berdasarkan jarak dan spesifikasi senjata.
   - Ammunition Management: Sistem pembatasan amunisi dan durasi reload otomatis.
   - Overlapping Coverage: Simulasi deteksi target oleh lebih dari satu radar.
   - Blind Spot Simulation: Keterbatasan deteksi akibat elevasi rendah (horizon) dan rintangan medan (gunung).
   - Swarm Attack Mode: Skenario serangan saturasi dengan multi-target (drone).
   - Dynamic Weapon Zones (IZ/LRZ): Visualisasi jangkauan efektif senjata secara real-time.
   - System Failure Simulation: Fitur untuk menonaktifkan radar secara manual.

II. MATRIKS PERBANDINGAN EFEKTIVITAS (KERANGKA ANALISIS KONSEPTUAL)

Tujuan: Untuk mengevaluasi performa berbagai konfigurasi pertahanan terhadap skenario ancaman yang berbeda.

| Skenario Gelar Tikar        | Metrik Evaluasi                  | Skenario Ancaman: Swarm Drone | Skenario Ancaman: Pop-up Helicopter | Skenario Ancaman: High-speed Missile |
|-----------------------------|----------------------------------|-------------------------------|-------------------------------------|--------------------------------------|
| 1. Gelar Lingkar Rapat      | - Waktu Reaksi (detik)           | Cepat                         | Sedang                              | Lambat (deteksi telat)               |
|    (Radius Pertahanan 5km)  | - Tingkat Kebocoran (Leakage %)  | Rendah                        | Rendah                              | Tinggi                               |
|                             | - Efisiensi Amunisi (Kill/Shot)  | Sedang (overkill mungkin)     | Tinggi                              | Rendah (banyak meleset)              |
|-----------------------------|----------------------------------|-------------------------------|-------------------------------------|--------------------------------------|
| 2. Gelar Lingkar Lebar      | - Waktu Reaksi (detik)           | Lambat (deteksi lebih awal)   | Cepat                               | Sedang                               |
|    (Radius Pertahanan 10km) | - Tingkat Kebocoran (Leakage %)  | Sedang (ada celah)            | Rendah                              | Sedang                               |
|                             | - Efisiensi Amunisi (Kill/Shot)  | Tinggi                        | Tinggi                              | Sedang                               |
|-----------------------------|----------------------------------|-------------------------------|-------------------------------------|--------------------------------------|
| 3. Gelar Berlapis (Layered) | - Waktu Reaksi (detik)           | Sangat Cepat                  | Sangat Cepat                        | Sangat Cepat                         |
|    (Kombinasi Jarak Jauh/Dekat)| - Tingkat Kebocoran (Leakage %)  | Sangat Rendah                 | Sangat Rendah                       | Rendah                               |
|                             | - Efisiensi Amunisi (Kill/Shot)  | Sangat Tinggi                 | Sangat Tinggi                       | Tinggi                               |
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

// --- KONFIGURASI SIMULASI (Sama dengan Python) ---
const OBJECT_TYPES = ["PTTA", "rotary-wing", "fixed-wing", "SSM", "AGM", "RAM", "EWC"];
const THREAT_SCORES = {
    "SSM": 100, "fixed-wing": 80, "rotary-wing": 60, "PTTA": 40, "RAM": 70, "EWC": 90, "unknown": 10
};
// Jarak ditingkatkan agar lebih realistis untuk skala tempur udara
const SCENE_SCALE = 1 / 100; // Skala dunia 3D (1m di dunia nyata = 0.01 unit di 3D)
const RADAR_EW_RANGE = 150000; // 150 km (Early Warning AU)
const RADAR_KRI_RANGE = 80000;  // 80 km (Radar KRI AL)
const RADAR_LOCAL_RANGE = 25000; // 25 km (Radar Dalpur/Dalbak Arhanud)
const RADAR_RANGE = 40000; // DEFINISI KRITIS: Jangan dihapus atau dipindah
const RADAR_RANGE = 40000; // Jangkauan Radar Standar
const RADAR_LOCAL_RANGE = 25000;

// --- POSISI ASET EKSTERNAL (Integrasi K4IPP) ---
const KRI_AL_POS = { x: -60000, z: -40000 };
const EW_RADAR_POS = { x: 50000, z: -70000 };

const VISUAL_RANGE = 15000; // 15 km (Untuk fusi data, tidak mempengaruhi deteksi)
let DEFENSE_RADIUS = 5000; // 5 km dari pusat untuk penempatan radar/rudal (diubah ke let)
const CRITICAL_RANGE = 5000; // 5 km (Game Over)
const SIM_TICK = 1000; // Update setiap 1 detik

// --- ASPEK TEKNIS ALUTSISTA ---
const MIN_ELEVATION_ANGLE = 1.5; // Blind spot radar di bawah 1.5 derajat (Clutter/Horizon)
const MOUNTAIN_CONFIG = { x: 25000, z: 10000, radius: 6000, height: 5000 }; // Rintangan Medan
const WEAPON_SPECS = {
    "NASAMS": { maxRange: 40000, pk: 80, maxAmmo: 6, reloadTime: 10000, cost: 2000 },  // Rp 2 Miliar
    "VL MICA": { maxRange: 20000, pk: 85, maxAmmo: 8, reloadTime: 5000, cost: 1500 },   // Rp 1.5 Miliar
    "Starstreak": { maxRange: 7000, pk: 95, maxAmmo: 12, reloadTime: 3000, cost: 500 }, // Rp 500 Juta
    "Oerlikon": { maxRange: 4000, pk: 70, maxAmmo: 100, reloadTime: 2000, cost: 5 }     // Rp 5 Juta
};

/**
 * MODUL 5: MONTE CARLO SIMULATOR
 * Menambahkan variabel 'Human Error' (Waktu Reaksi Operator)
 */
const AssetArhanud = { weapons: WEAPON_SPECS };
const MonteCarloEngine = {
    run(scenario, iterations = 1000) {
        let wins = 0;
        let totalKills = 0;
        for (let i = 0; i < iterations; i++) {
            const weatherVar = Object.keys(WEATHER_MODIFIERS)[Math.floor(Math.random() * 3)];
            const hasFailure = Math.random() > 0.98; // 2% kegagalan sistem acak
            const simResult = this.simulateBatch(scenario, weatherVar, hasFailure);
            if (simResult.success) wins++;
            totalKills += simResult.kills;
        }
        this.displayResults((wins / iterations) * 100, totalKills / iterations);
    },
    simulateBatch(scenario, weather, hasFailure) {
        let kills = 0;
        let success = true;

        // --- VARIABEL HUMAN ERROR ---
        // Mensimulasikan waktu reaksi operator (dalam detik)
        // Rata-rata reaksi manusia dalam kondisi stres adalah 1.5 - 5 detik.
        const reactionTime = 1.5 + (Math.random() * 3.5);

        scenario.targets.forEach(t => {
            // Hitung TTI (Time To Impact) secara akurat berdasarkan posisi kartesius
            const dist = Math.sqrt(t.x ** 2 + t.z ** 2);
            const dot = t.x * t.vx + t.y * t.vy + t.z * t.vz;
            const closingSpeed = dist > 0 ? -(dot / dist) : t.speed;
            const tti = dist / (closingSpeed || 1);

            // LOGIKA TAKTIS: Jika waktu benturan lebih cepat dari reaksi operator, 
            // pertahanan dianggap gagal merespon tepat waktu (Target Leak).
            if (tti < reactionTime) {
                success = false;
                return;
            }

            const pkMod = WEATHER_MODIFIERS[weather] || 1.0;
            const finalPk = (scenario.weapon.pk / 100) * pkMod * (hasFailure ? 0.6 : 1.0);
            if (Math.random() < finalPk) kills++;
            else if (tti < 10) success = false; // Kebocoran jika meleset di jarak kritis
        });
        return { success, kills };
    },
    displayResults(winRate, avgKills) {
        const logDiv = document.getElementById('combatLog');
        const msg = `MONTE CARLO (1000x): Success ${winRate.toFixed(1)}%, Avg Kills ${avgKills.toFixed(1)}. ${winRate > 80 ? 'TAKTIK VALID' : 'RESIKO TINGGI'}`;
        if (logDiv) logDiv.innerHTML = `<div style="color:#bb86fc; border:1px solid #333; padding:5px; margin-bottom:5px;">${msg}</div>` + logDiv.innerHTML;
        alert(msg);
    }
};

// --- STATE SIMULASI ---
let systemTracks = []; // Apa yang dilihat oleh sistem (hasil fusi)
let realTargets = [];  // Ground Truth (Posisi asli objek di dunia nyata)
const mapCenter = [-7.25, 110.4]; // Konseptual, tidak lagi dipakai untuk render
let currentAmmo = { "NASAMS": 6, "VL MICA": 8, "Starstreak": 12, "Oerlikon": 100 };
let reloadingStatus = { "NASAMS": false, "VL MICA": false, "Starstreak": false, "Oerlikon": false };
let radarActive = [true, true, true, true, true, true]; // 6 Unit Arhanud
let selectedTargetId = null; // ID target yang dipilih manual
let missionStats = { kills: 0, shots: 0, costSaved: 0, leaked: 0 };
let engagementHistory = []; // Untuk menyimpan data debat/AAR
let radarRings = []; // Referensi ke mesh cincin radar untuk toggle visibilitas
let weaponZoneMeshes = []; // Referensi ke mesh IZ/LRZ
let defenseSiteMeshes = []; // Menyimpan semua objek 3D dari site pertahanan
let isGameOver = false;
let simIntervalId;

// --- KONFIGURASI MCDM (Dynamic Threat Assessment) ---
const WEIGHTS = {
    TTI: 0.45,       // Urgensi (Waktu menuju benturan)
    STRATEGIC: 0.30, // Nilai Strategis (Tipe sasaran)
    DIRECTNESS: 0.10,// Arah serangan (Radial vs Tangensial)
    CONFIDENCE: 0.10, // Faktor Keyakinan Data
    PK_AVAIL: 0.05    // Ketersediaan Senjata
};
const MAX_SPEED_REF = 1000; // m/s
const MAX_TTI_REF = 300;    // 5 Menit (Window penilaian urgensi)
const MAX_RCS_REF = 10;
const RCS_MAP = { "missile": 10, "airplane": 5, "helicopter": 3, "drone": 1, "unknown": 2 };
let currentWeather = "Clear";
const WEATHER_MODIFIERS = { "Clear": 1.0, "Rain": 0.75, "Storm": 0.6 };

// --- INTEGRASI SERVER ---
const USE_PYTHON_SERVER = false; // Set ke true untuk mengambil data dari Python
const SERVER_URL = "http://localhost:5000/api/data"; // Sesuaikan dengan endpoint server Anda

// --- 3D SCENE STATE ---
let scene, camera, renderer, controls;
const clock = new THREE.Clock();
let targetMeshes = {}; // Menyimpan objek 3D untuk setiap target
let activeExplosions = []; // Untuk melacak partikel ledakan
let radarMeshes = []; // Untuk animasi putaran radar
let activeSmokeParticles = []; // Untuk jejak asap rudal
let launcherPositions = []; // Menyimpan posisi 3 launcher pertahanan
let activeInterceptors = []; // Daftar rudal pertahanan yang sedang terbang
let loader; // Inisialisasi nanti di startSimulation
const loadedModels = {}; // Cache untuk model yang sudah di-load
const modelPaths = {
    "fixed-wing": "models/airplane.glb",
    "rotary-wing": "models/helicopter.glb",
    "SSM": "models/missile.glb",
    "AGM": "models/missile.glb",
    "RAM": "models/missile.glb",
    "PTTA": "models/drone.glb",
    "EWC": "models/airplane.glb",
    "unknown": "models/unknown.glb"
};

// --- LOGIKA SIMULASI ---

function getRandomInt(min, max) {
    return Math.floor(Math.random() * (max - min + 1)) + min;
}

// Fungsi Geometri (Dipindahkan ke atas untuk akses global)
// Fungsi ini tidak lagi relevan untuk render, tapi masih berguna untuk logika AI
function getBearingAndDistance(lat1, lon1, lat2, lon2) {
    // NOTE: Fungsi ini sekarang hanya untuk kalkulasi non-visual (AI, sensor)
    // Kita akan menggunakan posisi kartesius untuk rendering
    const R = 6371e3;
    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;
    const Δφ = (lat2 - lat1) * Math.PI / 180;
    const Δλ = (lon2 - lon1) * Math.PI / 180;

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
        Math.cos(φ1) * Math.cos(φ2) *
        Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    const d = R * c;

    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) -
        Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
    const θ = Math.atan2(y, x);
    const brng = (θ * 180 / Math.PI + 360) % 360;

    return { distance: d, angle: brng };
}

/**
 * MODUL 6: TACTICAL MAP ENGINE (2D)
 * Implementasi Esri World Imagery & Layer Control (Battle-Ready)
 */
let tacticalMap = null;
let is2DMode = false;

function initTacticalMap() {
    // Fail-safe: Cek apakah library Leaflet (L) tersedia
    if (typeof L === 'undefined') {
        console.error("Tactical Map: Library Leaflet tidak ditemukan. Mode 2D dinonaktifkan.");
        return;
    }

    // 1. Definisikan Peta Standar (OSM)
    const petaJalan = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
    });

    // 2. Definisikan Esri Satelit (Profesional)
    const petaSatelit = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
        attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EBP, and the GIS User Community',
        maxZoom: 18
    });

    // 3. Inisialisasi Peta (Center di Titik Nol IKN Nusantara)
    tacticalMap = L.map('map', {
        center: [-0.8492, 116.7025],
        zoom: 12,
        layers: [petaJalan] // Set default tampilan awal ke OpenStreetMap (Peta Jalan)
    });

    // 4. Masukkan ke dalam Kontrol Layer agar bisa berganti tampilan
    const baseMaps = {
        "Peta Satelit (Profesional)": petaSatelit,
        "Peta Jalan (Standar)": petaJalan
    };
    L.control.layers(baseMaps).addTo(tacticalMap);
}

/**
 * Algoritma 3D Line of Sight (LoS)
 * Memeriksa apakah garis antara observer dan target terhalang oleh rintangan medan (Gunung).
 */
function checkLineOfSight(obsPos, targetPos) {
    const A = targetPos.x - obsPos.x;
    const B = targetPos.z - obsPos.z;
    const C = MOUNTAIN_CONFIG.x - obsPos.x;
    const D = MOUNTAIN_CONFIG.z - obsPos.z;

    const dot = A * C + B * D;
    const len_sq = A * A + B * B;
    let param = -1;
    if (len_sq !== 0) param = dot / len_sq;

    let xx, zz;
    if (param < 0) { xx = obsPos.x; zz = obsPos.z; }
    else if (param > 1) { xx = targetPos.x; zz = targetPos.z; }
    else { xx = obsPos.x + param * A; zz = obsPos.z + param * B; }

    const dx = MOUNTAIN_CONFIG.x - xx;
    const dz = MOUNTAIN_CONFIG.z - zz;
    const distToMountainCenter = Math.sqrt(dx * dx + dz * dz);

    if (distToMountainCenter < MOUNTAIN_CONFIG.radius) {
        const distObsToMountain = Math.sqrt(C * C + D * D);
        const distObsToTarget = Math.sqrt(A * A + B * B);
        if (distObsToTarget > distObsToMountain) {
            const ratio = distObsToMountain / distObsToTarget;
            const losHeightAtMountain = obsPos.y + (targetPos.y - obsPos.y) * ratio;
            if (losHeightAtMountain < MOUNTAIN_CONFIG.height) return false;
        }
    }
    return true;
}

// --- GROUND TRUTH (DUNIA NYATA) ---
function initRealWorld() {
    // Spawn awal 3-5 target
    for (let i = 0; i < getRandomInt(3, 5); i++) spawnTarget();
}

// Update spawnTarget untuk menerima parameter konfigurasi (mendukung Swarm)
function spawnTarget(config = {}) {
    const angle = config.angle !== undefined ? config.angle : Math.random() * 360;
    const angleRad = angle * (Math.PI / 180);
    // Spawn jauh di luar radar (Minimal 50km dari batas luar radar)
    const dist = config.dist !== undefined ? config.dist : getRandomInt(RADAR_RANGE + 50000, RADAR_RANGE + 80000);
    const altitude = config.altitude !== undefined ? config.altitude : getRandomInt(1000, 8000);

    realTargets.push({
        id: Math.random().toString(36).substr(2, 9),
        type: OBJECT_TYPES[getRandomInt(0, OBJECT_TYPES.length - 1)],
        // Posisi Kartesius 3D
        x: Math.cos(angleRad) * dist,
        y: altitude, // Ketinggian
        z: Math.sin(angleRad) * dist, // Sumbu Z untuk kedalaman
        speed: getRandomInt(100, 400), // m/s (sekitar 200-800 knots)
        // Heading tidak lagi 2D, tapi vektor kecepatan
        vx: 0, vy: 0, vz: 0
    });

    // Buat mesh 3D untuk target baru
    createTargetMesh(realTargets[realTargets.length - 1]);
}

// --- FUNGSI FETCH DATA SERVER ---
async function fetchServerData() {
    try {
        const response = await fetch(SERVER_URL);
        if (!response.ok) throw new Error("Gagal koneksi ke server");
        const data = await response.json();

        // 1. Update Ground Truth (Objek 3D Visual)
        // Server diharapkan mengirim: { "ground_truth": [ {id, type, x, y, z, ...}, ... ] }
        if (data.ground_truth) {
            // Hapus target lokal yang tidak ada di data server (target hancur/hilang)
            const serverIds = new Set(data.ground_truth.map(t => t.id));
            realTargets = realTargets.filter(t => {
                if (!serverIds.has(t.id)) {
                    if (targetMeshes[t.id]) {
                        scene.remove(targetMeshes[t.id]);
                        delete targetMeshes[t.id];
                    }
                    return false;
                }
                return true;
            });

            // Update posisi atau tambah target baru
            data.ground_truth.forEach(serverTarget => {
                const existing = realTargets.find(t => t.id === serverTarget.id);
                if (existing) {
                    // Update properti posisi dari server
                    Object.assign(existing, serverTarget);
                } else {
                    // Target baru dari server
                    realTargets.push(serverTarget);
                    createTargetMesh(serverTarget);
                }
            });
        }

        // 2. Update System Tracks (Data Tabel & Prioritas)
        // Server diharapkan mengirim: { "tracks": [ {id, classification, score, ...}, ... ] }
        if (data.tracks) {
            systemTracks = data.tracks;
        }
    } catch (error) {
        console.warn("Server Error:", error);
        document.getElementById('recText').innerHTML = `<span style="color:#cf6679;">KONEKSI SERVER TERPUTUS</span>`;
    }
}

function spawnSwarm() {
    const centerAngle = Math.random() * 360;
    const swarmSize = 8;
    for (let i = 0; i < swarmSize; i++) {
        spawnTarget({
            type: 'drone',
            angle: centerAngle + (Math.random() - 0.5) * 20, // Spread 20 derajat
            dist: RADAR_RANGE + 40000 + (Math.random() * 5000), // Jarak bervariasi sedikit
            altitude: 2000 + (Math.random() * 1000)
        });
    }
}

function updatePhysics(dtSeconds) {
    if (isGameOver) return;

    // Gerakkan target
    realTargets.forEach(t => {
        // AI Homing 3D: Arahkan ke titik (0, 0, 0)
        const distanceToBase = Math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z);

        // Normalisasi vektor arah
        const dirX = -t.x / distanceToBase;
        const dirY = -t.y / distanceToBase;
        const dirZ = -t.z / distanceToBase;

        // Update kecepatan
        t.vx = dirX * t.speed;
        t.vy = dirY * t.speed;
        t.vz = dirZ * t.speed;

        t.x += t.vx * dtSeconds;
        t.y += t.vy * dtSeconds;
        t.z += t.vz * dtSeconds;

        // Cek Game Over
        if (distanceToBase < CRITICAL_RANGE) {
            triggerGameOver(t);
        }
    });

    // Spawn musuh baru jika sepi
    if (realTargets.length < 2 && Math.random() > 0.9) spawnTarget();
}

function triggerGameOver(target) {
    isGameOver = true;
    missionStats.leaked++;
    clearInterval(simIntervalId);

    // Catat Kerugian Finansial Akibat Kebocoran ke History AAR
    engagementHistory.push({
        time: new Date().toLocaleTimeString(),
        type: `LEAKAGE: ${target.type.toUpperCase()}`,
        distance: 0,
        closingSpeed: 0,
        totalScore: 0,
        justification: "KERUSAKAN OBVIT (IKN) - PERTAHANAN JEBOL",
        targetId: target.id,
        status: 'CRITICAL',
        isOverkill: false,
        weaponCost: 50000, // Estimasi kerugian aset Rp 50 Miliar (Auditable)
        result: 'LEAKED',
        pk: '0%'
    });

    const logDiv = document.getElementById('combatLog');
    logDiv.innerHTML = `<div style="background: #cf6679; color: #000; padding: 10px; font-weight: bold; text-align: center;">
                ⚠️ MARKAS HANCUR! ⚠️<br>
                ${target.type.toUpperCase()} berhasil menembus pertahanan.
            </div>` + logDiv.innerHTML;

    // Disable controls
    document.getElementById('fireButton').disabled = true;
    document.getElementById('weaponSelect').disabled = true;

    // Tampilkan Modal Game Over
    const modal = document.getElementById('gameOverModal');
    if (modal) {
        const reason = document.getElementById('gameOverReason');
        reason.innerHTML = `SISTEM PERTAHANAN JEBOL.<br>Markas dihancurkan oleh unit <b>${target.type.toUpperCase()}</b> pada jarak dekat.`;
        modal.style.display = 'flex';
    }
}

// --- SENSOR SIMULATION ---
function generateSensorData() {
    const radar = [];
    const camera = [];

    realTargets.forEach(t => {
        // Kalkulasi sensor berdasarkan posisi kartesius
        const distance = Math.sqrt(t.x * t.x + t.z * t.z); // Jarak di bidang XZ
        const angle = (Math.atan2(t.z, t.x) * 180 / Math.PI + 360) % 360;
        const altitude = t.y;

        // Cek deteksi dari 3 Radar (Gelar Lingkar)
        let detectedByRadar = false;
        for (let i = 0; i < 3; i++) {
            // Cek apakah radar ini aktif (Simulasi Kegagalan Sistem)
            if (!radarActive[i]) continue;

            const rAngle = (i * 120) * (Math.PI / 180);
            const rx = Math.cos(rAngle) * (DEFENSE_RADIUS * SCENE_SCALE); // Posisi dalam unit scene? 
            // Koreksi: DEFENSE_RADIUS di generateSensorData menggunakan satuan meter (real world logic)
            const rx_m = Math.cos(rAngle) * DEFENSE_RADIUS;
            const rz_m = Math.sin(rAngle) * DEFENSE_RADIUS;
            const distToRadar = Math.sqrt((t.x - rx_m) ** 2 + (t.z - rz_m) ** 2);
            if (distToRadar > RADAR_RANGE) continue;
            const elevationAngle = Math.atan2(t.y, distToRadar) * (180 / Math.PI);
            if (elevationAngle < MIN_ELEVATION_ANGLE) continue;
            if (!checkLineOfSight({ x: rx_m, y: 5, z: rz_m }, t)) continue;
            detectedByRadar = true;
        }

        // RADAR: Ada noise dan limit jarak
        if (detectedByRadar) {
            // Noise: Jarak +/- 100m, Sudut +/- 2 derajat
            const noisyDist = distance + (Math.random() - 0.5) * 200;
            const noisyAngle = (angle + (Math.random() - 0.5) * 4 + 360) % 360;

            // Hitung Closing Speed (Kecepatan Mendekat)
            // Dot product vektor kecepatan dengan vektor posisi
            // Jika hasilnya negatif, berarti mendekat. Kita negasikan agar positif = bahaya.
            let closingSpeed = 0;
            if (distance > 0) {
                const dot = t.x * t.vx + t.y * t.vy + t.z * t.vz;
                closingSpeed = -(dot / distance);
            }

            radar.push({
                distance: noisyDist,
                angle: noisyAngle, // Azimut
                altitude: t.y,     // Ketinggian
                speed: t.speed,
                closingSpeed: closingSpeed, // Data baru: Kecepatan radial
                trueId: t.id,
                source: "RADAR_PRIMARY",
                mode: "ACTIVE_SCAN"
            });
        }

        // KAMERA: Hanya visual range
        if (distance < VISUAL_RANGE) {
            // Noise sudut kamera lebih kecil (+/- 1 derajat)
            const noisyAngle = (angle + (Math.random() - 0.5) * 2 + 360) % 360;
            camera.push({
                label: t.type,
                angle: noisyAngle
            });
        }
    });

    // Clutter / Hantu Radar (False Positive) - Jarang terjadi
    if (Math.random() > 0.95) {
        radar.push({
            distance: getRandomInt(5000, 20000),
            angle: getRandomInt(0, 360),
            trueId: "GHOST"
        });
    }

    return { camera, radar };
}

function processData(camera, radar) {
    // 1. Fusi Data: Mencocokkan Radar & Kamera berdasarkan SUDUT (Bearing)
    let fused = [];
    let cameraUsed = new Set();

    radar.forEach((r, rIdx) => {
        let bestCamIdx = -1;
        let minDiff = 10;

        camera.forEach((c, cIdx) => {
            if (cameraUsed.has(cIdx)) return;
            let diff = Math.abs(r.angle - c.angle);
            if (diff > 180) diff = 360 - diff;

            if (diff < minDiff) {
                minDiff = diff;
                bestCamIdx = cIdx;
            }
        });

        if (bestCamIdx !== -1) {
            cameraUsed.add(bestCamIdx);
            fused.push({
                id: r.trueId,
                classification: camera[bestCamIdx].label,
                distance: r.distance,
                angle: r.angle,
                speed: r.speed,
                closingSpeed: r.closingSpeed,
                source: "FUSED"
            });
        } else {
            fused.push({
                id: r.trueId,
                classification: "unknown",
                distance: r.distance,
                angle: r.angle,
                speed: r.speed,
                closingSpeed: r.closingSpeed,
                source: "RADAR"
            });
        }
    });

    // 2. Algoritma Optimasi Penentuan Prioritas (Objective Function)
    // Menggunakan Linear Weighted Sum yang merupakan basis dari ILP dalam penentuan bobot target
    fused.forEach(obj => {
        const realTarget = realTargets.find(rt => rt.id === obj.id);
        // 1. Time to Impact (TTI) - Semakin kecil TTI, semakin tinggi skornya
        let s_tti = 0;
        if (obj.closingSpeed > 0) {
            const tti = obj.distance / obj.closingSpeed;
            s_tti = Math.max(0, 1 - (tti / MAX_TTI_REF));
        }

        // 2. Strategic Threat Value (Berdasarkan Tipe)
        const s_strategic = (RCS_MAP[obj.classification] || 4) / MAX_RCS_REF;

        // 3. Directness (Seberapa tegak lurus sasaran menuju IKN)
        const s_direct = obj.speed > 0 ? Math.max(0, (obj.closingSpeed || 0) / obj.speed) : 0;

        // 4. Engageability (Pk & Ammo Check)
        let s_pkAvail = 0;
        Object.values(WEAPON_SPECS).forEach(spec => {
            if (obj.distance <= spec.maxRange) s_pkAvail = Math.max(s_pkAvail, spec.pk / 100);
        });

        const s_confidence = obj.source === "FUSED" ? 1.0 : 0.6;

        const total = (s_tti * WEIGHTS.TTI) +
            (s_strategic * WEIGHTS.STRATEGIC) +
            (s_direct * WEIGHTS.DIRECTNESS) +
            (s_pkAvail * WEIGHTS.PK_AVAIL) +
            (s_confidence * WEIGHTS.CONFIDENCE);

        obj.score = Math.round(total * 100);
        if (obj.distance < CRITICAL_RANGE) obj.score += 100;

        // 3. Optimization Solver untuk Alokasi Senjata (Weapon-Target Assignment)
        // Mencari Max(Pk / Cost) berdasarkan kendala Range dan Amunisi
        obj.weaponRec = "PANTAU";
        let bestEfficiency = -1;

        for (const [wName, wSpecs] of Object.entries(WEAPON_SPECS)) {
            if (obj.distance <= wSpecs.maxRange && realTarget) {
                // Constraint 2: Amunisi (Check state global)
                if (currentAmmo[wName] > 0 && !reloadingStatus[wName]) {
                    // Constraint 3: Geometri LoS untuk Launcher
                    let hasLoS = false;
                    for (let launcherPos of launcherPositions) {
                        const realLauncherPos = {
                            x: launcherPos.x / SCENE_SCALE,
                            y: launcherPos.y / SCENE_SCALE,
                            z: launcherPos.z / SCENE_SCALE
                        };
                        if (checkLineOfSight(realLauncherPos, realTarget)) {
                            hasLoS = true;
                            break;
                        }
                    }
                    if (!hasLoS) continue;

                    // Kalkulasi Pk Estimasi (Heuristic)
                    const estPk = (wSpecs.pk / 100) * (1 - (obj.distance / wSpecs.maxRange));

                    // Objective: Maximize Profit (Pk) / Cost
                    // Kita tambahkan pengali kecil agar rudal mahal tetap terpilih jika skor target sangat tinggi
                    const efficiency = (estPk * obj.score) / (wSpecs.cost || 1);

                    if (efficiency > bestEfficiency) {
                        bestEfficiency = efficiency;
                        obj.weaponRec = `${wName.toUpperCase()} (OPTIMAL)`;
                    }
                }
            }
        }

        if (obj.weaponRec === "PANTAU" && obj.distance > 40000) {
            obj.weaponRec = "PANTAU (OUT OF RANGE)";
        }
    });

    return fused.sort((a, b) => b.score - a.score);
}

// --- SISTEM INTERCEPTOR (RUDAL PENGEJAR) ---
function launchInterceptor(startPos, targetId, weaponType) {
    // startPos adalah Vector3 (Skala Scene) dari Launcher
    // Konversi ke koordinat Real World untuk logika pengejaran
    const realStartPos = {
        x: startPos.x / SCENE_SCALE,
        y: startPos.y / SCENE_SCALE,
        z: startPos.z / SCENE_SCALE
    };

    // Visualisasi Rudal (Cone Sederhana)
    const geometry = new THREE.ConeGeometry(2, 8, 8);
    geometry.rotateX(Math.PI / 2); // Putar agar moncong ke depan
    const material = new THREE.MeshBasicMaterial({ color: 0x00ffff }); // Cyan
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.copy(startPos);
    scene.add(mesh);

    // Kecepatan Rudal (m/s) - Dipercepat 3x agar gameplay tidak terlalu lama menunggu
    let speed = 2000; // Default supersonic
    if (weaponType === 'Starstreak') speed = 3000; // Sangat cepat (Mach 4+)
    else if (weaponType === 'NASAMS') speed = 1500; // Medium

    activeInterceptors.push({
        mesh: mesh,
        pos: realStartPos,
        targetId: targetId,
        weapon: weaponType,
        speed: speed
    });
}

function updateInterceptors(dt) {
    for (let i = activeInterceptors.length - 1; i >= 0; i--) {
        const interceptor = activeInterceptors[i];
        const target = realTargets.find(t => t.id === interceptor.targetId);

        // Jika target hilang/hancur sebelum rudal sampai
        if (!target) {
            scene.remove(interceptor.mesh);
            activeInterceptors.splice(i, 1);
            continue;
        }

        // Hitung vektor arah ke target
        const dx = target.x - interceptor.pos.x;
        const dy = target.y - interceptor.pos.y;
        const dz = target.z - interceptor.pos.z;
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

        // Jarak tempuh frame ini (Speed * DeltaTime * GameplayMultiplier)
        const moveDist = interceptor.speed * dt * 3.0;

        // Cek Tumbukan (Impact)
        if (dist <= moveDist || dist < 100) {
            scene.remove(interceptor.mesh);
            activeInterceptors.splice(i, 1);
            resolveInterceptorImpact(interceptor, target);
        } else {
            // Gerakkan rudal mendekat
            const dir = { x: dx / dist, y: dy / dist, z: dz / dist };
            interceptor.pos.x += dir.x * moveDist;
            interceptor.pos.y += dir.y * moveDist;
            interceptor.pos.z += dir.z * moveDist;

            // Update visual mesh
            interceptor.mesh.position.set(
                interceptor.pos.x * SCENE_SCALE,
                interceptor.pos.y * SCENE_SCALE,
                interceptor.pos.z * SCENE_SCALE
            );
            interceptor.mesh.lookAt(
                target.x * SCENE_SCALE,
                target.y * SCENE_SCALE,
                target.z * SCENE_SCALE
            );

            // Efek asap di belakang rudal
            if (Math.random() > 0.5) spawnSmokeParticle(interceptor.mesh.position);
        }
    }
}

function resolveInterceptorImpact(interceptor, target) {
    const weapon = interceptor.weapon;
    // Hitung jarak dari markas (pusat 0,0,0) untuk validasi range
    const distFromBase = Math.sqrt(target.x * target.x + target.z * target.z);
    const specs = WEAPON_SPECS[weapon];

    // --- RUMUS REALISTIS Pk (Probability of Kill) ---
    const pBase = specs.pk / 100;
    const distFactor = Math.max(0.1, 1 - (distFromBase / specs.maxRange));
    const weatherMod = WEATHER_MODIFIERS[currentWeather] || 1.0;
    const ecmMod = target.type === 'missile' ? 0.8 : 1.0;

    const pkTotal = pBase * distFactor * weatherMod * ecmMod;
    const pkPercent = Math.max(5, Math.min(95, pkTotal * 100));

    const roll = Math.random() * 100;
    const hit = roll < pkPercent;

    const logDiv = document.getElementById('combatLog');
    let msg = `IMPACT: ${weapon} vs ${target.type.toUpperCase()}... `;

    if (hit) {
        msg += `<span style="color:#cf6679; font-weight:bold;">HANCUR! (Splash)</span>`;
        missionStats.kills++;

        // Update status di AAR history
        const lastLog = [...engagementHistory].reverse().find(l => l.targetId === target.id && l.status === 'IN FLIGHT');
        if (lastLog) {
            lastLog.status = 'SUCCESS';
            lastLog.pk = pkPercent.toFixed(1) + '%';
            lastLog.result = 'HIT';
        }

        // --- HITUNG PENGHEMATAN (ECONOMY OF WAR) ---
        // Jika menggunakan Meriam untuk target yang disarankan Meriam, 
        // kita hitung selisihnya dengan rudal termurah (Starstreak)
        if (weapon === 'Oerlikon') {
            const potentialWaste = WEAPON_SPECS["Starstreak"].cost;
            missionStats.costSaved += (potentialWaste - WEAPON_SPECS["Oerlikon"].cost);
        }

        // Visual Ledakan
        createExplosion(target.x, target.y, target.z);

        // Hapus target dari dunia nyata
        realTargets = realTargets.filter(t => t.id !== target.id);
    } else {
        msg += `<span style="color:#ffb74d;">GAGAL (Meleset/Jamming).</span>`;
        const lastLog = [...engagementHistory].reverse().find(l => l.targetId === target.id && l.status === 'IN FLIGHT');
        if (lastLog) {
            lastLog.status = 'FAILED';
            lastLog.pk = pkPercent.toFixed(1) + '%';
            lastLog.result = 'MISS';
        }
    }

    logDiv.innerHTML = msg + "<br>" + logDiv.innerHTML;
    updateStatsUI();
    runSimulationCycle(); // Force update UI segera
}

// --- VISUALISASI ---
function initialize3DScene() {
    const canvas = document.getElementById('sceneCanvas');
    // Ambil ukuran visual dari CSS (500px) agar render tajam
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;

    scene = new THREE.Scene();
    // Perluas jarak pandang kamera (Far Plane) untuk melihat target jauh (150km+)
    camera = new THREE.PerspectiveCamera(75, width / height, 1, 1000000 * SCENE_SCALE); // Diperluas 5x agar langit tidak flickering
    camera.position.set(0, 200, 450); // Posisi kamera sedikit lebih tinggi untuk overview taktis

    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setSize(width, height, false);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;

    // Cahaya
    const ambientLight = new THREE.AmbientLight(0x404040, 2);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
    directionalLight.position.set(5, 10, 7.5);
    scene.add(directionalLight);

    // --- GELAR LINGKAR ARHANUD IKN ---
    createDefenseLayout(); // Panggil fungsi baru untuk membuat layout awal

    // 1. Ganti Grid dengan Ground Plane Berwarna Medan (Tactical Landscape)
    const groundSize = RADAR_RANGE * SCENE_SCALE * 20;
    const groundGeo = new THREE.PlaneGeometry(groundSize, groundSize);
    const groundMat = new THREE.MeshPhongMaterial({
        color: 0x1a2b1a, // Hijau hutan gelap (Khas Kalimantan)
        side: THREE.DoubleSide
    });
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.5;
    scene.add(ground);

    // 2. Tetap tambahkan grid tipis di atas ground untuk referensi jarak
    const gridHelper = new THREE.GridHelper(groundSize, 100, 0x00ff41, 0x222222);
    gridHelper.material.opacity = 0.2;
    gridHelper.material.transparent = true;
    scene.add(gridHelper);

    // Zona Kritis
    const criticalRingGeo = new THREE.RingGeometry(CRITICAL_RANGE * SCENE_SCALE - 1, CRITICAL_RANGE * SCENE_SCALE, 64);
    const criticalRingMat = new THREE.MeshBasicMaterial({ color: 0xcf6679, side: THREE.DoubleSide });
    const criticalRing = new THREE.Mesh(criticalRingGeo, criticalRingMat);
    criticalRing.rotation.x = -Math.PI / 2;
    scene.add(criticalRing);

    // Visualisasi Rintangan Medan (Gunung)
    const mountainGeo = new THREE.ConeGeometry(MOUNTAIN_CONFIG.radius * SCENE_SCALE, MOUNTAIN_CONFIG.height * SCENE_SCALE, 32);
    const mountainMat = new THREE.MeshPhongMaterial({ color: 0x5d4037, flatShading: true });
    const mountain = new THREE.Mesh(mountainGeo, mountainMat);
    mountain.position.set(MOUNTAIN_CONFIG.x * SCENE_SCALE, (MOUNTAIN_CONFIG.height * SCENE_SCALE) / 2, MOUNTAIN_CONFIG.z * SCENE_SCALE);
    scene.add(mountain);
    // Label Gunung? Opsional.

    createSkyDome();
    updateWeaponZones(); // Gambar zona senjata awal
}

/**
 * Membersihkan layout pertahanan lama dan membuat yang baru berdasarkan DEFENSE_RADIUS.
 */
function createDefenseLayout() {
    // 1. Hapus semua objek pertahanan lama dari scene
    defenseSiteMeshes.forEach(obj => scene.remove(obj));
    defenseSiteMeshes = [];
    launcherPositions = [];
    radarRings = [];
    radarMeshes = [];

    // 2. Buat objek pertahanan baru
    const defenseRadScaled = DEFENSE_RADIUS * SCENE_SCALE;

    // Platform Dasar IKN (jika perlu dibuat ulang atau tidak)
    if (!scene.getObjectByName("IKN_Base")) {
        const baseGeo = new THREE.CylinderGeometry(40, 45, 4, 32);
        const baseMat = new THREE.MeshPhongMaterial({ color: 0x222222 });
        const baseMesh = new THREE.Mesh(baseGeo, baseMat);
        baseMesh.position.set(0, 2, 0);
        baseMesh.name = "IKN_Base";
        scene.add(baseMesh);
    }
    // Gedung IKN
    if (!scene.getObjectByName("IKN_Building")) {
        const iknGeo = new THREE.BoxGeometry(15, 60, 15);
        const iknMat = new THREE.MeshPhongMaterial({ color: 0x00bcd4, shininess: 80 });
        const iknMesh = new THREE.Mesh(iknGeo, iknMat);
        iknMesh.position.set(0, 30, 0);
        iknMesh.name = "IKN_Building";
        scene.add(iknMesh);
    }

    // 2. Satuan Tembak (6 Unit: 3 Rudal, 3 Meriam) Gelar Lingkar
    for (let i = 0; i < 6; i++) {
        const angleDeg = i * 60; // 360 / 6 = 60 derajat antar unit
        const angleRad = angleDeg * (Math.PI / 180);
        const x = Math.cos(angleRad) * defenseRadScaled;
        const z = Math.sin(angleRad) * defenseRadScaled;
        const isRudal = i % 2 === 0; // Bergantian Rudal - Meriam

        // Simpan posisi dunia launcher (x, 3, z) -> 3 adalah tinggi launcher
        launcherPositions.push(new THREE.Vector3(x, 3, z));

        // Group untuk satu site pertahanan
        const siteGroup = new THREE.Group();
        siteGroup.position.set(x, 0, z);
        siteGroup.rotation.y = -angleRad; // Menghadap keluar

        // Visual Rudal (Launcher)
        const launcherGeo = new THREE.BoxGeometry(8, 6, 12);
        const launcherMat = new THREE.MeshPhongMaterial({ color: 0xcf6679 }); // Merah
        const launcher = new THREE.Mesh(launcherGeo, launcherMat);
        launcher.position.set(0, 3, 0);
        siteGroup.add(launcher);

        // Visual Radar (Di samping rudal)
        const radarGeo = new THREE.ConeGeometry(4, 8, 8);
        const radarMat = new THREE.MeshPhongMaterial({ color: 0xffeb3b }); // Kuning
        const radar = new THREE.Mesh(radarGeo, radarMat);
        radar.position.set(12, 4, 0); // Offset ke samping
        radar.rotation.z = -0.2; // Miring sedikit
        radarMeshes.push(radar); // Simpan referensi untuk animasi
        siteGroup.add(radar);

        scene.add(siteGroup);
        defenseSiteMeshes.push(siteGroup); // Tambahkan ke daftar untuk dihapus nanti

        // Visual Lingkaran Radar (2D Ring) per unit
        // Tebalkan ring agar terlihat dari jauh (dari 1 unit ke 10 unit)
        const ringGeo = new THREE.RingGeometry(RADAR_RANGE * SCENE_SCALE - 10, RADAR_RANGE * SCENE_SCALE, 128);
        const ringMat = new THREE.MeshBasicMaterial({
            color: 0x03dac6,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.9
        });
        const ring = new THREE.Mesh(ringGeo, ringMat);
        ring.rotation.x = -Math.PI / 2; // Putar agar rata dengan tanah
        ring.position.set(x, 1, z); // Pusat lingkaran di posisi site, sedikit di atas grid
        scene.add(ring);
        defenseSiteMeshes.push(ring); // Tambahkan ke daftar untuk dihapus nanti
        radarRings.push(ring); // Simpan referensi
    }

    // 3. Visualisasi Aset Eksternal (Integrasi)
    createExternalAssets();
}

function createExternalAssets() {
    // --- VISUAL KRI (AL) ---
    const shipGroup = new THREE.Group();
    shipGroup.position.set(KRI_AL_POS.x * SCENE_SCALE, 0, KRI_AL_POS.z * SCENE_SCALE);

    const hullGeo = new THREE.BoxGeometry(100 * SCENE_SCALE * 10, 40 * SCENE_SCALE * 10, 300 * SCENE_SCALE * 10);
    const shipMat = new THREE.MeshPhongMaterial({ color: 0x78909c }); // Grey Navy
    const hull = new THREE.Mesh(hullGeo, shipMat);
    hull.position.y = 10 * SCENE_SCALE * 10;
    shipGroup.add(hull);

    const bridgeGeo = new THREE.BoxGeometry(60 * SCENE_SCALE * 10, 80 * SCENE_SCALE * 10, 60 * SCENE_SCALE * 10);
    const bridge = new THREE.Mesh(bridgeGeo, shipMat);
    bridge.position.y = 50 * SCENE_SCALE * 10;
    shipGroup.add(bridge);

    scene.add(shipGroup);
    defenseSiteMeshes.push(shipGroup);

    // --- VISUAL TOWER RADAR EW / SIPIL ---
    const towerGroup = new THREE.Group();
    towerGroup.position.set(EW_RADAR_POS.x * SCENE_SCALE, 0, EW_RADAR_POS.z * SCENE_SCALE);

    const poleGeo = new THREE.CylinderGeometry(10, 20, 200 * SCENE_SCALE * 10, 8);
    const towerMat = new THREE.MeshPhongMaterial({ color: 0xe0e0e0 });
    const pole = new THREE.Mesh(poleGeo, towerMat);
    pole.position.y = 100 * SCENE_SCALE * 10;
    towerGroup.add(pole);

    const dishGeo = new THREE.SphereGeometry(40 * SCENE_SCALE * 10, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2);
    const dish = new THREE.Mesh(dishGeo, new THREE.MeshPhongMaterial({ color: 0x03dac6, side: THREE.DoubleSide }));
    dish.position.y = 210 * SCENE_SCALE * 10;
    dish.rotation.x = Math.PI / 4;
    towerGroup.add(dish);
    radarMeshes.push(dish); // Biar ikut berputar

    scene.add(towerGroup);
    defenseSiteMeshes.push(towerGroup);
}

function updateWeaponZones() {
    // Hapus zona lama
    weaponZoneMeshes.forEach(m => scene.remove(m));
    weaponZoneMeshes = [];

    const weapon = document.getElementById('weaponSelect').value;
    const specs = WEAPON_SPECS[weapon];
    const range = specs.maxRange * SCENE_SCALE;
    const izRange = range * 0.6; // Asumsi IZ (No Escape Zone) adalah 60% dari max range

    // Visualisasi LRZ (Launch Release Zone) - Kuning Putus-putus
    const lrzGeo = new THREE.RingGeometry(range - 2, range, 128);
    const lrzMat = new THREE.MeshBasicMaterial({ color: 0xffff00, side: THREE.DoubleSide, transparent: true, opacity: 0.3 });
    const lrz = new THREE.Mesh(lrzGeo, lrzMat);
    lrz.rotation.x = -Math.PI / 2;
    lrz.position.y = 5;
    scene.add(lrz);
    weaponZoneMeshes.push(lrz);

    // Visualisasi IZ (Interception Zone) - Merah Transparan
    // Kita gambar ini di tengah (IKN) sebagai representasi cakupan sistem gabungan
    // (Untuk penyederhanaan visual daripada menggambar 3 lingkaran per launcher)
}

function createSkyDome() {
    // Membuat tekstur langit secara prosedural (tanpa gambar eksternal)
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 512;
    const context = canvas.getContext('2d');

    // Gradient: Langit gelap di atas, terang di horizon, gelap di bawah
    const gradient = context.createLinearGradient(0, 0, 0, 512);
    gradient.addColorStop(0, '#0b1026');   // Zenith (Atas) - Biru Gelap
    gradient.addColorStop(0.45, '#2b32b2'); // Langit
    gradient.addColorStop(0.5, '#ff9966');  // Horizon (Sunset)
    gradient.addColorStop(0.55, '#141e30'); // Tanah (Gelap)
    gradient.addColorStop(1, '#000000');    // Nadir (Bawah)

    context.fillStyle = gradient;
    context.fillRect(0, 0, 512, 512);

    // Tambah Bintang
    for (let i = 0; i < 200; i++) {
        const x = Math.random() * 512;
        const y = Math.random() * 230; // Hanya di langit atas
        const r = Math.random() * 1.2;
        const opacity = Math.random();
        context.beginPath();
        context.arc(x, y, r, 0, 2 * Math.PI);
        context.fillStyle = `rgba(255, 255, 255, ${opacity})`;
        context.fill();
    }

    const texture = new THREE.CanvasTexture(canvas);
    // Perbesar SkyDome agar tidak memotong visual target jauh
    const geometry = new THREE.SphereGeometry(200000 * SCENE_SCALE, 32, 32);
    const material = new THREE.MeshBasicMaterial({ map: texture, side: THREE.BackSide });
    const skyDome = new THREE.Mesh(geometry, material);
    scene.add(skyDome);
}

function createExplosion(x, y, z) {
    const explosionGroup = new THREE.Group();
    const particleCount = 50;
    const particleMaterial = new THREE.MeshBasicMaterial({
        color: 0xffa500, // Oranye
        transparent: true
    });

    for (let i = 0; i < particleCount; i++) {
        const particleGeo = new THREE.SphereGeometry(2, 4, 4); // Partikel kecil
        const particle = new THREE.Mesh(particleGeo, particleMaterial.clone());

        // Atur kecepatan acak ke arah luar
        particle.velocity = new THREE.Vector3(
            (Math.random() - 0.5),
            (Math.random() - 0.5),
            (Math.random() - 0.5)
        ).normalize().multiplyScalar(Math.random() * 150 + 50); // Kecepatan

        particle.lifespan = Math.random() * 0.5 + 0.5; // Hidup selama 0.5 - 1 detik
        explosionGroup.add(particle);
    }

    explosionGroup.position.set(x * SCENE_SCALE, y * SCENE_SCALE, z * SCENE_SCALE);
    scene.add(explosionGroup);
    activeExplosions.push(explosionGroup);
}

function updateExplosions(dt) {
    for (let i = activeExplosions.length - 1; i >= 0; i--) {
        const explosion = activeExplosions[i];
        let allParticlesDead = true;

        explosion.children.forEach(particle => {
            if (particle.lifespan > 0) {
                allParticlesDead = false;
                particle.position.addScaledVector(particle.velocity, dt * SCENE_SCALE);
                particle.lifespan -= dt;
                particle.material.opacity = particle.lifespan * 2; // Pudar seiring waktu
            }
        });

        if (allParticlesDead) {
            scene.remove(explosion);
            activeExplosions.splice(i, 1);
        }
    }
}

function spawnSmokeParticle(position) {
    const smokeMaterial = new THREE.MeshBasicMaterial({
        color: 0xcccccc,
        transparent: true,
        opacity: 0.6
    });
    const smokeGeo = new THREE.SphereGeometry(3, 8, 8); // Puff asap
    const smokeParticle = new THREE.Mesh(smokeGeo, smokeMaterial);

    smokeParticle.position.copy(position);

    smokeParticle.lifespan = 2.0; // Detik
    smokeParticle.scale.set(1, 1, 1);

    activeSmokeParticles.push(smokeParticle);
    scene.add(smokeParticle);
}

function updateSmokeTrails(dt) {
    for (let i = activeSmokeParticles.length - 1; i >= 0; i--) {
        const particle = activeSmokeParticles[i];
        particle.lifespan -= dt;

        if (particle.lifespan <= 0) {
            scene.remove(particle);
            activeSmokeParticles.splice(i, 1);
        } else {
            particle.material.opacity = 0.6 * (particle.lifespan / 2.0); // Pudar
            particle.scale.multiplyScalar(1.03); // Membesar
        }
    }
}

function createTargetMesh(target) {
    const model = loadedModels[target.type] || loadedModels["unknown"];
    let newMesh;

    if (model) {
        newMesh = model.clone(); // Selalu clone, jangan gunakan objek asli
    } else {
        // FALLBACK: Jika file model tidak ditemukan, gunakan bentuk geometri sederhana
        // Ini memastikan simulasi tetap berjalan dan kita bisa lihat ada sesuatu.
        console.warn(`Model untuk tipe "${target.type}" tidak ditemukan. Menggunakan bentuk fallback.`);
        let geometry;
        let color = 0xff00ff; // Magenta, warna debug yang bagus

        switch (target.type) {
            case 'missile':
                geometry = new THREE.ConeGeometry(5, 20, 8);
                geometry.rotateX(Math.PI / 2); // Arahkan ke depan
                color = 0xcf6679; // Merah
                break;
            case 'airplane':
                geometry = new THREE.BoxGeometry(20, 4, 15);
                color = 0x00ff00; // Hijau
                break;
            case 'helicopter':
                geometry = new THREE.BoxGeometry(15, 8, 15);
                color = 0xffff00; // Kuning
                break;
            default: // drone & unknown
                geometry = new THREE.SphereGeometry(5, 16, 16);
                color = 0xaaaaaa; // Abu-abu (agar tidak silau jika model gagal load)
                break;
        }
        const material = new THREE.MeshPhongMaterial({ color: color });
        newMesh = new THREE.Mesh(geometry, material);
    }

    // Tambahkan cincin status waspada di bawah setiap target
    const alertRingGeo = new THREE.RingGeometry(20, 23, 32);
    const alertRingMat = new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide, visible: false });
    const alertRing = new THREE.Mesh(alertRingGeo, alertRingMat);
    alertRing.name = 'alert_ring';
    alertRing.rotation.x = -Math.PI / 2;
    alertRing.position.y = -15; // Posisikan di bawah model
    newMesh.add(alertRing);

    const initialPosition = new THREE.Vector3(
        target.x * SCENE_SCALE,
        target.y * SCENE_SCALE,
        target.z * SCENE_SCALE
    );
    newMesh.position.copy(initialPosition);

    // Inisialisasi properti untuk interpolasi posisi
    newMesh.userData.lerpStart = initialPosition.clone();
    newMesh.userData.lerpEnd = initialPosition.clone();
    newMesh.userData.lerpAlpha = 1.0; // Sudah berada di posisi target

    scene.add(newMesh);
    targetMeshes[target.id] = newMesh;
}

function drawTargetsInScene() {
    // 1. Hapus mesh dari target yang sudah hancur
    Object.keys(targetMeshes).forEach(id => {
        if (!realTargets.find(t => t.id === id)) {
            scene.remove(targetMeshes[id]);
            delete targetMeshes[id];
        }
    });

    // 2. Update posisi mesh yang masih ada
    realTargets.forEach(t => {
        const mesh = targetMeshes[t.id];
        if (mesh) {
            // Atur titik awal dan akhir untuk interpolasi visual
            mesh.userData.lerpStart.copy(mesh.position);
            mesh.userData.lerpEnd.set(t.x * SCENE_SCALE, t.y * SCENE_SCALE, t.z * SCENE_SCALE);
            mesh.userData.lerpAlpha = 0; // Reset progress interpolasi
        }
    });
}

function updateUI(prioritized) {
    const tbody = document.querySelector('#targetTable tbody');
    const fireButton = document.getElementById('fireButton');

    // Jika target yang dipilih sudah hancur, hapus seleksi
    if (selectedTargetId && !prioritized.find(t => t.id === selectedTargetId)) {
        selectedTargetId = null;
    }

    tbody.innerHTML = '';

    prioritized.forEach(t => {
        const row = document.createElement('tr');
        row.dataset.id = t.id;
        row.classList.add('target-row');
        if (t.id === selectedTargetId) row.classList.add('selected-row');

        const distStr = t.distance ? (t.distance / 1000).toFixed(1) + ' km' : 'N/A';

        let scoreClass = 'score-low';
        if (t.score >= 100) scoreClass = 'score-high';
        else if (t.score >= 50) scoreClass = 'score-med';

        let statusIndicator = '-';
        let statusColor = '#aaa';
        // Tentukan status berdasarkan kecepatan mendekat (closingSpeed)
        if (t.closingSpeed) {
            if (t.closingSpeed > 25) { // Dianggap mendekat jika > 25 m/s
                statusIndicator = 'IN';
                statusColor = '#cf6679'; // Merah (Bahaya)
            } else if (t.closingSpeed < -25) { // Dianggap menjauh jika < -25 m/s
                statusIndicator = 'OUT';
                statusColor = '#81c784'; // Hijau (Aman)
            }
        }

        let keteranganText = '-';
        let keteranganColor = '#aaa';
        if (t.distance) {
            if (t.distance < 5000) {
                keteranganText = 'BAHAYA';
                keteranganColor = '#cf6679';
            } else if (t.distance < 10000) {
                keteranganText = 'AWAS';
                keteranganColor = '#ffeb3b';
            } else if (t.distance < 15000) {
                keteranganText = 'WASPADA';
                keteranganColor = '#81c784';
            }
        }

        row.innerHTML = `
                    <td>${t.classification.toUpperCase()}</td>
                    <td style="font-size: 0.75em; color: #03dac6;">${t.source}</td>
                    <td class="${scoreClass}">${t.score}</td>
                    <td style="color: ${statusColor}; font-weight: bold;">${statusIndicator}</td>
                    <td style="color: #bb86fc; font-size: 0.85em;"><b>${t.weaponRec}</b></td>
                `;
        tbody.appendChild(row);
    });

    const actionPanel = document.getElementById('actionPanel');
    const recText = document.getElementById('recText');
    const selectedTarget = prioritized.find(t => t.id === selectedTargetId);
    const topTarget = selectedTarget || prioritized[0];

    if (!actionPanel) {
        console.warn("Sistem Taktis: Elemen 'actionPanel' tidak ditemukan di DOM. Update HUD dibatalkan.");
        return;
    }

    if (topTarget && !isGameOver) {
        const weapon = document.getElementById('weaponSelect').value;
        const specs = WEAPON_SPECS[weapon];
        const estPk = (specs.pk * (1 - (topTarget.distance / specs.maxRange))).toFixed(0);
        const cost = specs.cost;

        actionPanel.innerHTML = `
            <div class="action-card primary" onclick="executeFireLogicByTarget('${topTarget.id}')">
                <div style="color: #cf6679; font-weight: bold; font-size: 0.8rem;">OPSI 1: INTERSEPSI SEGERA</div>
                <div style="font-size: 1.1rem; margin: 5px 0;">Tembak ${topTarget.classification.toUpperCase()}</div>
                <div style="font-size: 0.8rem; color: #aaa;">Probabilitas: ${estPk}% | Biaya: Rp ${cost}jt</div>
            </div>
            <div class="action-card secondary" onclick="alert('Tactical Hold: Mengamati pergerakan...')">
                <div style="color: #81c784; font-weight: bold; font-size: 0.8rem;">OPSI 2: HEMAT AMUNISI</div>
                <div style="font-size: 1.1rem; margin: 5px 0;">Pantau & Verifikasi</div>
                <div style="font-size: 0.8rem; color: #aaa;">Risiko Kebocoran: ${topTarget.score > 80 ? 'TINGGI' : 'RENDAH'}</div>
            </div>
        `;
        recText.innerHTML = `REKOMENDASI: Tembak <b>${topTarget.classification.toUpperCase()}</b> (Skor: ${topTarget.score})`;
        fireButton.disabled = false;
    } else {
        actionPanel.innerHTML = `<div class="action-card" style="grid-column: span 2; text-align: center; color: #81c784;">SEKTOR AMAN - TIDAK ADA ANCAMAN AKTIF</div>`;
        recText.textContent = "SEKTOR AMAN - TIDAK ADA ANCAMAN AKTIF";
        fireButton.disabled = true;
    }
}

/** Helper untuk eksekusi via HUD */
function executeFireLogicByTarget(targetId) {
    const target = systemTracks.find(t => t.id === targetId);
    const weapon = document.getElementById('weaponSelect').value;
    if (target) executeFireLogic(target, weapon);
}

function updateAmmoUI() {
    const container = document.getElementById('ammoDisplay');
    container.innerHTML = '';
    for (const [type, count] of Object.entries(currentAmmo)) {
        const isReloading = reloadingStatus[type];
        const div = document.createElement('div');
        div.style.background = '#333';
        div.style.padding = '5px 10px';
        div.style.borderRadius = '4px';
        div.style.flex = '1';
        div.style.textAlign = 'center';

        if (isReloading) {
            div.style.border = '1px solid #ffb74d';
            div.style.color = '#ffb74d';
            div.innerHTML = `<div style="font-size:0.8em;">${type}</div><div style="font-size:0.9em; font-weight:bold; animation: blink 1s infinite;">RELOAD...</div>`;
        } else {
            div.style.border = count > 0 ? '1px solid #555' : '1px solid #cf6679';
            div.style.color = count > 0 ? '#e0e0e0' : '#cf6679';
            div.innerHTML = `<div style="font-size:0.8em; color:#aaa;">${type}</div><div style="font-size:1.2em; font-weight:bold;">${count}</div>`;
        }
        container.appendChild(div);
    }
}

function updateStatsUI() {
    document.getElementById('statKills').innerText = missionStats.kills;
    const acc = missionStats.shots > 0 ? ((missionStats.kills / missionStats.shots) * 100).toFixed(1) : 0;
    document.getElementById('statAccuracy').innerText = acc + "%";
    document.getElementById('statSavings').innerText = `Rp ${missionStats.costSaved}jt`;
}

// --- KONTROL PENGGUNA ---
/**
 * Inisialisasi Kontrol Pengguna dengan Fail-Safe Pattern.
 * Mencegah error 'null' jika elemen HTML tidak ditemukan.
 */
function setupControls() {
    const fireButton = document.getElementById('fireButton');
    const weaponSelect = document.getElementById('weaponSelect');
    const resetButton = document.getElementById('resetButton');
    const reportButton = document.getElementById('reportButton');
    const weatherSelect = document.getElementById('weatherSelect');
    const downloadCsvButton = document.getElementById('downloadCsvButton');
    const radiusSlider = document.getElementById('defenseRadiusSlider');
    const applyLayoutButton = document.getElementById('applyLayoutButton');
    const swarmButton = document.getElementById('swarmButton');

    // 1. Weather
    if (weatherSelect) {
        weatherSelect.addEventListener('change', (e) => {
            currentWeather = e.target.value;
        });
    }

    // 2. Report/AAR
    if (reportButton) {
        reportButton.addEventListener('click', () => {
            const modal = document.getElementById('aarModal');
            const tbody = document.getElementById('aarBody');
            if (!modal || !tbody) return;

            tbody.innerHTML = '';
            let hits = 0;
            let overkillCount = 0;
            let totalExpenditure = 0;
            let missileCost = 0;
            let gunCost = 0;
            let leakageCost = 0;

            engagementHistory.forEach(log => {
                if (log.result === 'HIT') hits++;
                if (log.isOverkill) overkillCount++;
                totalExpenditure += (log.weaponCost || 0);
                if (log.type.includes("LEAKAGE")) {
                    leakageCost += (log.weaponCost || 0);
                } else if (log.weaponCost === WEAPON_SPECS["Oerlikon"].cost) { // Asumsi Oerlikon adalah satu-satunya 'gun'
                    gunCost += (log.weaponCost || 0);
                } else { // Asumsi lainnya adalah 'missile'
                    missileCost += (log.weaponCost || 0);
                }

                const row = document.createElement('tr');
                row.innerHTML = `
                    <td style="padding:8px;">${log.time}</td>
                    <td><b>${log.type}</b></td>
                    <td>${(log.distance / 1000).toFixed(2)}</td>
                    <td>${log.closingSpeed.toFixed(1)}</td>
                    <td style="color:#03dac6">${log.pk || '-'}</td>
                    <td style="font-weight:bold; color:${log.result === 'HIT' ? '#81c784' : '#cf6679'}">${log.result || 'MISS'}</td>
                    <td style="color:#81c784">Rp ${log.weaponCost || 0}jt</td>
                    <td style="font-size:0.75em; color:#aaa;">${log.justification}</td>
                `;
                tbody.appendChild(row);
            });

            // Tambahkan baris Total Pengeluaran di bagian bawah tabel
            if (engagementHistory.length > 0) {
                const totalRow = document.createElement('tr');
                totalRow.style.borderTop = "2px solid #555";
                totalRow.style.background = "rgba(3, 218, 198, 0.1)"; // Background cyan transparan
                totalRow.innerHTML = `
                    <td colspan="6" style="text-align:right; padding:10px; font-weight:bold; color:#03dac6;">TOTAL PENGELUARAN & KERUGIAN:</td>
                    <td style="padding:10px; font-weight:bold; color:#81c784">Rp ${totalExpenditure}jt</td>
                    <td></td>
                `;
                tbody.appendChild(totalRow);
            }

            // Tambahkan baris Rata-rata Biaya per Kill
            if (engagementHistory.length > 0) {
                const avgCostPerKill = missionStats.kills > 0 ? (totalExpenditure / missionStats.kills).toFixed(1) : 0;
                const avgRow = document.createElement('tr');
                avgRow.style.background = "rgba(255, 235, 59, 0.05)"; // Kuning sangat transparan
                avgRow.innerHTML = `
                    <td colspan="6" style="text-align:right; padding:10px; font-weight:bold; color:#ffeb3b;">RATA-RATA BIAYA PER KILL (Efisiensi):</td>
                    <td style="padding:10px; font-weight:bold; color:#ffeb3b">Rp ${avgCostPerKill}jt</td>
                    <td></td>
                `;
                tbody.appendChild(avgRow);
            }

            // --- Pie Chart untuk Distribusi Biaya & Kerugian ---
            const pieChartContainer = document.createElement('div');
            pieChartContainer.className = 'pie-chart-container';

            const totalPieValue = missileCost + gunCost + leakageCost;
            let missilePercentage = totalPieValue > 0 ? (missileCost / totalPieValue) * 100 : 0;
            let gunPercentage = totalPieValue > 0 ? (gunCost / totalPieValue) * 100 : 0;
            let leakagePercentage = totalPieValue > 0 ? (leakageCost / totalPieValue) * 100 : 0;

            let gradientString = [];
            let currentStart = 0;

            if (missilePercentage > 0) {
                let end = currentStart + missilePercentage;
                gradientString.push(`var(--missile-color) ${currentStart}% ${end}%`);
                currentStart = end;
            }
            if (gunPercentage > 0) {
                let end = currentStart + gunPercentage;
                gradientString.push(`var(--gun-color) ${currentStart}% ${end}%`);
                currentStart = end;
            }
            if (leakagePercentage > 0) {
                let end = currentStart + leakagePercentage;
                gradientString.push(`var(--leakage-color) ${currentStart}% ${end}%`);
                currentStart = end;
            }

            let conicGradient = `conic-gradient(${gradientString.join(', ')})`;
            if (gradientString.length === 0) {
                conicGradient = `conic-gradient(transparent)`; // Jika tidak ada data, buat transparan
            }

            pieChartContainer.innerHTML = `
                <h4 style="color:#bb86fc; margin-top:20px; border-top:1px solid #333; padding-top:15px;">Distribusi Biaya & Kerugian</h4>
                <div class="pie-chart" style="background: ${conicGradient};"></div>
                <div class="pie-chart-legend">
                    <div><span class="legend-color" style="background-color: var(--missile-color);"></span> Rudal (${missilePercentage.toFixed(1)}%)</div>
                    <div><span class="legend-color" style="background-color: var(--gun-color);"></span> Meriam (${gunPercentage.toFixed(1)}%)</div>
                    <div><span class="legend-color" style="background-color: var(--leakage-color);"></span> Kebocoran (${leakagePercentage.toFixed(1)}%)</div>
                </div>
            `;
            modal.querySelector('#aarModal > div').appendChild(pieChartContainer);

            modal.style.display = 'flex';
        });
    }

    // 3. Download CSV
    if (downloadCsvButton) {
        downloadCsvButton.addEventListener('click', () => {
            if (engagementHistory.length === 0) {
                alert("Belum ada data penembakan.");
                return;
            }
            const headers = ["Waktu", "Sasaran", "Jarak (km)", "V_Radial (m/s)", "Pk (%)", "Hasil", "Overkill", "Biaya (Rp Juta)", "Justifikasi"];
            const rows = engagementHistory.map(log => [
                log.time, log.type, (log.distance / 1000).toFixed(2), log.closingSpeed.toFixed(1),
                log.pk ? log.pk.replace('%', '') : "0", log.result || "MISS",
                log.isOverkill ? "YA" : "TIDAK", `Rp ${log.weaponCost || 0}jt`, `"${log.justification}"`
            ]);
            const csvContent = [headers, ...rows].map(e => e.join(",")).join("\n");
            const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
            const url = URL.createObjectURL(blob);
            const link = document.createElement("a");
            link.href = url;
            link.download = `AAR_Simulasi_Taktik_Arhanud_Gelar_Lingkar.csv`;
            link.click();
        });
    }

    // --- LOGIKA TOGGLE PETA 2D (LEAFLET) ---
    const toggleViewBtn = document.getElementById('toggleViewButton');
    if (toggleViewBtn) {
        toggleViewBtn.addEventListener('click', () => {
            is2DMode = !is2DMode;
            const canvas = document.getElementById('sceneCanvas');
            const mapDiv = document.getElementById('map');

            if (is2DMode) {
                // Sembunyikan 3D, Tampilkan 2D
                canvas.style.display = 'none';
                mapDiv.style.display = 'block';
                toggleViewBtn.textContent = "🛰️ MODE ENGAGEMENT (3D)";
                if (tacticalMap) tacticalMap.invalidateSize(); // Refresh peta agar tidak abu-abu
            } else {
                // Tampilkan 3D, Sembunyikan 2D
                canvas.style.display = 'block';
                mapDiv.style.display = 'none';
                toggleViewBtn.textContent = "🛰️ MODE PETA TAKTIS (2D)";
            }
        });
    }

    // 4. Reset
    if (resetButton) {
        resetButton.addEventListener('click', resetSimulation);
    }

    // 5. Table Target Selection
    const targetTableBody = document.querySelector('#targetTable tbody');
    if (targetTableBody) {
        targetTableBody.addEventListener('click', (event) => {
            const row = event.target.closest('tr');
            if (row && row.dataset.id) {
                selectedTargetId = row.dataset.id;
                updateUI(systemTracks);
            }
        });
    }

    // 6. Layout Radius
    if (radiusSlider) {
        const radiusValueLabel = document.getElementById('radiusValue');
        radiusSlider.addEventListener('input', () => {
            if (radiusValueLabel) radiusValueLabel.textContent = parseFloat(radiusSlider.value).toFixed(1);
        });
    }
    if (applyLayoutButton) {
        applyLayoutButton.addEventListener('click', () => {
            if (radiusSlider) {
                DEFENSE_RADIUS = parseFloat(radiusSlider.value) * 1000;
                createDefenseLayout();
            }
        });
    }

    // 7. Swarm
    if (swarmButton) {
        swarmButton.addEventListener('click', () => {
            spawnSwarm();
            const logDiv = document.getElementById('combatLog');
            if (logDiv) logDiv.innerHTML = `<span style="color:#ff9800; font-weight:bold;">PERINGATAN: SERANGAN SWARM!</span><br>` + logDiv.innerHTML;
        });
    }

    // 8. Radar Toggles
    [0, 1, 2].forEach(i => {
        const btn = document.getElementById(`btnRadar${i}`);
        if (btn) {
            btn.addEventListener('click', (e) => {
                radarActive[i] = !radarActive[i];
                btn.style.opacity = radarActive[i] ? '1' : '0.4';
                btn.style.backgroundColor = radarActive[i] ? '#03dac6' : '#cf6679';
                if (radarMeshes[i]) radarMeshes[i].visible = radarActive[i];
                if (radarRings[i]) radarRings[i].visible = radarActive[i];
            });
        }
    });

    // 9. Fire Button
    if (fireButton) {
        fireButton.addEventListener('click', () => {
            if (isGameOver) return;
            let targetToEngage = systemTracks.find(t => t.id === selectedTargetId) || systemTracks[0];
            if (targetToEngage && weaponSelect) {
                const weapon = weaponSelect.value;
                executeFireLogic(targetToEngage, weapon);
            }
        });
    }

    // Monte Carlo Analysis Trigger
    const mcButton = document.getElementById('monteCarloButton');
    if (mcButton) {
        mcButton.addEventListener('click', () => {
            const selectedWeapon = document.getElementById('weaponSelect').value;
            const scenario = {
                targets: realTargets.slice(0, 5), // Ambil 5 target terdepan sebagai sampel
                weapon: AssetArhanud.weapons[selectedWeapon]
            };
            if (scenario.targets.length === 0) return alert("Butuh target aktif untuk analisis.");

            MonteCarloEngine.run(scenario, 1000);
        });
    }

    // 10. Update Zona Senjata
    if (weaponSelect) {
        weaponSelect.addEventListener('change', updateWeaponZones);
    }

    // 11. Resize window
    window.addEventListener('resize', () => {
        const canvas = document.getElementById('sceneCanvas');
        if (!canvas || !renderer || !camera) return;
        const width = canvas.clientWidth;
        const height = canvas.clientHeight;
        renderer.setSize(width, height, false);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
    });

    // Hilangkan loading overlay setelah inisialisasi selesai
    const loaderOverlay = document.getElementById('loading-overlay');
    if (loaderOverlay) loaderOverlay.style.display = 'none';
}

/**
 * Logika Eksekusi Penembakan (Fire Logic)
 * Mengelola amunisi, reload, dan peluncuran rudal secara modular.
 */
function executeFireLogic(targetToEngage, weapon) {
    const logDiv = document.getElementById('combatLog');

    // Cek Status Reload
    if (reloadingStatus[weapon]) {
        if (logDiv) logDiv.innerHTML = `<span style="color:#ffb74d;">GAGAL: Sedang reload ${weapon}...</span><br>` + logDiv.innerHTML;
        return;
    }

    // Cek Ketersediaan Amunisi
    if (currentAmmo[weapon] <= 0) {
        reloadingStatus[weapon] = true;
        updateAmmoUI();
        if (logDiv) logDiv.innerHTML = `<span style="color:#03dac6;">INFO: Memulai reload ${weapon} (${WEAPON_SPECS[weapon].reloadTime / 1000}s)...</span><br>` + logDiv.innerHTML;

        setTimeout(() => {
            currentAmmo[weapon] = WEAPON_SPECS[weapon].maxAmmo;
            reloadingStatus[weapon] = false;
            updateAmmoUI();
            if (logDiv) logDiv.innerHTML = `<span style="color:#03dac6; font-weight:bold;">INFO: ${weapon} SIAP!</span><br>` + logDiv.innerHTML;
        }, WEAPON_SPECS[weapon].reloadTime);
        return;
    }

    // Kurangi Amunisi
    currentAmmo[weapon]--;
    updateAmmoUI();
    missionStats.shots++;

    // --- SIMPAN DATA UNTUK BAHAN EVALUASI (AAR) ---
    const rcsVal = RCS_MAP[targetToEngage.classification] || 2;
    const s_dist = ((RADAR_LOCAL_RANGE - targetToEngage.distance) / RADAR_LOCAL_RANGE).toFixed(2);
    const s_speed = (Math.max(0, targetToEngage.closingSpeed || 0) / MAX_SPEED_REF).toFixed(2);

    let justification = `TTI(${s_dist}*${WEIGHTS.TTI}) + Strategic(${rcsVal}/10*${WEIGHTS.STRATEGIC})`;
    if (targetToEngage.distance < CRITICAL_RANGE) justification += ` + CRITICAL_BOOST(100)`;

    const isOverkill = targetToEngage.weaponRec.includes("MERIAM") && weapon !== "Oerlikon";

    engagementHistory.push({
        time: new Date().toLocaleTimeString(),
        type: targetToEngage.classification.toUpperCase(),
        distance: targetToEngage.distance,
        closingSpeed: targetToEngage.closingSpeed || 0,
        totalScore: targetToEngage.score,
        justification: justification,
        targetId: targetToEngage.id,
        status: 'IN FLIGHT',
        isOverkill: isOverkill,
        weaponCost: WEAPON_SPECS[weapon].cost
    });

    // --- LOGIKA PELUNCURAN DARI LAUNCHER TERDEKAT ---
    const realTarget = realTargets.find(t => t.id === targetToEngage.id);
    if (realTarget && launcherPositions.length > 0) {
        const targetPos = new THREE.Vector3(
            realTarget.x * SCENE_SCALE,
            realTarget.y * SCENE_SCALE,
            realTarget.z * SCENE_SCALE
        );

        let nearestLauncher = null;
        let minDist = Infinity;

        launcherPositions.forEach(pos => {
            const d = pos.distanceTo(targetPos);
            if (d < minDist) {
                minDist = d;
                nearestLauncher = pos;
            }
        });

        if (nearestLauncher) {
            spawnSmokeParticle(nearestLauncher);
            launchInterceptor(nearestLauncher, targetToEngage.id, weapon);

            const distKm = (targetToEngage.distance / 1000).toFixed(1);
            if (logDiv) logDiv.innerHTML = `<span style="color:#03dac6;">FOX-3!</span> Meluncurkan ${weapon} ke ${targetToEngage.classification} (${distKm}km)...<br>` + logDiv.innerHTML;
        }
    }
}

function loadModels() {
    // Cek apakah THREE.GLTFLoader tersedia
    if (typeof THREE.GLTFLoader === 'undefined') {
        console.error("GLTFLoader tidak ditemukan. Pastikan script CDN dimuat dengan benar.");
        return Promise.resolve(); // Lanjut dengan fallback geometri dasar
    }

    loader = new THREE.GLTFLoader();

    const promises = Object.entries(modelPaths).map(([type, path]) => {
        return new Promise((resolve, reject) => {
            loader.load(path, (gltf) => {
                // Scaling berdasarkan tipe yang sudah disesuaikan
                if (type === 'fixed-wing' || type === 'EWC') gltf.scene.scale.set(15, 15, 15);
                else if (['SSM', 'AGM', 'RAM'].includes(type)) gltf.scene.scale.set(8, 8, 8);
                else if (type === 'rotary-wing') gltf.scene.scale.set(12, 12, 12);
                else gltf.scene.scale.set(5, 5, 5);

                loadedModels[type] = gltf.scene;
                resolve();
            }, undefined, (error) => {
                console.warn(`Info: Model ${type} tidak ditemukan di folder models/. Menggunakan bentuk dasar.`);
                // Resolve agar simulasi tetap jalan menggunakan fallback di createTargetMesh
                resolve();
            });
        });
    });
    return Promise.all(promises);
}

function resetSimulation() {
    // Hentikan loop lama
    clearInterval(simIntervalId);

    // Reset State
    isGameOver = false;
    currentAmmo = { "NASAMS": 6, "VL MICA": 8, "Starstreak": 12, "Oerlikon": 100 };
    reloadingStatus = { "NASAMS": false, "VL MICA": false, "Starstreak": false, "Oerlikon": false };
    selectedTargetId = null;
    radarActive = [true, true, true];
    missionStats = { kills: 0, shots: 0, costSaved: 0 };
    realTargets = [];
    systemTracks = [];

    // Sembunyikan Modal Game Over
    document.getElementById('gameOverModal').style.display = 'none';

    // Reset UI
    document.getElementById('combatLog').innerHTML = '';
    document.getElementById('recText').innerHTML = 'Menunggu data sensor...';
    document.getElementById('fireButton').disabled = false;
    document.getElementById('weaponSelect').disabled = false;
    updateStatsUI();

    // Reset Tombol Radar
    [0, 1, 2].forEach(i => {
        const btn = document.getElementById(`btnRadar${i}`);
        btn.style.opacity = '1';
        btn.style.backgroundColor = '#03dac6';
    });

    // Bersihkan Scene 3D
    for (const id in targetMeshes) {
        scene.remove(targetMeshes[id]);
    }
    targetMeshes = {};
    // Bersihkan juga partikel asap dan ledakan
    [...activeExplosions, ...activeSmokeParticles].forEach(groupOrParticle => {
        scene.remove(groupOrParticle);
    });
    // Bersihkan interceptor
    activeInterceptors.forEach(i => scene.remove(i.mesh));
    activeInterceptors = [];

    // Reset dan gambar ulang layout pertahanan ke nilai default
    DEFENSE_RADIUS = 5000;
    document.getElementById('defenseRadiusSlider').value = 5;
    document.getElementById('radiusValue').textContent = "5.0";
    createDefenseLayout();

    activeExplosions = [];
    activeSmokeParticles = [];
    updateWeaponZones();

    camera.position.set(0, 200, 450);

    // Re-init
    initRealWorld();
    updateAmmoUI();

    // Mulai Loop Baru
    simIntervalId = setInterval(runSimulationCycle, SIM_TICK);
    runSimulationCycle();
}

// --- LOOP UTAMA ---
function runSimulationCycle() {
    if (isGameOver) return;

    // 1. Update Fisika Dunia Nyata
    if (USE_PYTHON_SERVER) {
        // --- MODE SERVER ---
        // Ambil data, lalu update visualisasi
        fetchServerData().then(() => {
            drawTargetsInScene();
            updateUI(systemTracks);
        });
    } else {
        // --- MODE SIMULASI LOKAL (JS) ---
        updatePhysics(SIM_TICK / 1000);
        // 2. Sensor Membaca Dunia Nyata & Proses Data
        const data = generateSensorData();
        systemTracks = processData(data.camera, data.radar);
        drawTargetsInScene();
        updateUI(systemTracks);
    }
}

function animate() {
    requestAnimationFrame(animate);

    const delta = clock.getDelta();

    controls.update(); // Update kontrol kamera setiap frame (60 FPS)
    updateInterceptors(delta); // Update pergerakan rudal pertahanan

    // Animasikan visual (Radar, Ledakan, Asap) dengan halus
    const rotationSpeed = 1.0 * delta;
    radarMeshes.forEach(radar => {
        radar.rotation.y += rotationSpeed;
    });

    // INTERPOLASI & UPDATE VISUAL TARGET
    for (const id in targetMeshes) {
        const mesh = targetMeshes[id];
        const realTarget = realTargets.find(t => t.id === id); // Cari ground truth yang sesuai

        // Interpolasi Posisi untuk pergerakan halus
        if (mesh.userData.lerpAlpha < 1) {
            // Waktu simulasi per detik (misal: 1000ms -> 1.0s)
            const interpolationTime = SIM_TICK / 1000;
            mesh.userData.lerpAlpha += delta / interpolationTime;
            mesh.userData.lerpAlpha = Math.min(mesh.userData.lerpAlpha, 1.0);

            mesh.position.lerpVectors(
                mesh.userData.lerpStart,
                mesh.userData.lerpEnd,
                mesh.userData.lerpAlpha
            );
        }

        if (realTarget) {
            // Rotasi: Buat model selalu menghadap ke markas
            mesh.lookAt(0, 0, 0);

            // Update visual status waspada berdasarkan jarak interpolasi
            const alertRing = mesh.getObjectByName('alert_ring');
            if (alertRing) {
                const distance = mesh.position.length() / SCENE_SCALE; // Jarak dari pusat (0,0,0)

                if (distance < 5000) { alertRing.material.color.setHex(0xcf6679); alertRing.visible = true; }
                else if (distance < 10000) { alertRing.material.color.setHex(0xffeb3b); alertRing.visible = true; }
                else if (distance < 15000) { alertRing.material.color.setHex(0x81c784); alertRing.visible = true; }
                else { alertRing.visible = false; }
            }

            // Tambahkan jejak asap untuk rudal secara berkala
            if (realTarget.type === 'missile') {
                if (!mesh.userData.smokeTimer) mesh.userData.smokeTimer = 0;
                mesh.userData.smokeTimer += delta;
                if (mesh.userData.smokeTimer > 0.1) { // Spawn asap setiap 0.1 detik
                    spawnSmokeParticle(mesh.position);
                    mesh.userData.smokeTimer = 0;
                }
            }
        }
    }

    updateExplosions(delta);
    updateSmokeTrails(delta);

    renderer.render(scene, camera);
}

// Jalankan
function startSimulation() {
    // Peta 2D dimuat pertama agar UI segera responsif
    initTacticalMap();
    setupControls();
    updateAmmoUI();

    initialize3DScene();

    const actionPanel = document.getElementById('actionPanel');
    if (actionPanel) {
        actionPanel.innerHTML = `<div class="action-card" style="grid-column: span 2; text-align: center; color: #03dac6;">MENGINISIALISASI SISTEM TAKTIS...</div>`;
    } else {
        console.error("KRITIS: Elemen 'actionPanel' tidak ditemukan! Pastikan Anda menggunakan index.html atau tambahkan <div id='actionPanel'></div>");
    }
    document.getElementById('recText').innerHTML = "MENGINISIALISASI SISTEM...";

    loadModels().then(() => {
        console.log("Sistem Siap. Menggunakan model 3D atau fallback geometri.");
        // Hanya spawn target lokal jika TIDAK menggunakan server
        if (!USE_PYTHON_SERVER) {
            initRealWorld();
        }
        initRealWorld();
        simIntervalId = setInterval(runSimulationCycle, SIM_TICK);
        animate(); // Mulai loop rendering visual

        // Pastikan modal tertutup saat start awal
        document.getElementById('gameOverModal').style.display = 'none';
    }).catch(error => {
        if (actionPanel) {
            actionPanel.innerHTML = `<div class="action-card" style="grid-column: span 2; text-align: center; color: #cf6679;">GAGAL MEMUAT MODEL 3D.<br>Pastikan folder 'models' telah diunggah ke repository.</div>`;
        }
        document.getElementById('recText').innerHTML = "GAGAL MEMUAT MODEL.";
    });
}

startSimulation();