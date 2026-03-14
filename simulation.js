/*
================================================================================
ASPEK DOKUMENTASI & PENYAJIAN
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
const OBJECT_TYPES = ["drone", "helicopter", "airplane", "missile"];
const THREAT_SCORES = {
    "missile": 100, "airplane": 80, "helicopter": 60, "drone": 40, "unknown": 10
};
// Jarak ditingkatkan agar lebih realistis untuk skala tempur udara
const SCENE_SCALE = 1 / 100; // Skala dunia 3D (1m di dunia nyata = 0.01 unit di 3D)
const RADAR_RANGE = 20000; // 20 km
const VISUAL_RANGE = 15000; // 15 km (Untuk fusi data, tidak mempengaruhi deteksi)
let DEFENSE_RADIUS = 5000; // 5 km dari pusat untuk penempatan radar/rudal (diubah ke let)
const CRITICAL_RANGE = 5000; // 5 km (Game Over)
const SIM_TICK = 1000; // Update setiap 1 detik

// --- ASPEK TEKNIS ALUTSISTA ---
const MIN_ELEVATION_ANGLE = 1.5; // Blind spot radar di bawah 1.5 derajat (Clutter/Horizon)
const MOUNTAIN_CONFIG = { x: 25000, z: 10000, radius: 6000, height: 5000 }; // Rintangan Medan
const WEAPON_SPECS = { // Parameter disesuaikan dengan spesifikasi asli
    "NASAMS": { maxRange: 40000, pk: 80, maxAmmo: 6, reloadTime: 10000 },  // Area Defense (AMRAAM)
    "VL MICA": { maxRange: 20000, pk: 85, maxAmmo: 8, reloadTime: 5000 },   // Point Defense
    "Starstreak": { maxRange: 7000, pk: 95, maxAmmo: 12, reloadTime: 3000 }  // VSHORAD
};

// --- STATE SIMULASI ---
let systemTracks = []; // Apa yang dilihat oleh sistem (hasil fusi)
let realTargets = [];  // Ground Truth (Posisi asli objek di dunia nyata)
const mapCenter = [-7.25, 110.4]; // Konseptual, tidak lagi dipakai untuk render
let currentAmmo = { "NASAMS": 6, "VL MICA": 8, "Starstreak": 12 };
let reloadingStatus = { "NASAMS": false, "VL MICA": false, "Starstreak": false };
let radarActive = [true, true, true]; // Status aktif/mati 3 radar
let selectedTargetId = null; // ID target yang dipilih manual
let missionStats = { kills: 0, shots: 0 };
let radarRings = []; // Referensi ke mesh cincin radar untuk toggle visibilitas
let weaponZoneMeshes = []; // Referensi ke mesh IZ/LRZ
let defenseSiteMeshes = []; // Menyimpan semua objek 3D dari site pertahanan
let isGameOver = false;
let simIntervalId;

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
const loader = new THREE.GLTFLoader();
const loadedModels = {}; // Cache untuk model yang sudah di-load
const modelPaths = {
    "airplane": "models/airplane.glb",
    "helicopter": "models/helicopter.glb",
    "missile": "models/missile.glb",
    "drone": "models/drone.glb",
    "unknown": "models/unknown.glb" // Model fallback
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
    clearInterval(simIntervalId);

    const logDiv = document.getElementById('combatLog');
    logDiv.innerHTML = `<div style="background: #cf6679; color: #000; padding: 10px; font-weight: bold; text-align: center;">
                ⚠️ MARKAS HANCUR! ⚠️<br>
                ${target.type.toUpperCase()} berhasil menembus pertahanan.
            </div>` + logDiv.innerHTML;

    alert(`GAME OVER! Markas dihancurkan oleh ${target.type}.`);

    // Disable controls
    document.getElementById('fireButton').disabled = true;
    document.getElementById('weaponSelect').disabled = true;
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

            // Jarak Target ke Radar ini
            const distToRadar = Math.sqrt((t.x - rx_m) ** 2 + (t.z - rz_m) ** 2);

            // 1. Cek Jangkauan Maksimum
            if (distToRadar > RADAR_RANGE) continue;

            // 2. Cek Blind Spot Elevasi (Radar Horizon / Clutter)
            // Sudut elevasi = atan(ketinggian / jarak_mendatar)
            const elevationAngle = Math.atan2(t.y, distToRadar) * (180 / Math.PI);
            if (elevationAngle < MIN_ELEVATION_ANGLE) continue; // Target terlalu rendah/tenggelam di horizon

            // 3. Cek Rintangan Medan (Gunung) - Line of Sight Check
            // Menggunakan rumus jarak titik ke garis (Line Segment)
            // Garis: Radar(rx_m, rz_m) ke Target(t.x, t.z)
            // Titik: Gunung(MOUNTAIN_CONFIG.x, MOUNTAIN_CONFIG.z)
            const A = t.x - rx_m;
            const B = t.z - rz_m;
            const C = MOUNTAIN_CONFIG.x - rx_m;
            const D = MOUNTAIN_CONFIG.z - rz_m;

            const dot = A * C + B * D;
            const len_sq = A * A + B * B;
            let param = -1;
            if (len_sq !== 0) param = dot / len_sq;

            let xx, zz;
            if (param < 0) { xx = rx_m; zz = rz_m; }
            else if (param > 1) { xx = t.x; zz = t.z; }
            else { xx = rx_m + param * A; zz = rz_m + param * B; }

            const dx = MOUNTAIN_CONFIG.x - xx;
            const dz = MOUNTAIN_CONFIG.z - zz;
            const distToMountainCenter = Math.sqrt(dx * dx + dz * dz);

            // Jika garis pandang memotong radius gunung DAN target di balik gunung DAN target lebih rendah dari gunung
            if (distToMountainCenter < MOUNTAIN_CONFIG.radius) {
                const distRadarToMountain = Math.sqrt((MOUNTAIN_CONFIG.x - rx_m) ** 2 + (MOUNTAIN_CONFIG.z - rz_m) ** 2);
                if (distToRadar > distRadarToMountain && t.y < MOUNTAIN_CONFIG.height) {
                    continue; // BLOCKED BY TERRAIN
                }
            }

            // Jika lolos semua cek, target terdeteksi
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
                angle: noisyAngle,
                closingSpeed: closingSpeed, // Data baru: Kecepatan radial
                trueId: t.id
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
    // Fusi Data: Mencocokkan Radar & Kamera berdasarkan SUDUT (Bearing)
    let fused = [];
    let radarUsed = new Set();
    let cameraUsed = new Set();

    // 1. Loop Radar, cari Kamera yang sudutnya mirip
    radar.forEach((r, rIdx) => {
        let bestCamIdx = -1;
        let minDiff = 10; // Toleransi 10 derajat

        camera.forEach((c, cIdx) => {
            if (cameraUsed.has(cIdx)) return;

            let diff = Math.abs(r.angle - c.angle);
            if (diff > 180) diff = 360 - diff; // Handle crossing 0/360

            if (diff < minDiff) {
                minDiff = diff;
                bestCamIdx = cIdx;
            }
        });

        if (bestCamIdx !== -1) {
            // MATCH: Objek teridentifikasi
            cameraUsed.add(bestCamIdx);
            fused.push({
                id: r.trueId, // Internal tracking
                classification: camera[bestCamIdx].label,
                distance: r.distance,
                angle: r.angle,
                closingSpeed: r.closingSpeed, // Teruskan data kecepatan
                source: "FUSED"
            });
        } else {
            // NO MATCH: Objek tak dikenal (hanya radar)
            fused.push({
                id: r.trueId,
                classification: "unknown",
                distance: r.distance,
                angle: r.angle,
                closingSpeed: r.closingSpeed, // Teruskan data kecepatan
                source: "RADAR"
            });
        }
    });

    // Hitung Skor Prioritas (ALGORITMA BARU)
    fused.forEach(obj => {
        // 1. Base Threat (Berdasarkan Tipe)
        let score = THREAT_SCORES[obj.classification] || 10;

        // 2. Proximity Threat (Ancaman Terdekat ke Obvit/Pusat)
        // Prioritas utama: Seberapa dekat dengan IKN (0,0,0)
        // Jarak < 20km mulai panik.
        const proximityScore = Math.max(0, (1 - (obj.distance / 30000)) * 80);
        score += proximityScore;

        // 3. Kinetic Threat (Berdasarkan Kecepatan Mendekat)
        // Jika mendekat cepat (> 300 m/s atau Mach 0.9), tambah skor signifikan.
        if (obj.closingSpeed && obj.closingSpeed > 0) {
            const kineticScore = Math.min(40, (obj.closingSpeed / 300) * 30);
            score += kineticScore;
        } else {
            // Jika target menjauh, kurangi prioritas karena ancaman berkurang
            score -= 20;
        }

        // 4. Critical Zone (Bahaya Fatal)
        if (obj.distance < CRITICAL_RANGE) score += 100;

        obj.score = Math.round(score);
    });

    // Urutkan
    return fused.sort((a, b) => b.score - a.score);
}

// --- VISUALISASI ---
function initialize3DScene() {
    const canvas = document.getElementById('sceneCanvas');
    // Ambil ukuran visual dari CSS (500px) agar render tajam
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;

    scene = new THREE.Scene();
    // Perluas jarak pandang kamera (Far Plane) untuk melihat target jauh (150km+)
    camera = new THREE.PerspectiveCamera(75, width / height, 1, 200000 * SCENE_SCALE);
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

    // Ground Plane / Grid
    // Grid diperluas untuk mencakup area spawn yang jauh (200km)
    const gridHelper = new THREE.GridHelper(RADAR_RANGE * SCENE_SCALE * 10, 100);
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

    // 2. Satuan Tembak (3 Rudal & 3 Radar) Mengelilingi IKN
    for (let i = 0; i < 3; i++) {
        const angleDeg = i * 120;
        const angleRad = angleDeg * (Math.PI / 180);
        const x = Math.cos(angleRad) * defenseRadScaled;
        const z = Math.sin(angleRad) * defenseRadScaled;

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

function visualizeInterceptor(start, end) {
    // Membuat garis jejak rudal instan (seperti laser/asap cepat)
    const geometry = new THREE.BufferGeometry().setFromPoints([start, end]);
    const material = new THREE.LineBasicMaterial({ color: 0x00ffff, transparent: true, opacity: 0.8 });
    const line = new THREE.Line(geometry, material);
    scene.add(line);

    // Hapus garis setelah 200ms (efek kilatan)
    setTimeout(() => {
        scene.remove(line);
        geometry.dispose();
        material.dispose();
    }, 200);
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
                color = 0xffffff; // Putih
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
                    <td>${distStr}</td>
                    <td class="${scoreClass}">${t.score}</td>
                    <td style="color: ${statusColor}; font-weight: bold;">${statusIndicator}</td>
                    <td style="color: ${keteranganColor}; font-weight: bold;">${keteranganText}</td>
                `;
        tbody.appendChild(row);
    });

    const recText = document.getElementById('recText');
    const selectedTarget = prioritized.find(t => t.id === selectedTargetId);

    if (selectedTarget) {
        recText.innerHTML = `TARGET TERPILIH: <b>${selectedTarget.classification.toUpperCase()}</b> (Skor: ${selectedTarget.score})`;
        fireButton.disabled = false;
    } else if (prioritized.length > 0 && !isGameOver) {
        const top = prioritized[0];
        recText.innerHTML = `Pilih target dari daftar. Rekomendasi: <b>${top.classification.toUpperCase()}</b>`;
        fireButton.disabled = true;
    } else {
        recText.textContent = "Tidak ada ancaman terdeteksi.";
        fireButton.disabled = true;
    }
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
}

// --- KONTROL PENGGUNA ---
function setupControls() {
    const fireButton = document.getElementById('fireButton');
    const weaponSelect = document.getElementById('weaponSelect');
    const resetButton = document.getElementById('resetButton');

    resetButton.addEventListener('click', resetSimulation);

    // Event listener untuk memilih target dari tabel
    document.querySelector('#targetTable tbody').addEventListener('click', (event) => {
        const row = event.target.closest('tr');
        if (row && row.dataset.id) {
            selectedTargetId = row.dataset.id;
            updateUI(systemTracks); // Render ulang UI untuk menampilkan highlight
        }
    });

    // Kontrol Pengaturan Gelar
    const radiusSlider = document.getElementById('defenseRadiusSlider');
    const radiusValueLabel = document.getElementById('radiusValue');
    const applyLayoutButton = document.getElementById('applyLayoutButton');

    radiusSlider.addEventListener('input', () => {
        radiusValueLabel.textContent = parseFloat(radiusSlider.value).toFixed(1);
    });

    applyLayoutButton.addEventListener('click', () => {
        DEFENSE_RADIUS = parseFloat(radiusSlider.value) * 1000; // Konversi km ke meter
        createDefenseLayout();
    });

    // Kontrol Swarm
    document.getElementById('swarmButton').addEventListener('click', () => {
        spawnSwarm();
        const logDiv = document.getElementById('combatLog');
        logDiv.innerHTML = `<span style="color:#ff9800; font-weight:bold;">PERINGATAN: SERANGAN SWARM TERDETEKSI!</span><br>` + logDiv.innerHTML;
    });

    // Kontrol Radar (Toggle)
    [0, 1, 2].forEach(i => {
        document.getElementById(`btnRadar${i}`).addEventListener('click', (e) => {
            radarActive[i] = !radarActive[i];
            e.target.style.opacity = radarActive[i] ? '1' : '0.4';
            e.target.style.backgroundColor = radarActive[i] ? '#03dac6' : '#cf6679';
            // Update visual 3D
            if (radarMeshes[i]) radarMeshes[i].visible = radarActive[i];
            if (radarRings[i]) radarRings[i].visible = radarActive[i];
        });
    });

    // Update Zona Senjata saat ganti senjata
    weaponSelect.addEventListener('change', updateWeaponZones);

    fireButton.addEventListener('click', () => {
        if (isGameOver || !selectedTargetId) return;

        const targetToEngage = systemTracks.find(t => t.id === selectedTargetId);

        if (targetToEngage) {

            const weapon = weaponSelect.value;
            const logDiv = document.getElementById('combatLog');

            // Cek Status Reload
            if (reloadingStatus[weapon]) {
                logDiv.innerHTML = `<span style="color:#ffb74d;">GAGAL: Sedang reload ${weapon}...</span><br>` + logDiv.innerHTML;
                return;
            }

            // Cek Ketersediaan Amunisi
            if (currentAmmo[weapon] <= 0) {
                // Mulai Reload Otomatis
                reloadingStatus[weapon] = true;
                updateAmmoUI();
                logDiv.innerHTML = `<span style="color:#03dac6;">INFO: Memulai reload ${weapon} (${WEAPON_SPECS[weapon].reloadTime / 1000}s)...</span><br>` + logDiv.innerHTML;

                setTimeout(() => {
                    currentAmmo[weapon] = WEAPON_SPECS[weapon].maxAmmo;
                    reloadingStatus[weapon] = false;
                    updateAmmoUI();
                    document.getElementById('combatLog').innerHTML = `<span style="color:#03dac6; font-weight:bold;">INFO: ${weapon} SIAP!</span><br>` + document.getElementById('combatLog').innerHTML;
                }, WEAPON_SPECS[weapon].reloadTime);
                return;
            }

            // Kurangi Amunisi
            currentAmmo[weapon]--;
            updateAmmoUI();
            missionStats.shots++;

            // --- LOGIKA PELUNCURAN DARI LAUNCHER TERDEKAT ---
            const realTarget = realTargets.find(t => t.id === targetToEngage.id);
            if (realTarget) {
                const targetPos = new THREE.Vector3(
                    realTarget.x * SCENE_SCALE,
                    realTarget.y * SCENE_SCALE,
                    realTarget.z * SCENE_SCALE
                );

                // Cari launcher dengan jarak terpendek ke target
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
                    // Visualisasi: Asap di launcher & Jejak ke target
                    spawnSmokeParticle(nearestLauncher);
                    visualizeInterceptor(nearestLauncher, targetPos);
                }
            }

            // Hitung Probabilitas Hit (Pk)
            let pk = 0;
            const distKm = targetToEngage.distance / 1000;
            const specs = WEAPON_SPECS[weapon];

            // Logika Pk Variabel (Berdasarkan Jarak Efektif)
            // Jika di luar jangkauan max, Pk drop drastis
            if (targetToEngage.distance > specs.maxRange) pk = 10;
            // Jika terlalu dekat (min range), Pk juga turun
            else if (targetToEngage.distance < 1000) pk = 30;
            // Ideal
            else pk = specs.pk;

            // Roll Dice
            const roll = Math.random() * 100;
            const hit = roll < pk;

            let msg = `FOX-3! Menembak ${targetToEngage.classification} @ ${distKm.toFixed(1)}km dengan ${weapon}... `;

            if (hit) {
                msg += `<span style="color:#cf6679; font-weight:bold;">SPLASH! Target Hancur.</span>`;
                missionStats.kills++;

                // Cari target asli untuk mendapatkan posisi ledakan
                const destroyedTarget = realTargets.find(t => t.id === targetToEngage.id);
                if (destroyedTarget) {
                    createExplosion(destroyedTarget.x, destroyedTarget.y, destroyedTarget.z);
                }

                // Hapus dari Real World
                realTargets = realTargets.filter(t => t.id !== targetToEngage.id);
            } else {
                msg += `<span style="color:#ffb74d;">MISS! Target bermanuver.</span>`;
            }

            logDiv.innerHTML = msg + "<br>" + logDiv.innerHTML;
            updateStatsUI();

            // Force update segera
            runSimulationCycle();
        }
    });

    window.addEventListener('resize', () => {
        const canvas = document.getElementById('sceneCanvas');
        const width = canvas.clientWidth;
        const height = canvas.clientHeight;
        renderer.setSize(width, height, false);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
    });
}

function loadModels() {
    const promises = Object.entries(modelPaths).map(([type, path]) => {
        return new Promise((resolve, reject) => {
            loader.load(path, (gltf) => {
                // Atur skala dasar model saat pertama kali dimuat
                if (type === 'airplane') gltf.scene.scale.set(15, 15, 15);
                else if (type === 'missile') gltf.scene.scale.set(8, 8, 8);
                else if (type === 'helicopter') gltf.scene.scale.set(12, 12, 12);
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
    currentAmmo = { "NASAMS": 6, "VL MICA": 8, "Starstreak": 12 };
    reloadingStatus = { "NASAMS": false, "VL MICA": false, "Starstreak": false };
    selectedTargetId = null;
    radarActive = [true, true, true];
    missionStats = { kills: 0, shots: 0 };
    realTargets = [];
    systemTracks = [];

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
    initialize3DScene();
    setupControls();
    updateAmmoUI();

    document.getElementById('recText').innerHTML = "SISTEM PERTAHANAN UDARA: MEMUAT DATA...";
    document.getElementById('recText').style.color = "#03dac6"; // Cyan agar terlihat aktif

    loadModels().then(() => {
        console.log("Semua model berhasil dimuat.");
        // Hanya spawn target lokal jika TIDAK menggunakan server
        if (!USE_PYTHON_SERVER) {
            initRealWorld();
        }
        simIntervalId = setInterval(runSimulationCycle, SIM_TICK);
        animate(); // Mulai loop rendering visual
    }).catch(error => {
        document.getElementById('recText').innerHTML = `<span style="color: #cf6679;">GAGAL MEMUAT MODEL 3D.<br>Pastikan folder 'models' beserta isinya telah diunggah ke repository.</span>`;
    });
}

startSimulation();