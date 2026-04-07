import random
import math
from typing import List, Dict
from flask import Flask, jsonify
from flask_cors import CORS

WEATHER_MODIFIERS = {
    "Clear": 1.0,
    "Rain": 0.75,
    "Storm": 0.6
}

# Definisi Topografi (Sinkron dengan simulation.js)
MOUNTAIN_CONFIG = {
    "x": 25000,
    "z": 10000,
    "radius": 6000,
    "height": 5000
}

class K4IPPEngine:
    def __init__(self):
        self.objects = ["fixed-wing", "rotary-wing", "PTTA", "SSM", "AGM", "RAM", "EWC"]
        self.network_status = "ONLINE"
        self.targets = [] # Menyimpan state target dunia nyata
        
        # Spawn initial targets
        for i in range(3):
            self.spawn_target()

    def spawn_target(self):
        angle = random.uniform(0, 360)
        dist = random.uniform(30000, 50000)
        target_id = f"TGT-{random.randint(1000, 9999)}"
        self.targets.append({
            "id": target_id,
            "type": random.choice(self.objects),
            "x": math.cos(math.radians(angle)) * dist,
            "y": random.uniform(1000, 5000),
            "z": math.sin(math.radians(angle)) * dist,
            "speed": random.uniform(150, 400)
        })

    def update_targets(self):
        """Update posisi target (bergerak menuju pusat 0,0,0)"""
        for t in self.targets:
            dist = math.sqrt(t['x']**2 + t['z']**2)
            if dist > 1000:
                vx = -t['x'] / dist * t['speed']
                vz = -t['z'] / dist * t['speed']
                t['x'] += vx
                t['z'] += vz
        if len(self.targets) < 2:
            self.spawn_target()

        # Inisialisasi 4 Kapal KRI untuk Patroli (Poin 10.1 Juknis)
        self.kri_assets = [
            {"id": "KRI-01", "angle": 0, "radius": 80000, "range": 70000},
            {"id": "KRI-02", "angle": 90, "radius": 85000, "range": 70000},
            {"id": "KRI-03", "angle": 180, "radius": 90000, "range": 70000},
            {"id": "KRI-04", "angle": 270, "radius": 82000, "range": 70000}
        ]

    def move_kri_patrol(self, speed_deg: float = 0.5):
        """Membuat kapal bergerak berkeliling area perairan."""
        for kri in self.kri_assets:
            kri["angle"] = (kri["angle"] + speed_deg) % 360
            # Update koordinat kartesius berdasarkan sudut patroli
            kri["x"] = math.cos(math.radians(kri["angle"])) * kri["radius"]
            kri["z"] = math.sin(math.radians(kri["angle"])) * kri["radius"]

    def get_camera_data(self) -> List[Dict]:
        """
        Deteksi dari kamera; setiap entri berisi label dan bounding
        box (x,y,w,h). Di versi lab ini di‑generate acak.
        """
        detections = []
        for _ in range(random.randint(0, 3)):
            label = random.choice(self.objects)
            bbox = (random.randint(0, 640), random.randint(0, 480),
                    random.randint(20, 100), random.randint(20, 100))
            detections.append({"label": label, "bbox": bbox})
        return detections

    def get_radar_data(self, is_jamming: bool = False) -> List[Dict]:
        """
        Return list radar dengan variabel noise dan confidence.
        """
        returns = []
        noise_level = 500 if is_jamming else 100
        accuracy = 0.6 if is_jamming else 0.9

        for _ in range(random.randint(1, 4)):
            distance = random.uniform(500, 25000)
            returns.append({
                "distance": distance + random.uniform(-noise_level, noise_level),
                "angle": random.uniform(0, 360),
                "confidence": random.uniform(accuracy - 0.2, accuracy)
            })
        return returns

def fuse_data(camera: List[Dict], radar: List[Dict]) -> List[Dict]:
    """
    Penggabungan sederhana: setiap deteksi kamera dipasangkan dengan
    satu return radar (pop dari list). Radar sisa ditambahkan sebagai
    objek ‘unknown’.
    """
    fused = []
    radar_copy = radar.copy()
    for det in camera:
        obj = {"type": det["label"], "distance": None, "angle": None}
        if radar_copy:
            r = radar_copy.pop(0)
            obj["distance"] = r["distance"]
            obj["angle"] = r["angle"]
        fused.append(obj)
    for r in radar_copy:
        fused.append({"type": "unknown", "distance": r["distance"], "angle": r["angle"]})
    return fused

def classify_targets(fused: List[Dict]) -> List[Dict]:
    """
    Stub klasifikasi; nanti bisa diganti model CV/ML.
    """
    for obj in fused:
        obj["classification"] = obj.get("type", "unknown")
    return fused

def check_line_of_sight(obs_pos: Dict, tar_pos: Dict) -> bool:
    """
    Menghitung intersec 3D antara garis pandang dan rintangan gunung.
    """
    A = tar_pos['x'] - obs_pos['x']
    B = tar_pos['z'] - obs_pos['z']
    C = MOUNTAIN_CONFIG['x'] - obs_pos['x']
    D = MOUNTAIN_CONFIG['z'] - obs_pos['z']

    len_sq = A**2 + B**2
    param = (A * C + B * D) / len_sq if len_sq != 0 else -1

    if param < 0: xx, zz = obs_pos['x'], obs_pos['z']
    elif param > 1: xx, zz = tar_pos['x'], tar_pos['z']
    else:
        xx = obs_pos['x'] + param * A
        zz = obs_pos['z'] + param * B

    dist_to_mtn = math.sqrt((MOUNTAIN_CONFIG['x'] - xx)**2 + (MOUNTAIN_CONFIG['z'] - zz)**2)
    if dist_to_mtn < MOUNTAIN_CONFIG['radius']:
        dist_obs_to_mtn = math.sqrt(C**2 + D**2)
        dist_obs_to_tar = math.sqrt(len_sq)
        if dist_obs_to_tar > dist_obs_to_mtn:
            los_h = obs_pos['y'] + (tar_pos['y'] - obs_pos['y']) * (dist_obs_to_mtn / dist_obs_to_tar)
            if los_h < MOUNTAIN_CONFIG['height']: return False
    return True

# --- KONFIGURASI MCDM (Multi-Criteria Decision Making) ---
W_DISTANCE = 0.4
W_SPEED = 0.3
W_ANGLE = 0.2
W_RCS = 0.1

MAX_RANGE = 20000  # 20km
MAX_SPEED = 1000   # m/s (Mach 3)
MAX_RCS = 10

RCS_VALUES = {
    "missile": 10,
    "airplane": 5,
    "helicopter": 3,
    "drone": 1,
    "unknown": 2,
}

def prioritize_targets(classified: List[Dict]) -> List[Dict]:
    """
    Implementasi MCDM berdasarkan Pokok Pertimbangan:
    Kekritisan, Kerawanan, Pemulihan, Mobilitas (Poin 2.a.1)
    """
    for obj in classified:
        dist = obj.get("distance", MAX_RANGE)
        speed = random.uniform(100, 400) # Simulasi speed jika tidak ada di data
        azimuth = obj.get("angle", 45)
        altitude = obj.get("altitude", 5000)
        rcs = RCS_VALUES.get(obj["classification"], 2)

        # Sesuai Poin 10.(3): Data diolah menjadi Berita Sasaran (Brasas)
        criticality = (MAX_RANGE - min(dist, MAX_RANGE)) / MAX_RANGE # Kekritisan
        vulnerability = rcs / MAX_RCS # Kerawanan
        recovery = 0.5 # Default heuristic
        mobility = speed / MAX_SPEED

        total_score = (criticality * 0.4) + (vulnerability * 0.25) + \
                      (recovery * 0.15) + (mobility * 0.2)
        
        obj["score"] = round(total_score * 100, 1) # Skala 0-100
    return sorted(classified, key=lambda x: x["score"], reverse=True)

class AAREngine:
    def __init__(self):
        self.total_shots = 0
        self.hits = 0
        self.missile_used = 0
        self.gun_used = 0
        self.wasted_missiles = 0 # Rudal buat drone kecil (Overkill)
        self.targets_leaked = 0  # Sasaran yang masuk ke zona lindung

    def record_action(self, target: Dict, weapon_used: str, result: Dict):
        self.total_shots += 1
        if result["hit"]:
            self.hits += 1
        
        if "RUDAL" in weapon_used:
            self.missile_used += 1
            # Evaluasi Taktis: Jika sistem nyaranin MERIAM tapi Danton pakai RUDAL
            if "MERIAM" in target.get("weapon_recommendation", ""):
                self.wasted_missiles += 1
        else:
            self.gun_used += 1

    def generate_report(self):
        hr = (self.hits / self.total_shots * 100) if self.total_shots > 0 else 0
        evaluation = []
        
        if self.wasted_missiles > 2:
            evaluation.append("- KRITIK: Anda terlalu boros RUDAL untuk sasaran murah. Potensi kehabisan amunisi!")
        else:
            evaluation.append("- PUJIAN: Disiplin amunisi sangat baik. Penggunaan rudal selektif.")

        if hr < 60 and self.total_shots > 0:
            evaluation.append("- KRITIK: Akurasi rendah. Anda cenderung menembak di luar jarak efektif.")
        
        if self.targets_leaked > 0:
            evaluation.append(f"- FATAL: Ada {self.targets_leaked} sasaran lolos. Pertahanan bocor!")
        else:
            evaluation.append("- PRESTASI: Pertahanan udara sempurna. Objek vital aman sepenuhnya.")

        print("\n" + "="*45)
        print("      LAPORAN PASCA-OPERASI (AAR)      ")
        print("="*45)
        print(f"Total Tembakan : {self.total_shots}")
        print(f"Akurasi (Hit)  : {hr:.1f}%")
        print(f"Rudal Terpakai : {self.missile_used}")
        print(f"Meriam Terpakai: {self.gun_used}")
        print("-" * 45)
        print("KESIMPULAN TAKTIS:")
        for note in evaluation:
            print(note)
        print("="*45)

def allocate_weapon(target: Dict, ammo_missile: int, ammo_gun: int) -> str:
    """
    Decision Engine: Logika Penjatahan Amunisi Berbasis Efisiensi & Pk.
    """
    # Parameter Kritis (Adjustable oleh Danton)
    GUN_RANGE_MAX = 5000      # 5km
    MISSILE_RANGE_MAX = 25000 # 25km
    HIGH_SPEED_LIMIT = 800    # m/s (Supersonik)
    
    dist = target.get("distance", 99999)
    speed = target.get("speed", 0) # Simulasi kecepatan
    score = target.get("score", 0) # 0 - 100
    rcs = RCS_VALUES.get(target.get("classification"), 2)

    # Skenario D: Di luar jangkauan
    if dist > MISSILE_RANGE_MAX:
        return "PANTAU (DI LUAR JANGKAUAN TEMBAK)"

    # Skenario A: Sasaran Sangat Berbahaya & Jauh
    if score > 80 and dist > GUN_RANGE_MAX:
        if ammo_missile > 0:
            return "RUDAL (INTERCEPT JARAK JAUH)"
        else:
            return "MERIAM (TUNGGU MASUK JARAK EFEKTIF - AMMO RUDAL HABIS!)"

    # Skenario B: Kecepatan Tinggi (Supersonik)
    elif speed > HIGH_SPEED_LIMIT:
        return "RUDAL (PRIORITAS KECEPATAN)"

    # Skenario C: Drone / Helikopter (RCS Kecil & Lambat)
    elif rcs < 3 and dist <= GUN_RANGE_MAX:
        if ammo_gun > 0:
            return "MERIAM (EFISIENSI BIAYA)"
        return "RUDAL (OVERKILL - MERIAM HABIS)"

    else:
        return "MERIAM (OPTIMAL)"

def evaluate_shot(weapon_type: str, target: Dict, weather: str = "Clear") -> Dict:
    """
    BDA Engine: Menghitung Pk (Probability of Kill) berdasarkan Jarak, Cuaca, dan ECM.
    """
    # Akurasi Dasar (P_base)
    p_base = 0.85 if "RUDAL" in weapon_type else 0.65
    max_range = 25000 if "RUDAL" in weapon_type else 5000
    
    dist = target.get("distance", max_range)
    
    # 1. Modifikator Jarak (D/Dmax)
    dist_factor = max(0.1, 1 - (dist / max_range))
    
    # 2. Modifikator Cuaca
    weather_mod = WEATHER_MODIFIERS.get(weather, 1.0)
    
    # 3. Modifikator ECM (Electronic Countermeasures) - Target militer punya jamming
    ecm_mod = 0.8 if target.get("classification") == "missile" else 1.0
    
    pk_total = p_base * dist_factor * weather_mod * ecm_mod
    pk_total = max(0.05, min(0.95, pk_total)) # Cap 5% - 95%
    
    roll = random.random()
    is_destroyed = roll < pk_total
    
    return {"hit": is_destroyed, "pk": round(pk_total * 100, 2)}

def decision_support(prioritized: List[Dict], ammo_missile: int = 10, ammo_gun: int = 100, weather: str = "Clear") -> Dict:
    """
    Rekomendasi: target dengan skor tertinggi.
    """
    if not prioritized:
        return {}
    
    top_target = prioritized[0]
    top_target["weapon_recommendation"] = allocate_weapon(top_target, ammo_missile, ammo_gun)
    
    # Simulasi evaluasi tembakan jika Danton menembak
    if "PANTAU" not in top_target["weapon_recommendation"]:
        top_target["bda_preview"] = evaluate_shot(top_target["weapon_recommendation"], top_target, weather)
        
    return top_target

# --- FLASK SERVER CONFIG ---
app = Flask(__name__)
CORS(app)
sim_engine = K4IPPEngine()
aar_engine = AAREngine()

@app.route('/api/data', methods=['GET'])
def get_simulation_data():
    # 1. Update Lingkungan
    sim_engine.move_kri_patrol()
    sim_engine.update_targets()
    
    # 2. Simulasi Sensor
    cam = sim_engine.get_camera_data()
    rad = sim_engine.get_radar_data()
    
    # 3. Pengolahan Data (Fusi & Klasifikasi)
    fused = fuse_data(cam, rad)
    classified = classify_targets(fused)
    prioritized = prioritize_targets(classified)
    
    # 4. Rekomendasi Keputusan
    recommendation = decision_support(prioritized)
    
    # Response format sesuai kebutuhan garuda-simulation.js
    return jsonify({
        "status": "ONLINE",
        "ground_truth": sim_engine.targets,
        "tracks": prioritized,
        "recommendation": recommendation,
        "kri_assets": sim_engine.kri_assets
    })

if __name__ == "__main__":
    print("--- GARUDA EYE BACKEND ACTIVE ---")
    print("Server running at http://localhost:5000")
    app.run(port=5000, debug=False)