import random
import math
from typing import List, Dict
from flask import Flask, jsonify
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from flask_cors import CORS

WEATHER_MODIFIERS = {
    "Clear": 1.0,
    "Rain": 0.75,
    "Storm": 0.6
}

# Konfigurasi Topografi & Geografi Akademi Militer (Akmil)
ELEVASI_RADAR = 300 
RINTANGAN_ALAM = [
    {"nama": "Gunung Tidar", "jarak_m": 2000, "ketinggian_m": 503},
    {"nama": "Gunung Sumbing", "jarak_m": 18000, "ketinggian_m": 3371},
]

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
            "speed": random.uniform(150, 400),
            "vx": 0,
            "vz": 0
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
                t['vx'] = vx
                t['vz'] = vz
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
            detections.append({"label": label, "bbox": bbox, "id": f"CAM-{random.randint(1,99)}"})
        return detections

    def get_radar_data(self, is_jamming: bool = False) -> List[Dict]:
        """
        Membaca target real-time dengan filter Terrain Masking (LoS).
        """
        returns = []
        noise_level = 500 if is_jamming else 50
        accuracy = 0.6 if is_jamming else 0.95

        for t in self.targets:
            dist = math.sqrt(t['x']**2 + t['z']**2)
            if dist > 30000: continue # Batas deteksi radar
            
            # Filter Terrain Masking (LoS)
            visible, obstacle = check_line_of_sight(t)
            if not visible: continue

            returns.append({
                "id": t['id'],
                "distance": dist + random.uniform(-noise_level, noise_level),
                "angle": math.degrees(math.atan2(t['z'], t['x'])),
                "confidence": random.uniform(accuracy - 0.1, accuracy),
                "x": t['x'], "y": t['y'], "z": t['z'],
                "vx": t['vx'], "vz": t['vz'], "speed": t['speed']
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
        obj = {"type": det["label"], "distance": None, "angle": None, "id": det.get("id")}
        if radar_copy:
            r = radar_copy.pop(0)
            obj["distance"] = r["distance"]
            obj["angle"] = r["angle"]
            obj.update({k: r[k] for k in ["x", "y", "z", "vx", "vz", "speed", "id"]})
        fused.append(obj)
    for r in radar_copy:
        obj = {"type": "unknown", "distance": r["distance"], "angle": r["angle"]}
        obj.update({k: r[k] for k in ["x", "y", "z", "vx", "vz", "speed", "id"]})
        fused.append(obj)
    return fused

def classify_targets(fused: List[Dict]) -> List[Dict]:
    """
    Stub klasifikasi; nanti bisa diganti model CV/ML.
    """
    for obj in fused:
        obj["classification"] = obj.get("type", "unknown")
    return fused

def check_line_of_sight(target: Dict) -> (bool, str):
    """
    Fungsi Pengecekan Tabir Medan (Terrain Masking) berbasis database gunung Akmil.
    """
    dist_m = math.sqrt(target['x']**2 + target['z']**2)
    target_alt = target['y']
    
    sudut_target = math.atan2((target_alt - ELEVASI_RADAR), dist_m)
    
    for mtn in RINTANGAN_ALAM:
        if dist_m > mtn["jarak_m"]:
            sudut_mtn = math.atan2((mtn["ketinggian_m"] - ELEVASI_RADAR), mtn["jarak_m"])
            # Jika sudut target lebih rendah dari sudut puncak gunung, maka tertutup.
            if sudut_target < sudut_mtn:
                return False, mtn["nama"]
                
    return True, None

# --- KONFIGURASI FUZZY-SAW (Sistem Pendukung Keputusan Fuzzy) ---
MAX_RANGE = 20000  # 20km
MAX_SPEED = 1000   # m/s (Mach 3)
MAX_RCS = 10

RCS_VALUES = {
    "SSM": 10, # Surface-to-Surface Missile
    "AGM": 10, # Air-to-Ground Missile
    "RAM": 10, # Rocket Artillery Missile
    "fixed-wing": 5,
    "rotary-wing": 3,
    "PTTA": 1, # Pesawat Terbang Tanpa Awak (Drone)
    "EWC": 5, # Electronic Warfare Craft (Pesawat EW)
    "unknown": 2,
}

# --- Definisi Variabel Fuzzy ---
# Antecedent (Input) Variables
distance = ctrl.Antecedent(fuzz.universe_range(0, MAX_RANGE, 1), 'distance')
speed = ctrl.Antecedent(fuzz.universe_range(0, MAX_SPEED, 1), 'speed')
rcs = ctrl.Antecedent(fuzz.universe_range(0, MAX_RCS, 1), 'rcs')

# Consequent (Output) Variable
threat_score = ctrl.Consequent(fuzz.universe_range(0, 100, 1), 'threat_score')

# Membership Functions for Inputs
# Jarak: Semakin dekat, semakin tinggi ancaman (Z-shaped untuk 'very_close', S-shaped untuk 'far')
distance['very_close'] = fuzz.zmf(distance.universe, 0, 7000) # 0-7km sangat dekat
distance['close'] = fuzz.trimf(distance.universe, [5000, 10000, 15000]) # 5-15km dekat
distance['medium'] = fuzz.trimf(distance.universe, [10000, 15000, 20000]) # 10-20km sedang
distance['far'] = fuzz.smf(distance.universe, 15000, MAX_RANGE) # 15km+ jauh

# Kecepatan: Semakin cepat, semakin tinggi ancaman
speed['slow'] = fuzz.zmf(speed.universe, 0, 300) # 0-300 m/s lambat
speed['medium'] = fuzz.trimf(speed.universe, [200, 500, 800]) # 200-800 m/s sedang
speed['fast'] = fuzz.smf(speed.universe, 700, MAX_SPEED) # 700 m/s+ cepat

# RCS: Semakin besar, semakin tinggi ancaman
rcs['small'] = fuzz.zmf(rcs.universe, 0, 4) # 0-4 kecil
rcs['medium'] = fuzz.trimf(rcs.universe, [2, 5, 8]) # 2-8 sedang
rcs['large'] = fuzz.smf(rcs.universe, 6, MAX_RCS) # 6+ besar

# Membership Functions for Output (Threat Score 0-100)
threat_score['low'] = fuzz.trimf(threat_score.universe, [0, 0, 40])
threat_score['medium'] = fuzz.trimf(threat_score.universe, [20, 50, 80])
threat_score['high'] = fuzz.trimf(threat_score.universe, [60, 100, 100])

# --- Aturan Fuzzy (Contoh Sederhana, perlu diperluas untuk sistem nyata) ---
rules = [
    # Skenario Ancaman Tinggi
    ctrl.Rule(distance['very_close'] & speed['fast'] & rcs['large'], threat_score['high']),
    ctrl.Rule(distance['very_close'] & speed['medium'] & rcs['large'], threat_score['high']),
    ctrl.Rule(distance['close'] & speed['fast'] & rcs['large'], threat_score['high']),

    # Skenario Ancaman Sedang
    ctrl.Rule(distance['medium'] & speed['medium'] & rcs['medium'], threat_score['medium']),
    ctrl.Rule(distance['far'] & speed['fast'] & rcs['large'], threat_score['medium']), # Jauh tapi cepat & RCS besar
    ctrl.Rule(distance['very_close'] & speed['slow'] & rcs['small'], threat_score['medium']), # Dekat tapi lambat & RCS kecil

    # Skenario Ancaman Rendah
    ctrl.Rule(distance['far'] & speed['slow'] & rcs['small'], threat_score['low']),
    ctrl.Rule(distance['medium'] & speed['slow'] & rcs['small'], threat_score['low']),
]

# --- Sistem Kontrol Fuzzy ---
threat_control_system = ctrl.ControlSystem(rules)
threat_simulation = ctrl.ControlSystemSimulation(threat_control_system)

def prioritize_targets(classified: List[Dict]) -> List[Dict]:
    """
    Implementasi SPK Fuzzy-SAW untuk menentukan skor prioritas ancaman.
    Menggunakan sistem inferensi fuzzy berdasarkan jarak, kecepatan, dan RCS.
    """
    for obj in classified:
        dist = obj.get("distance", MAX_RANGE)
        target_speed = obj.get("speed", 0)
        rcs = RCS_VALUES.get(obj["classification"], 2)

        # Clamp input values to the universe of discourse for fuzzy system
        # Pastikan nilai input berada dalam rentang yang didefinisikan oleh universe
        dist_input = max(0, min(dist, MAX_RANGE))
        speed_input = max(0, min(target_speed, MAX_SPEED))
        rcs_input = max(0, min(rcs, MAX_RCS))

        # Masukkan nilai crisp ke sistem fuzzy
        threat_simulation.input['distance'] = dist_input
        threat_simulation.input['speed'] = speed_input
        threat_simulation.input['rcs'] = rcs_input

        try:
            # Lakukan komputasi fuzzy
            threat_simulation.compute()
            # Ambil nilai defuzzifikasi sebagai skor ancaman
            obj["score"] = round(threat_simulation.output['threat_score'], 1)
        except ValueError as e:
            # Tangani kasus di mana komputasi fuzzy gagal (misalnya, input di luar universe)
            print(f"Fuzzy computation error for target {obj.get('id')}: {e}. Defaulting score to 0.")
            obj["score"] = 0 # Beri skor rendah jika ada error

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
    
    return {"hit": is_destroyed, "pk": round(pk_total * 100, 1)}

def calculate_debris_impact(target: Dict) -> Dict:
    """
    Algoritma Titik Jatuh Puing (Predictive Debris Footprint)
    Menghitung proyeksi jatuhnya puing berdasarkan vektor kecepatan dan gravitasi.
    """
    g = 9.81
    x, y, z = target.get('x', 0), target.get('y', 0), target.get('z', 0)
    vx, vz = target.get('vx', 0), target.get('vz', 0)
    
    # Estimasi waktu jatuh bebas (t = sqrt(2h/g))
    t_fall = math.sqrt(max(0, 2 * y / g))
    
    # Proyeksi horizontal puing akibat inersia
    impact_x = x + (vx * t_fall)
    impact_z = z + (vz * t_fall)
    
    # Jarak titik jatuh ke Akmil (titik 0,0)
    dist_to_base = math.sqrt(impact_x**2 + impact_z**2)
    
    # Radius Zona Bahaya (Ksatrian Akmil)
    AKMIL_DANGER_RADIUS = 1000 # 1km
    
    risk = "SAFE"
    if dist_to_base < AKMIL_DANGER_RADIUS:
        risk = "CRITICAL"
    elif dist_to_base < AKMIL_DANGER_RADIUS * 2.5:
        risk = "WARNING"
        
    return {
        "impact_x": impact_x, "impact_z": impact_z,
        "dist_to_base": dist_to_base, "risk": risk
    }

def decision_support(prioritized: List[Dict], ammo_missile: int = 10, ammo_gun: int = 100, weather: str = "Clear") -> Dict:
    """
    Rekomendasi Taktis Strategis: Menggabungkan SPK Fuzzy dengan Mitigasi Kerusakan Kolateral.
    """
    if not prioritized:
        return {}
    
    top_target = prioritized[0]
    
    # 1. Analisis Risiko Puing (Inertia Physics)
    debris = calculate_debris_impact(top_target)
    top_target["debris_analysis"] = debris
    
    # 2. Alokasi Senjata Optimal
    rec = allocate_weapon(top_target, ammo_missile, ammo_gun)
    
    # Simulasi evaluasi tembakan jika Danton menembak
    if "PANTAU" not in rec:
        # Intervensi AI: Cek Mitigasi Kerusakan Kolateral
        if debris["risk"] == "CRITICAL" and top_target.get("distance", 9999) > 6500:
            top_target["weapon_recommendation"] = "HOLD FIRE (RISIKO PUING TINGGI)"
            top_target["justification"] = "Puing diprediksi jatuh di Ksatrian Akmil. Menunda tembakan 5-10 detik untuk area aman."
        else:
            top_target["weapon_recommendation"] = rec
            top_target["justification"] = "Area jatuh puing aman (Lereng Gunung Tidar)."
            
        top_target["bda_preview"] = evaluate_shot(top_target["weapon_recommendation"], top_target, weather)
    else:
        top_target["weapon_recommendation"] = rec

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
        "kri_assets": sim_engine.kri_assets,
        "debris_analysis": recommendation.get("debris_analysis", {}) # Pastikan data debris dikirim
    })

if __name__ == "__main__":
    print("--- GARUDA EYE BACKEND ACTIVE ---")
    print("Server running at http://localhost:5000")
    app.run(port=5000, debug=False)