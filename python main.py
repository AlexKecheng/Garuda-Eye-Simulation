import random
import math
from typing import List, Dict

WEATHER_MODIFIERS = {
    "Clear": 1.0,
    "Rain": 0.75,
    "Storm": 0.6
}

class SensorSimulator:
    def __init__(self):
        # daftar tipe objek yang bisa “dikecam”
        self.objects = ["drone", "helicopter", "airplane", "missile"]

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

    def get_radar_data(self) -> List[Dict]:
        """
        Return list radar: jarak dan sudut.
        """
        returns = []
        for _ in range(random.randint(0, 3)):
            returns.append({
                "distance": random.uniform(100, 3000),
                "angle": random.uniform(-45, 45)
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
    Implementasi Algoritma MCDM untuk Prioritas Sasaran.
    """
    for obj in classified:
        dist = obj.get("distance", MAX_RANGE)
        speed = random.uniform(100, 400) # Simulasi speed jika tidak ada di data
        angle = obj.get("angle", 45)     # Simulasi aspect angle
        rcs = RCS_VALUES.get(obj["classification"], 2)

        # 1. Normalisasi Parameter (0.0 - 1.0)
        score_dist = (MAX_RANGE - min(dist, MAX_RANGE)) / MAX_RANGE
        score_speed = min(speed, MAX_SPEED) / MAX_SPEED
        score_angle = math.cos(math.radians(angle))
        score_rcs = rcs / MAX_RCS

        # 2. Total Skor Berbobot
        total_score = (score_dist * W_DISTANCE) + \
                      (score_speed * W_SPEED) + \
                      (score_angle * W_ANGLE) + \
                      (score_rcs * W_RCS)
        
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

def run_cycle(sim, aar):
    cam = sim.get_camera_data()
    rad = sim.get_radar_data()
    fused = fuse_data(cam, rad)
    classified = classify_targets(fused)
    prioritized = prioritize_targets(classified)
    weather = "Clear"
    recommendation = decision_support(prioritized, weather=weather)

    if recommendation and "PANTAU" not in recommendation.get("weapon_recommendation", ""):
        weapon = recommendation["weapon_recommendation"]
        result = evaluate_shot(weapon, recommendation, weather=weather)
        aar.record_action(recommendation, weapon, result)

    for obj in prioritized:
        if obj.get("distance", 99999) < 5000:
            aar.targets_leaked += 1

    print("Camera detections:", cam)
    print("Radar returns:   ", rad)
    print("Recommendation:  ", recommendation)

def main():
    sim = SensorSimulator()
    aar = AAREngine()
    for _ in range(5):
        run_cycle(sim, aar)
    aar.generate_report()

if __name__ == "__main__":
    main()