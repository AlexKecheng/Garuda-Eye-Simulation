import random
from typing import List, Dict

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

THREAT_SCORES = {
    "missile": 100,
    "airplane": 70,
    "helicopter": 60,
    "drone": 40,
    "unknown": 10,
}

def prioritize_targets(classified: List[Dict]) -> List[Dict]:
    """
    Beri skor ancaman dan urutkan menurun.
    Skor dihitung berdasarkan tipe ancaman dan jarak.
    """
    for obj in classified:
        base_score = THREAT_SCORES.get(obj["classification"], THREAT_SCORES["unknown"])
        distance_score = 0
        if obj.get("distance") and obj["distance"] < 500: # Ancaman jarak sangat dekat
            distance_score = 50
        elif obj.get("distance") and obj["distance"] < 1500: # Ancaman jarak dekat
            distance_score = 20
        obj["score"] = base_score + distance_score
    return sorted(classified, key=lambda x: x["score"], reverse=True)

def decision_support(prioritized: List[Dict]) -> Dict:
    """
    Rekomendasi: target dengan skor tertinggi.
    """
    if not prioritized:
        return {}
    return prioritized[0]

def run_cycle(sim):
    cam = sim.get_camera_data()
    rad = sim.get_radar_data()
    fused = fuse_data(cam, rad)
    classified = classify_targets(fused)
    prioritized = prioritize_targets(classified)
    recommendation = decision_support(prioritized)

    print("Camera detections:", cam)
    print("Radar returns:   ", rad)
    print("Fused objects:   ", fused)
    print("Prioritized:     ", prioritized)
    print("Recommendation:  ", recommendation)
    print("-" * 40)

def main():
    sim = SensorSimulator()
    for _ in range(5):
        run_cycle(sim)

if __name__ == "__main__":
    main()