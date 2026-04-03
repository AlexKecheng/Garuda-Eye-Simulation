from flask import Flask, jsonify
from flask_cors import CORS
import random

app = Flask(__name__)
CORS(app) # Mengizinkan akses dari browser

@app.route('/api/data', methods=['GET'])
def get_data():
    # Simulasi data yang diharapkan oleh simulation.js
    data = {
        "ground_truth": [
            {
                "id": "target_1",
                "type": "fixed-wing",
                "x": random.randint(-50000, 50000),
                "y": random.randint(2000, 5000),
                "z": random.randint(-50000, 50000),
                "speed": 250
            }
        ]
    }
    return jsonify(data)

if __name__ == '__main__':
    app.run(debug=True, port=5000)