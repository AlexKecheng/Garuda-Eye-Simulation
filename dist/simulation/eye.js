"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.EyeSimulation = void 0;
class EyeSimulation {
    constructor() {
        this.isRunning = false;
    }
    startSimulation() {
        if (!this.isRunning) {
            this.isRunning = true;
            console.log("Eye simulation started.");
            // Additional logic for starting the simulation
        }
        else {
            console.log("Eye simulation is already running.");
        }
    }
    stopSimulation() {
        if (this.isRunning) {
            this.isRunning = false;
            console.log("Eye simulation stopped.");
            // Additional logic for stopping the simulation
        }
        else {
            console.log("Eye simulation is not running.");
        }
    }
    isSimulationRunning() {
        return this.isRunning;
    }
}
exports.EyeSimulation = EyeSimulation;
exports.default = EyeSimulation;
