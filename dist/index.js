"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.stopSimulation = exports.startSimulation = void 0;
const eye_1 = require("./simulation/eye");
const simulation = new eye_1.EyeSimulation();
const startSimulation = () => {
    simulation.startSimulation();
};
exports.startSimulation = startSimulation;
const stopSimulation = () => {
    simulation.stopSimulation();
};
exports.stopSimulation = stopSimulation;
// If this file is executed directly (`node dist/index.js`), start the simulation.
// This avoids side effects during import (helps tests and reusability).
if (typeof require !== 'undefined' && require.main === module) {
    startSimulation();
}
