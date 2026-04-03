import { EyeSimulation } from './simulation/eye';

// Allow checking `require.main` in a TS project without adding Node types.
declare const require: any;

const simulation = new EyeSimulation();

const startSimulation = () => {
    simulation.startSimulation();
};

const stopSimulation = () => {
    simulation.stopSimulation();
};

// Export functions for potential use in other modules
export { startSimulation, stopSimulation };

// If this file is executed directly (`node dist/index.js`), start the simulation.
// This avoids side effects during import (helps tests and reusability).
if (typeof require !== 'undefined' && require.main === module) {
    startSimulation();
}