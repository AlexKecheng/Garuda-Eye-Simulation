import { EyeSimulation } from './simulation/eye';

const startSimulation = () => {
    const simulation = new EyeSimulation();
    simulation.startSimulation();
};

const stopSimulation = () => {
    const simulation = new EyeSimulation();
    simulation.stopSimulation();
};

// Initialize the simulation
startSimulation();

// Export functions for potential use in other modules
export { startSimulation, stopSimulation };