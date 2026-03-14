import { EyeSimulation } from '../src/simulation/eye';

describe('EyeSimulation', () => {
    let simulation: EyeSimulation;

    beforeEach(() => {
        simulation = new EyeSimulation();
    });

    test('should start the simulation', () => {
        simulation.startSimulation();
        expect(simulation.isRunning).toBe(true);
    });

    test('should stop the simulation', () => {
        simulation.startSimulation();
        simulation.stopSimulation();
        expect(simulation.isRunning).toBe(false);
    });

    test('should not start the simulation if already running', () => {
        simulation.startSimulation();
        simulation.startSimulation();
        expect(simulation.isRunning).toBe(true);
    });

    test('should not stop the simulation if not running', () => {
        simulation.stopSimulation();
        expect(simulation.isRunning).toBe(false);
    });
});