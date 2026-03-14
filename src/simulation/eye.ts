class EyeSimulation {
    private isRunning: boolean;

    constructor() {
        this.isRunning = false;
    }

    startSimulation(): void {
        if (!this.isRunning) {
            this.isRunning = true;
            console.log("Eye simulation started.");
            // Additional logic for starting the simulation
        } else {
            console.log("Eye simulation is already running.");
        }
    }

    stopSimulation(): void {
        if (this.isRunning) {
            this.isRunning = false;
            console.log("Eye simulation stopped.");
            // Additional logic for stopping the simulation
        } else {
            console.log("Eye simulation is not running.");
        }
    }
}

export default EyeSimulation;