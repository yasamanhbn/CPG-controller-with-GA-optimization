from controller import Robot
import numpy as np

class Controller:
    def __init__(self, num_oscillators, motor_names):
        self.num_oscillators = num_oscillators
        self.motor_names = motor_names

        self.robot = Robot()
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        self.motors = [self.robot.getDevice(name) for name in self.motor_names]
        
        self.f = 0.5  # Frequency

        self.phi = np.random.uniform(0, 2 * np.pi, num_oscillators)  # Initial phases
        self.coupling_weights = np.ones((num_oscillators, num_oscillators))  # Coupling strength
        self.phase_biases = np.zeros((num_oscillators, num_oscillators))  # Phase biases
        self.omega = 2 * np.pi * self.f * np.ones(num_oscillators)  # Natural frequencies

        self.dt = self.TIME_STEP / 1000.0  # Convert time step to seconds

        self.r = np.random.uniform(-1, 1, num_oscillators)  # Initial amplitudes
        self.x = np.random.uniform(-1, 1, num_oscillators)  # Initial offsets

        self.ar = 10  # Amplitude convergence rate
        self.ax = 10  # Offset convergence rate
        
        Parameters for the walking gait
        aC, aF, aT = 0.25, 0.2, 0.05  # amplitudes [rad]
        dC, dF, dT = 0.6, 0.8, -2.4  # offsets [rad]
        self.R = [aC, aF, -aT, -aC, -aF, aT, aC, aF, -aT, aC, -aF, aT, -aC, aF, -aT, aC, -aF, aT]
        self.X = [-dC, dF, dT, 0.0, dF, dT, dC, dF, dT, dC, dF, dT, 0.0, dF, dT, -dC, dF, dT]


    def update_phases(self):
        dphi = np.zeros(self.num_oscillators)
        for i in range(self.num_oscillators):
            sum_sin = np.sum([
                self.coupling_weights[i][j] * np.sin(self.phi[j] - self.phi[i] - self.phase_biases[i][j])
                for j in range(self.num_oscillators) if i != j
            ])
            dphi[i] = self.omega[i] + sum_sin
        new_phi = self.phi + dphi * self.dt
        new_phi = np.mod(new_phi, 2 * np.pi)  # Wrap phases to the range [0, 2*pi]
        self.phi = new_phi
        return new_phi

    def update_amplitude_offset(self):
        dri = self.ar * ((self.ar / 4) * (self.R - self.r))
        dxi = self.ax * ((self.ax / 4) * (self.X - self.x))

        self.r += dri * self.dt
        self.x += dxi * self.dt

def main():
    motor_names = ["RPC", "RPF", "RPT", "RMC", "RMF", "RMT", "RAC", "RAF", "RAT",
                   "LPC", "LPF", "LPT", "LMC", "LMF", "LMT", "LAC", "LAF", "LAT"]

    pC, pF, pT = 0.0, 2.0, 2.5
    p = [pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT]

    num_oscillators = len(motor_names)
    controller = Controller(num_oscillators, motor_names)

    while controller.robot.step(controller.TIME_STEP) != -1:
        phi = controller.update_phases()
        controller.update_amplitude_offset()
        
        for i in range(num_oscillators):
            move = controller.r[i] * np.cos(phi[i] + p[i]) + controller.x[i]
            controller.motors[i].setPosition(move)
            

if __name__ == "__main__":
    main()
