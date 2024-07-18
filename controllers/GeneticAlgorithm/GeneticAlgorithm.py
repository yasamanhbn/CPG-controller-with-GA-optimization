from controller import Robot
import numpy as np
import random
from controller import GPS
import time

pC, pF, pT = 0.0, 2.0, 2.5
p = [pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT]

class MyRobot:
    def __init__(self):
        self.robot = Robot()
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        self.dt = self.TIME_STEP / 1000.0  # Convert time step to seconds
        
        self.motor_names = ["RPC", "RPF", "RPT", "RMC", "RMF", "RMT", "RAC", "RAF", "RAT",
                   "LPC", "LPF", "LPT", "LMC", "LMF", "LMT", "LAC", "LAF", "LAT"]
        self.num_oscillators = len(self.motor_names)

        self.motors = [self.robot.getDevice(name) for name in self.motor_names]
        
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.TIME_STEP)

    def set_controller(self, controller):
        self.controller = controller

    def run(self):
        steps = 0          
        while self.robot.step(self.TIME_STEP) != -1:
            if steps == 0:
                initial_position = self.gps.getValues()
            phi = self.controller.update_phases()
            self.controller.update_amplitude_offset()
            
            for i in range(self.num_oscillators):
                move = self.controller.r[i] * np.cos(phi[i] + p[i]) + self.controller.x[i]
                self.motors[i].setPosition(move)
            if steps > 50:
                break
            steps+=1

        final_position = self.gps.getValues()
        distance_traveled = np.sqrt((final_position[0] - initial_position[0])**2 + (final_position[2] - initial_position[2])**2)
                                    
        time_elapsed = steps * self.dt
        speed = distance_traveled / time_elapsed
        return speed
        
    def final_run(self):
        while self.robot.step(self.TIME_STEP) != -1:
            phi = self.controller.update_phases()
            self.controller.update_amplitude_offset()
            
            for i in range(self.num_oscillators):
                move = self.controller.r[i] * np.cos(phi[i] + p[i]) + self.controller.x[i]
                self.motors[i].setPosition(move)                           
        return

class Controller:
    def __init__(self, num_oscillators, f, ar, ax, aC, aF, aT, dC, dF, dT):
        self.num_oscillators = num_oscillators

        self.f = f  # Frequency

        self.phi = np.random.uniform(0, 2 * np.pi, num_oscillators)  # Initial phases
        self.coupling_weights = np.ones((num_oscillators, num_oscillators))  # Coupling strength
        self.phase_biases = np.zeros((num_oscillators, num_oscillators))  # Phase biases
        self.omega = 2 * np.pi * self.f * np.ones(num_oscillators)  # Natural frequencies

        self.dt = 32 / 1000.0  # Convert time step to seconds

        self.r = np.random.uniform(-1, 1, num_oscillators)  # Initial amplitudes
        self.x = np.random.uniform(-1, 1, num_oscillators)  # Initial offsets

        self.ar = ar  # Amplitude convergence rate
        self.ax = ax  # Offset convergence rate
        
        # Parameters for the walking gait
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

def genetic_algorithm(population_size, generations, num_oscillators, robot):
    def create_individual():
        f = random.uniform(0.1, 1.0)
        ar = random.uniform(5, 15)
        ax = random.uniform(5, 15)
        aC = random.uniform(0.1, 0.5)
        aF = random.uniform(0.1, 0.5)
        aT = random.uniform(0.01, 0.1)
        dC = random.uniform(0.5, 1.0)
        dF = random.uniform(0.5, 1.0)
        dT = random.uniform(-3.0, -2.0)
        return [f, ar, ax, aC, aF, aT, dC, dF, dT]

    def crossover(parent1, parent2):
        child = parent1[:]
        for i in range(len(child)):
            if random.random() < 0.5:
                child[i] = parent2[i]
        return child

    def mutate(individual):
        mutation_rate = 0.1
        for i in range(len(individual)):
            if random.random() < mutation_rate:
                if i == 0:
                    individual[i] = random.uniform(0.1, 1.0)
                elif i < 3:
                    individual[i] = random.uniform(5, 15)
                elif i < 6:
                    individual[i] = random.uniform(0.1, 0.5) if i < 5 else random.uniform(0.01, 0.1)
                else:
                    individual[i] = random.uniform(0.5, 1.0) if i < 8 else random.uniform(-3.0, -2.0)
        return individual

    def select(population, fitnesses):
        total_fitness = sum(fitnesses)
        pick = random.uniform(0, total_fitness)
        current = 0
        for individual, fitness in zip(population, fitnesses):
            current += fitness
            if current > pick:
                return individual

    # Initialize population
    population = [create_individual() for _ in range(population_size)]
    
    for generation in range(generations):
        fitnesses = []
        for individual in population:
            controller = Controller(num_oscillators, *individual)
            robot.set_controller(controller)
            fitness = robot.run()
            fitnesses.append(fitness)
        
        new_population = []
        for _ in range(population_size // 2):
            parent1 = select(population, fitnesses)
            parent2 = select(population, fitnesses)
            child1 = crossover(parent1, parent2)
            child2 = crossover(parent2, parent1)
            new_population.append(mutate(child1))
            new_population.append(mutate(child2))
        
        population = new_population

        best_individual = max(zip(population, fitnesses), key=lambda x: x[1])[0]
        print(f'Generation {generation + 1}: Best Individual {best_individual}')
    
    best_individual = max(zip(population, fitnesses), key=lambda x: x[1])[0]
    return best_individual

if __name__ == "__main__":

    robot = MyRobot()
    num_oscillators = robot.num_oscillators

    population_size = 30
    generations = 10
    
    best_params = genetic_algorithm(population_size, generations, num_oscillators, robot)
    print(f'Best Hyperparameters: {best_params}')

    controller = Controller(num_oscillators, *best_params)
    velocity = robot.run()
    print(f'Final speed: {velocity}')
    robot.final_run()
