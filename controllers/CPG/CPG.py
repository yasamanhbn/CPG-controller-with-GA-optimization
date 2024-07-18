# from controller import Robot
# import numpy as np

# class Controller:
    # def __init__(self, num_oscillators, motor_names):
        # self.num_oscillators = num_oscillators
        # self.motor_names = motor_names

        # self.robot = Robot()
        # self.TIME_STEP = int(self.robot.getBasicTimeStep())
        # self.motors = [self.robot.getDevice(name) for name in self.motor_names]
        
        # self.f = 0.5  # Frequency

        # self.phi = np.random.uniform(0, 2 * np.pi, num_oscillators)  # Initial phases
        # self.coupling_weights = np.ones((num_oscillators, num_oscillators))  # Coupling strength
        # self.phase_biases = np.zeros((num_oscillators, num_oscillators))  # Phase biases
        # self.omega = 2 * np.pi * self.f * np.ones(num_oscillators)  # Natural frequencies

        # self.dt = self.TIME_STEP / 1000.0  # Convert time step to seconds

        # self.r = np.random.uniform(-1, 1, num_oscillators)  # Initial amplitudes
        # self.x = np.random.uniform(-1, 1, num_oscillators)  # Initial offsets

        # self.ar = 10  # Amplitude convergence rate
        # self.ax = 10  # Offset convergence rate
        
        # Parameters for the walking gait
        # aC, aF, aT = 0.25, 0.2, 0.05  # amplitudes [rad]
        # dC, dF, dT = 0.6, 0.8, -2.4  # offsets [rad]
        # self.R = [aC, aF, -aT, -aC, -aF, aT, aC, aF, -aT, aC, -aF, aT, -aC, aF, -aT, aC, -aF, aT]
        # self.X = [-dC, dF, dT, 0.0, dF, dT, dC, dF, dT, dC, dF, dT, 0.0, dF, dT, -dC, dF, dT]


    # def update_phases(self):
        # dphi = np.zeros(self.num_oscillators)
        # for i in range(self.num_oscillators):
            # sum_sin = np.sum([
                # self.coupling_weights[i][j] * np.sin(self.phi[j] - self.phi[i] - self.phase_biases[i][j])
                # for j in range(self.num_oscillators) if i != j
            # ])
            # dphi[i] = self.omega[i] + sum_sin
        # new_phi = self.phi + dphi * self.dt
        # new_phi = np.mod(new_phi, 2 * np.pi)  # Wrap phases to the range [0, 2*pi]
        # self.phi = new_phi
        # return new_phi

    # def update_amplitude_offset(self):
        # dri = self.ar * ((self.ar / 4) * (self.R - self.r))
        # dxi = self.ax * ((self.ax / 4) * (self.X - self.x))

        # self.r += dri * self.dt
        # self.x += dxi * self.dt

# def main():
    # motor_names = ["RPC", "RPF", "RPT", "RMC", "RMF", "RMT", "RAC", "RAF", "RAT",
                   # "LPC", "LPF", "LPT", "LMC", "LMF", "LMT", "LAC", "LAF", "LAT"]

    # pC, pF, pT = 0.0, 2.0, 2.5
    # p = [pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT]

    # num_oscillators = len(motor_names)
    # controller = Controller(num_oscillators, motor_names)

    # while controller.robot.step(controller.TIME_STEP) != -1:
        # phi = controller.update_phases()
        # controller.update_amplitude_offset()
        
        # for i in range(num_oscillators):
            # move = controller.r[i] * np.cos(phi[i] + p[i]) + controller.x[i]
            # controller.motors[i].setPosition(move)
            

# if __name__ == "__main__":
    # main()
    
from controller import Robot, Motor
import math
import random
import numpy as np

# Initialize the robot
robot = Robot()
time_step = int(robot.getBasicTimeStep())

# Define the motors
motor_labels = ["RPC", "RPF", "RPT", "RMC", "RMF", "RMT", "RAC", "RAF", "RAT",
                "LPC", "LPF", "LPT", "LMC", "LMF", "LMT", "LAC", "LAF", "LAT"]
motors = [robot.getDevice(label) for label in motor_labels]

class CPG:
    def __init__(self, initial_phase, base_frequency):
        self.phase = initial_phase
        self.frequency = 2 * 3.14 * base_frequency
        self.amplitude = 0
        self.offset = 0
        self.phase_lag = 0
        self.r = 0
        self.x = 0

# Initialize the CPG neurons with more reasonable values
cpgs = [
    CPG(0.0, 0.5), CPG(2.0, 0.5), CPG(2.5, 0.5),
    CPG(0.0, 0.5), CPG(2.0, 0.5), CPG(2.5, 0.5),
    CPG(0.0, 0.5), CPG(2.0, 0.5), CPG(2.5, 0.5),
    CPG(0.0, 0.5), CPG(2.0, 0.5), CPG(2.5, 0.5),
    CPG(0.0, 0.5), CPG(2.0, 0.5), CPG(2.5, 0.5),
    CPG(0.0, 0.5), CPG(2.0, 0.5), CPG(2.5, 0.5)
]

def update_cpg(neuron_list, dt_step):
    for index, neuron in enumerate(neuron_list):
        total_sin = 0
        for j, other_neuron in enumerate(neuron_list):
            if index != j:
                total_sin += math.sin(other_neuron.phase - neuron.phase - neuron.phase_lag)
        
        # Update phase
        neuron.phase += neuron.frequency * dt_step + total_sin * dt_step

        # Update amplitude and offset using simplified dynamics
        neuron.r += 10 * (neuron.amplitude - neuron.r) * dt_step
        neuron.x += 10 * (neuron.offset - neuron.x) * dt_step

class GeneticAlgorithm:
    def __init__(self, population_size, mutation_rate, crossover_rate, elitism_count, tournament_size):
        self.population_size = population_size
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.elitism_count = elitism_count
        self.tournament_size = tournament_size

    def init_population(self, chromosome_length):
        return Population(self.population_size, chromosome_length)

    def calc_fitness(self, individual):
        aC, aF, aT = 0.25, 0.2, 0.05  # amplitudes [rad]
        dC, dF, dT = 0.6, 0.8, -2.4  # offsets

        # Set amplitudes and offsets based on genes
        for i, gene in enumerate(individual.chromosome):
            if i % 3 == 0:  # aC
                cpgs[i].amplitude = aC * gene
            elif i % 3 == 1:  # aF
                cpgs[i].amplitude = aF * gene
            elif i % 3 == 2:  # aT
                cpgs[i].amplitude = aT * gene

        for i, gene in enumerate(individual.chromosome):
            if i % 3 == 0:  # dC
                cpgs[i].offset = dC * gene
            elif i % 3 == 1:  # dF
                cpgs[i].offset = dF * gene
            elif i % 3 == 2:  # dT
                cpgs[i].offset = dT * gene

        # Simulate and calculate fitness
        for _ in range(1000):
            update_cpg(cpgs, time_step / 1000.0)
            for idx, motor in enumerate(motors):
                neuron = cpgs[idx]
                motor_position = neuron.r * math.cos(neuron.phase) + neuron.x
                motor.setPosition(motor_position)
            robot.step(time_step)
        
        # Measure fitness (e.g., forward distance traveled)
        fitness = measure_fitness()
        individual.fitness = fitness
        return fitness

    def eval_population(self, population):
        total_fitness = 0
        for individual in population.individuals:
            total_fitness += self.calc_fitness(individual)
        population.fitness = total_fitness

    # def is_termination_condition_met(self, generations, max_generations):
        # return generations > max_generations

    def select_parent(self, population):
        tournament = Population(self.tournament_size)
        population.shuffle()
        for i in range(self.tournament_size):
            tournament.individuals[i] = population.individuals[i]
        return tournament.get_fittest()

    def crossover_population(self, population):
        new_population = Population(population.size)
        for i in range(population.size):
            parent1 = population.get_fittest(i)
            if self.crossover_rate > random.random() and i >= self.elitism_count:
                parent2 = self.select_parent(population)
                offspring = Individual(len(parent1.chromosome))
                swap_point = random.randint(0, len(parent1.chromosome))
                for j in range(len(parent1.chromosome)):
                    if j < swap_point:
                        offspring.chromosome[j] = parent1.chromosome[j]
                    else:
                        offspring.chromosome[j] = parent2.chromosome[j]
                new_population.individuals[i] = offspring
            else:
                new_population.individuals[i] = parent1
        return new_population

    def mutate_population(self, population):
        new_population = Population(self.population_size)
        for i in range(population.size):
            individual = population.get_fittest(i)
            for j in range(len(individual.chromosome)):
                if i >= self.elitism_count and self.mutation_rate > random.random():
                    individual.chromosome[j] = self.mutate_gene(individual.chromosome[j])
            new_population.individuals[i] = individual
        return new_population

    def mutate_gene(self, gene):
        mutation_value = random.uniform(-0.05, 0.05)
        return gene + mutation_value

class Population:
    def __init__(self, size, chromosome_length=None):
        self.size = size
        self.individuals = [Individual(chromosome_length) for _ in range(size)]
        self.fitness = 0

    def get_fittest(self, index=0):
        return sorted(self.individuals, key=lambda x: x.fitness, reverse=True)[index]

    def shuffle(self):
        random.shuffle(self.individuals)

class Individual:
    def __init__(self, chromosome_length):
        self.chromosome = [random.uniform(0.8, 1.2) for _ in range(chromosome_length)]
        self.fitness = 0

def measure_fitness():
    initial_position = [0, 0, 0]  # Assuming robot starts at origin
    
    # Perform the kinematics calculations here
    final_position = calculate_end_effector_position(cpgs)
    
    # Measure distance traveled forward
    distance_traveled = math.sqrt(
        (final_position[0] - initial_position[0])**2 +
        (final_position[1] - initial_position[1])**2 +
        (final_position[2] - initial_position[2])**2
    )
    
    return distance_traveled

def calculate_end_effector_position(cpgs):
    # Example function to calculate the end effector's position
    # based on the CPG parameters and the robot's configuration
    # You need to replace this with the actual kinematics calculations of your robot
    x = sum([math.cos(cpg.phase) * cpg.amplitude for cpg in cpgs])
    y = sum([math.sin(cpg.phase) * cpg.amplitude for cpg in cpgs])
    z = sum([cpg.offset for cpg in cpgs])
    
    return [x, y, z]

def main():
    ga = GeneticAlgorithm(100, 0.01, 0.9, 2, 5)
    population = ga.init_population(len(cpgs))

    max_generations = 1
    generation = 0

    while generation  < 1:
        ga.eval_population(population)
        print(f"Generation: {generation}, Fitness: {population.fitness}")
        population = ga.crossover_population(population)
        population = ga.mutate_population(population)
        generation += 1

    best_individual = population.get_fittest()
    print("Best CPG parameters:", best_individual.chromosome)

    for i, gene in enumerate(best_individual.chromosome):
        if i % 3 == 0:  # aC
            cpgs[i].amplitude = 0.25 * gene
        elif i % 3 == 1:  # aF
            cpgs[i].amplitude = 0.2 * gene
        elif i % 3 == 2:  # aT
            cpgs[i].amplitude = 0.05 * gene

    for i, gene in enumerate(best_individual.chromosome):
        if i % 3 == 0:  # dC
            cpgs[i].offset = 0.6 * gene
        elif i % 3 == 1:  # dF
            cpgs[i].offset = 0.8 * gene
        elif i % 3 == 2:  # dT
            cpgs[i].offset = -2.4 * gene

    while robot.step(time_step) != -1:
        update_cpg(cpgs, time_step / 1000.0)
        for idx, motor in enumerate(motors):
            neuron = cpgs[idx]
            motor_position = neuron.r * math.cos(neuron.phase) + neuron.x
            motor.setPosition(motor_position)

if __name__ == "__main__":
    main()
