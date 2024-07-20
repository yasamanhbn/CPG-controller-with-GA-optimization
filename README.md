# CPG Controller with GA Optimization

This repository contains the implementation of a Central Pattern Generator (CPG) model optimized using a Genetic Algorithm (GA) for controlling a Mantis robot.

## Overview

The project demonstrates the integration of a CPG model with GA optimization to enhance the locomotion capabilities of a Mantis robot in a Webots simulation environment. The CPG controller is based on the principles discussed in the following paper: [Learning to Move in Modular Robots using Central Pattern Generators and Online Optimization](https://www.researchgate.net/publication/220121826_Learning_to_Move_in_Modular_Robots_using_Central_Pattern_Generators_and_Online_Optimization).

## Setup Instructions

To run the code, follow these steps:

1. **Add the world to Webots:**
   - Open the Webots simulation environment.
   - Add the provided world file to the Webots environment.

2. **Choose the controller:**
   - Select the appropriate controller for the robot in Webots.

## Controllers

### CPG Controller

The CPG controller implements a central pattern generator based on the mentioned paper. The hyperparameters for this controller are selected experimentally and are not optimized.

### GA Controller

The GA controller uses optimized hyperparameters of the CPG controller obtained via a genetic algorithm. The optimization process is run for 10 generations with an initial population of 30.

## Usage

To use the controllers, follow these steps:

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/CPG-controller-with-GA-optimization.git
   cd CPG-controller-with-GA-optimization
