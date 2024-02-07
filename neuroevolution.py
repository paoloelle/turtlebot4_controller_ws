import random

from deap import creator, base, tools
from random import uniform
import os

def main():  # get the fitness value for every individual and perform the evolution

    pop = toolbox.population(n=50)  # create population

    # get the fitness value for every individual
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):  # associate for every individual its fitness
        ind.fitness.values = fit

    # extract all the fitnesses of every individuals
    fits = [ind.fitness.values[0] for ind in pop]

    g = 0  # number of generation
    while g < 100:
        g = g + 1
        print("-- Generation %i --" % g)

        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))

        for mutant in offspring:
            toolbox.mutate(mutant)
            del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        pop[:] = offspring

        # Gather all the fitnesses in one list and print the stats
        fits = [ind.fitness.values[0] for ind in pop]

        length = len(pop)
        mean = sum(fits) / length
        sum2 = sum(x * x for x in fits)
        std = abs(sum2 / length - mean ** 2) ** 0.5

        print("  Min %s" % min(fits))
        print("  Max %s" % max(fits))
        print("  Avg %s" % mean)
        print("  Std %s" % std)


def get_objects_collected(individual):
    # os("ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=arena") # write a launch file that launch also controller node
    # use subprocess module

    return 


creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)

toolbox = base.Toolbox()

# attribute generator
toolbox.register("attr_weights", uniform, -0.5, 0.5)

# structure initializers
# let's assume the I have like 4 input neurons, 8 hidden neurons, and 3 output neurons (67 parameters for a genotype)
# define the structure of a genome
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_weights, 67)
# define the population structure
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

toolbox.register("evaluate", get_objects_collected)

# genetic operators (tournament selection, no elitism, small mutation)
toolbox.register("select", tools.selTournament, tournsize=3)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=100)

main()
