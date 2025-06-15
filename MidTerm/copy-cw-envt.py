import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import creature
import simulation
import population
import genome
import os

# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

import random
#import pybullet as p
import math
#import pybullet as p



def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    def gaussian(x, y, sigma=arena_size/4):
        """Return the height of the mountain at position (x, y) using a Gaussian function."""
        return mountain_height * math.exp(-((x**2 + y**2) / (2 * sigma**2)))

    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = gaussian(x, y)  # Height determined by the Gaussian function

        # Adjust the size of the rocks based on height. Higher rocks (closer to the peak) will be smaller.
        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor

        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)



def make_rocks(num_rocks=100, max_size=0.25, arena_size=10):
    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = 0.5  # Adjust based on your needs
        size = random.uniform(0.1,max_size)
        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)


def make_arena(arena_size=10, wall_height=1):
    wall_thickness = 0.5
    floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness])
    floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1])
    floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    # Create four walls
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, wall_height/2])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size/2, 0, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size/2, 0, wall_height/2])

def default_simple_example() :
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0, 0, -10)

    arena_size = 20
    make_arena(arena_size=arena_size)

    #make_rocks(arena_size=arena_size)

    mountain_position = (0, 0, -1)  # Adjust as needed
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
    p.setAdditionalSearchPath('shapes/')
    # mountain = p.loadURDF("mountain.urdf", mountain_position, mountain_orientation, useFixedBase=1)
    # mountain = p.loadURDF("mountain_with_cubes.urdf", mountain_position, mountain_orientation, useFixedBase=1)

    mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)

    # generate a random creature
    cr = creature.Creature(gene_count=3)
    # save it to XML
    with open('test.urdf', 'w') as f:
        f.write(cr.to_xml())
    # load it into the sim
    rob1 = p.loadURDF('test.urdf', (0, 0, 10))

    p.setRealTimeSimulation(1)

def run_the_simul_in_the_arena(connect_mode):
    # generate a random creature
    cr = creature.Creature(gene_count=3)    
    sim = simulation.Simulation()
    if p.GUI == connect_mode:
        sim.set_as_GUI()
        print("run as GUI Mode")
    else:
        print("run as DIRECT Mode")
    sim.run_one_creature_in_the_arena(cr, 2400 * 100)

def GA_without_threads(population_size=10, gene_count=3, generation_count=1000, motor_update_count=2400):
    print(f'population_size({population_size}), gene_count({gene_count}), generation_count({generation_count}), motor_update_count({motor_update_count})')
    pop = population.Population(pop_size=population_size, 
                                gene_count=gene_count)
    #sim = simulation.ThreadedSim(pool_size=1)
    sim = simulation.Simulation()

    for iteration in range(generation_count):
        # this is a non-threaded version 
        # where we just call run_creature instead
        # of eval_population 
        for cr in pop.creatures:
            sim.run_one_creature_in_the_arena(cr=cr, motor_update_count=motor_update_count)            
#            sim.run_creature(cr, 2400)            
        #sim.eval_population(pop, 2400)
        fits = [cr.get_distance_travelled() 
                for cr in pop.creatures]
        links = [len(cr.get_expanded_links()) 
                for cr in pop.creatures]
        if iteration % 100 == 0 or iteration == generation_count -1:
            print(iteration, "fittest:", np.round(np.max(fits), 3), 
                "mean:", np.round(np.mean(fits), 3), "mean links", np.round(np.mean(links)), "max links", np.round(np.max(links)))       
        fit_map = population.Population.get_fitness_map(fits)
        new_creatures = []
        for i in range(len(pop.creatures)):
            p1_ind = population.Population.select_parent(fit_map)
            p2_ind = population.Population.select_parent(fit_map)
            p1 = pop.creatures[p1_ind]
            p2 = pop.creatures[p2_ind]
            # now we have the parents!
            dna = genome.Genome.crossover(p1.dna, p2.dna)
            dna = genome.Genome.point_mutate(dna, rate=0.1, amount=0.25)
            dna = genome.Genome.shrink_mutate(dna, rate=0.25)
            dna = genome.Genome.grow_mutate(dna, rate=0.1)
            cr = creature.Creature(1)
            cr.update_dna(dna)
            new_creatures.append(cr)
        DIRNAME_CSVS = 'csvs'
        if not os.path.isdir(DIRNAME_CSVS):
            os.mkdir(DIRNAME_CSVS)
        # elitism
        max_fit = np.max(fits)
        for cr in pop.creatures:
            if cr.get_distance_travelled() == max_fit:
                new_cr = creature.Creature(1)
                new_cr.update_dna(cr.dna)
                new_creatures[0] = new_cr   
                filename = os.path.join(DIRNAME_CSVS, "elite_"+ "P" + str(population_size) + "_" + "GC" + str(gene_count)  + "_" + "IT" + str(iteration).zfill(5)+"_" + str(np.round(max_fit, 3))  +".csv")
                genome.Genome.to_csv(cr.dna, filename)
                break
         
        pop.creatures = new_creatures

def GA_with_threads(population_size=10, gene_count=3, generation_count=1000, motor_update_count=2400):

    pop = population.Population(pop_size=population_size, 
                                gene_count=gene_count)
    sim = simulation.ThreadedSim(pool_size=4)
    #sim = simulation.Simulation()

    for iteration in range(generation_count):
        sim.eval_population_in_the_arena(pop, motor_update_count)
        fits = [cr.get_distance_travelled() 
                for cr in pop.creatures]
        links = [len(cr.get_expanded_links()) 
                for cr in pop.creatures]
        if iteration % 100 == 0 or iteration == generation_count -1:
            print(iteration, "fittest:", np.round(np.max(fits), 3), 
                "mean:", np.round(np.mean(fits), 3), "mean links", np.round(np.mean(links)), "max links", np.round(np.max(links)), population_size, gene_count)       
        fit_map = population.Population.get_fitness_map(fits)
        new_creatures = []
        for i in range(len(pop.creatures)):
            p1_ind = population.Population.select_parent(fit_map)
            p2_ind = population.Population.select_parent(fit_map)
            p1 = pop.creatures[p1_ind]
            p2 = pop.creatures[p2_ind]
            # now we have the parents!
            dna = genome.Genome.crossover(p1.dna, p2.dna)
            dna = genome.Genome.point_mutate(dna, rate=0.1, amount=0.25)
            dna = genome.Genome.shrink_mutate(dna, rate=0.25)
            dna = genome.Genome.grow_mutate(dna, rate=0.1)
            cr = creature.Creature(1)
            cr.update_dna(dna)
            new_creatures.append(cr)

        DIRNAME_CSVS = 'csvs'
        if not os.path.isdir(DIRNAME_CSVS):
            os.mkdir(DIRNAME_CSVS)

        # elitism
        max_fit = np.max(fits)
        for cr in pop.creatures:
            if cr.get_distance_travelled() == max_fit:
                new_cr = creature.Creature(1)
                new_cr.update_dna(cr.dna)
                new_creatures[0] = new_cr
                filename = os.path.join(DIRNAME_CSVS, "elite_"+ "P" + str(population_size) + "_" + "GC" + str(gene_count)  + "_" + "IT" + str(iteration).zfill(5)+"_" + str(np.round(max_fit, 3))  +".csv")
                genome.Genome.to_csv(cr.dna, filename)
                break
        
        pop.creatures = new_creatures


if __name__ == '__main__':
#    for ps in [10, 20, 30, 40, 50]:
#    for ps in [10, ]:
        # for gc in [3, 5, 7, 9]:
 #       for gc in [3, ]:
  #          for genc in [500, ]:
   #             GA_with_threads(population_size=ps, gene_count=gc, generation_count=genc, motor_update_count=2400)
    GA_with_threads(population_size=10, gene_count=3, generation_count=20, motor_update_count=2400)
    GA_with_threads(population_size=10, gene_count=5, generation_count=20, motor_update_count=2400)
    GA_with_threads(population_size=10, gene_count=7, generation_count=20, motor_update_count=2400)
#    run_the_simul(p.DIRECT)
#    run_the_simul_in_the_arena(p.GUI)
#    default_simple_example()
