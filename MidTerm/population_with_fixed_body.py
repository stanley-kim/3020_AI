import creature_with_fixed_body 
import numpy as np

class Population_with_fixed_body:
    def __init__(self, pop_size, gene_count):
        self.creatures = [creature_with_fixed_body.Creature_with_fixed_body(
                          gene_count=gene_count) 
                          for i in range(pop_size)]

    @staticmethod
    def get_fitness_map2(fits):
        fitmap = []
        total = 0
        for f in fits:
            total = total + f
            fitmap.append(total)
        return fitmap
    
    @staticmethod
    def select_parent2(fitmap):
        r = np.random.rand() # 0-1
        r = r * fitmap[-1]
        for i in range(len(fitmap)):
            if r <= fitmap[i]:
                return i

