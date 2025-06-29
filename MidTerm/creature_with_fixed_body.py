import genome
import genome_with_fixed_body 
from xml.dom.minidom import getDOMImplementation
from enum import Enum
import numpy as np
import creature

class Creature_with_fixed_body:
    def __init__(self, gene_count):
        self.spec = genome_with_fixed_body.Genome_with_fixed_body.get_gene_spec2()
        self.dna = genome_with_fixed_body.Genome_with_fixed_body.get_random_genome2(gene_count)
        self.flat_links = None
        #self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None
 
    def get_flat_links2(self):
        if self.flat_links == None:
            gdicts = genome_with_fixed_body.Genome_with_fixed_body.get_genome_dicts2(self.dna, self.spec)
            self.flat_links = genome_with_fixed_body.Genome_with_fixed_body.genome_to_flat_links2(gdicts)
        return self.flat_links
    
    def to_xml(self):
        self.get_flat_links2()
        domimpl = getDOMImplementation()
        adom = domimpl.createDocument(None, "start", None)
        robot_tag = adom.createElement("robot")
        for link in self.flat_links:
            robot_tag.appendChild(link.to_link_element(adom))
        first = True
        for link in self.flat_links:
            if first:# skip the root node! 
                first = False
                continue
            robot_tag.appendChild(link.to_joint_element(adom))
        robot_tag.setAttribute("name", "pepe") #  choose a name!
        return '<?xml version="1.0"?>' + robot_tag.toprettyxml()

    def get_motors(self):
        self.get_flat_links2()
        if self.motors == None:
            motors = []
            for i in range(1, len(self.flat_links)):
                l = self.flat_links[i]
                m = creature.Motor(l.control_waveform, l.control_amp,  l.control_freq)
                motors.append(m)
            self.motors = motors 
        return self.motors 
       
    def update_position(self, pos):
        if self.start_position == None:
            self.start_position = pos
        else:
            self.last_position = pos

    def get_distance_travelled0(self):
        if self.start_position is None or self.last_position is None:
            return 0
        p1 = np.asarray(self.start_position)
        p2 = np.asarray(self.last_position)
        dist = np.linalg.norm(p1-p2)
        return dist 


    def get_distance_travelled2(self):
        if self.start_position is None or self.last_position is None:
            return 0
        p1 = np.asarray([0, 0, 5.0])
        p2 = np.asarray(self.last_position)
        THRESHOLD_DISTANCE = 20.0
        dist0 = np.linalg.norm(p1-p2)
        dist = (THRESHOLD_DISTANCE-dist0) if dist0 <= THRESHOLD_DISTANCE else 0
        return dist 
  
    def get_distance_travelled(self):
        return self.get_distance_travelled2() 
   
    def update_dna2(self, dna):
        self.dna = dna
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None
