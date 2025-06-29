import numpy as np
import copy 
import random
import genome

class Genome_with_fixed_body():
    @staticmethod 
    def get_random_gene2(length):
        gene = np.array([np.random.random() for i in range(length)])
        return gene
     
    @staticmethod 
    def get_random_genome2(gene_count=1):
        gene_length=len(Genome_with_fixed_body.get_gene_spec2())
        genome = [Genome_with_fixed_body.get_random_gene2(gene_length) for i in range(gene_count)]
        return genome
 
    @staticmethod
    def get_gene_spec2():
        gene_spec =  {
            "link-shape":{"scale":1}, 
            "link-length": {"scale":2},
            "link-radius": {"scale":1},
            #fixed "link-recurrence": {"scale":3},
            "link-mass": {"scale":1},
            "joint-type": {"scale":1},
            #fixed "joint-parent":{"scale":1},
            "joint-axis-xyz": {"scale":1},
            "joint-origin-rpy-1":{"scale":np.pi * 2},
            "joint-origin-rpy-2":{"scale":np.pi * 2},
            "joint-origin-rpy-3":{"scale":np.pi * 2},
            "joint-origin-xyz-1":{"scale":1},
            "joint-origin-xyz-2":{"scale":1},
            "joint-origin-xyz-3":{"scale":1},
            "control-waveform":{"scale":1},
            "control-amp":{"scale":0.25},
            "control-freq":{"scale":1}
            }
        ind = 0
        for key in gene_spec.keys():
            gene_spec[key]["ind"] = ind
            ind = ind + 1
        return gene_spec

    @staticmethod
    def get_gene_dict2(gene, spec):
        gdict = {}
        for key in spec:
            ind = spec[key]["ind"]
            scale = spec[key]["scale"]
            gdict[key] = gene[ind] * scale
        return gdict

    @staticmethod
    def get_genome_dicts2(genome, spec):
        gdicts = []
        for gene in genome:
            gdicts.append(Genome_with_fixed_body.get_gene_dict2(gene, spec))
        return gdicts
  
    @staticmethod
    def get_Parent_Link(link_name="0"):
        link = genome.URDFLink(name=link_name, 
                parent_name=link_name, 
                recur=0, #recur+1, 
                link_length=1.0, #fixed gdict["link-length"], 
                link_radius=0.1, #fixed gdict["link-radius"], 
                link_mass=1.4, #fixed gdict["link-mass"],
                joint_type=2, #gdict["joint-type"],
                joint_parent=0, #fixed gdict["joint-parent"],
                joint_axis_xyz=2, #gdict["joint-axis-xyz"],
                joint_origin_rpy_1=2, #gdict["joint-origin-rpy-1"],
                joint_origin_rpy_2=2, #gdict["joint-origin-rpy-2"],
                joint_origin_rpy_3=2, #gdict["joint-origin-rpy-3"],
                joint_origin_xyz_1=2, #gdict["joint-origin-xyz-1"],
                joint_origin_xyz_2=2, #gdict["joint-origin-xyz-2"],
                joint_origin_xyz_3=2, #gdict["joint-origin-xyz-3"],
                control_waveform=2, #gdict["control-waveform"],
                control_amp=2, #gdict["control-amp"],
                control_freq=2 #gdict["control-freq"])
        )
        return link
 
    @staticmethod
    def genome_to_flat_links2(gdicts):
        links = []
        link_ind = 0
        parent_names = [str(link_ind)]
        links.append(Genome_with_fixed_body.get_Parent_Link())
        link_ind = link_ind + 1
        for gdict in gdicts:
            link_name = str(link_ind)
            parent_ind = 0 #gdict["joint-parent"] * len(parent_names)
            assert parent_ind < len(parent_names), "genome.py: parent ind too high: " + str(parent_ind) + "got: " + str(parent_names)
            parent_name = parent_names[int(parent_ind)]
            #print("available parents: ", parent_names, "chose", parent_name)
            recur = 2 #gdict["link-recurrence"]
            link = genome.URDFLink(name=link_name, 
                            parent_name=parent_name, 
                            recur=recur+1, 
                            link_length=0.01, #fixed gdict["link-length"], 
                            link_radius=0.30, #fixed gdict["link-radius"], 
                            link_mass=0.2, #fixed gdict["link-mass"],
                            joint_type=gdict["joint-type"],
                            joint_parent=0, #fixed gdict["joint-parent"],
                            joint_axis_xyz=gdict["joint-axis-xyz"],
                            joint_origin_rpy_1=gdict["joint-origin-rpy-1"],
                            joint_origin_rpy_2=gdict["joint-origin-rpy-2"],
                            joint_origin_rpy_3=gdict["joint-origin-rpy-3"],
                            joint_origin_xyz_1=gdict["joint-origin-xyz-1"],
                            joint_origin_xyz_2=gdict["joint-origin-xyz-2"],
                            joint_origin_xyz_3=gdict["joint-origin-xyz-3"],
                            control_waveform=gdict["control-waveform"],
                            control_amp=gdict["control-amp"],
                            control_freq=gdict["control-freq"])
            links.append(link)
            if link_ind != 0:# don't re-add the first link
                parent_names.append(link_name)
            link_ind = link_ind + 1
  
        # now just fix the first link so it links to nothing
        links[0].parent_name = "None"
        return links

    @staticmethod
    def expandLinks2(parent_link, uniq_parent_name, flat_links, exp_links):
        children = [l for l in flat_links if l.parent_name == parent_link.name]
        sibling_ind = 1
        for c in children:
            for r in range(int(c.recur)):
                sibling_ind  = sibling_ind +1
                c_copy = copy.copy(c)
                c_copy.parent_name = uniq_parent_name
                uniq_name = c_copy.name + str(len(exp_links))
                #print("exp: ", c.name, " -> ", uniq_name)
                c_copy.name = uniq_name
                c_copy.sibling_ind = sibling_ind
                exp_links.append(c_copy)
                assert c.parent_name != c.name, "Genome::expandLinks: link joined to itself: " + c.name + " joins " + c.parent_name 
                Genome.expandLinks(c, uniq_name, flat_links, exp_links)



    @staticmethod
    def crossover2(g1, g2):
        x1 = random.randint(0, len(g1)-1)
        x2 = random.randint(0, len(g2)-1)
        g3 = np.concatenate((g1[x1:], g2[x2:])) 
        if len(g3) > len(g1):
            g3 = g3[0:len(g1)] 
        return g3

    @staticmethod
    def point_mutate2(genome, rate, amount):
        new_genome = copy.copy(genome)
        for gene in new_genome:
            for i in range(len(gene)):
                if random.random() < rate:
                    gene[i] += 0.1
                if gene[i] >= 1.0:
                    gene[i] = 0.9999
                if gene[i] < 0.0:
                    gene[i] = 0.0
        return new_genome

    @staticmethod
    def shrink_mutate2(genome, rate):
        if len(genome) == 1:
            return copy.copy(genome)
        if random.random() < rate:
            ind = random.randint(0, len(genome)-1)
            new_genome = np.delete(genome, ind, 0)
            return new_genome
        else:
            return copy.copy(genome)

    @staticmethod
    def grow_mutate2(genome, rate):
        if random.random() < rate:
            gene = Genome_with_fixed_body.get_random_gene2(len(genome[0]))
            new_genome = copy.copy(genome)
            new_genome = np.append(new_genome, [gene], axis=0)
            return new_genome
        else:
            return copy.copy(genome)


    @staticmethod
    def to_csv2(dna, csv_file):
        csv_str = ""
        for gene in dna:
            for val in gene:
                csv_str = csv_str + str(val) + ","
            csv_str = csv_str + '\n'

        with open(csv_file, 'w') as f:
            f.write(csv_str)

    @staticmethod
    def from_csv2(filename):
        csv_str = ''
        with open(filename) as f:
            csv_str = f.read()   
        dna = []
        lines = csv_str.split('\n')
        for line in lines:
            vals = line.split(',')
            gene = [float(v) for v in vals if v != '']
            if len(gene) > 0:
                dna.append(gene)
        return dna

