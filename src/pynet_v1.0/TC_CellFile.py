from matplotlib import pyplot
import random
from datetime import datetime
import pickle
from neuron import h, gui


class Cell(object):
    def __init__(self):
        self.synlist = []
        self.createSections()
        self.buildTopology()
        self.defineGeometry()
        self.defineBiophysics()
        self.createSynapses()
        self.nclist = []

    def createSections(self):
        pass

    def buildTopology(self):
        pass

    def defineGeometry(self):
        pass

    def defineBiophysics(self):
        pass

    def createSynapses(self):
        """Add an exponentially decaying synapse """
        synsoma = h.ExpSyn(self.soma(0.5))
        synsoma.tau = 2
        synsoma.e = 0
        self.synlist.append(synsoma) # synlist is defined in Cell


    def createNetcon(self, thresh=10):
        """ created netcon to record spikes """
        nc = h.NetCon(self.soma(0.5)._ref_v, None, sec = self.soma)
        nc.threshold = thresh
        return nc


class TC_cell(Cell): 
    """HH cell: A soma with active channels""" 
    def createSections(self):
        """Create the sections of the cell."""
        self.soma = h.Section(name='soma', cell=self)

    def defineGeometry(self):
        """Set the 3D geometry of the cell."""
        self.soma.L = 100
        self.soma.diam = 76.58
        self.soma.Ra = 173
        self.soma.cm = 1
    
    def defineBiophysics(self):
        """Assign the membrane properties across the cell."""
        
        self.soma.cm = 1
        G_pas = 3.79e-5
        E_pas = -76.5
        h.celsius = 34
        
        # Insert passive current in the soma
        self.soma.insert('pas')
        self.soma.g_pas = G_pas
        self.soma.e_pas = E_pas
        self.soma.cm = 0.88
        
        # Insert active Hodgkin-Huxley current in the soma
        self.soma.insert('hh2')
        self.soma.ena = 50
        self.soma.ek = -100
        self.soma.gnabar_hh2 = 0.01  # Sodium conductance in S/cm2
        self.soma.gkbar_hh2 = 0.01  # Potassium conductance in S/cm2
        self.soma.vtraub_hh2 = -52
        
        self.soma.insert('itGHK')	#T-current
        self.soma.cai = 2.4e-4
        self.soma.cao = 2
        self.soma.eca = 120
        self.soma.shift_itGHK = -1
        self.soma.pcabar_itGHK = 0.0002
        
        self.soma.insert('htc')
        self.soma.gmax_htc = 8e-7




