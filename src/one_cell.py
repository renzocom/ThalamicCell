from TC_CellFile import *
from matplotlib import pyplot
import numpy as np
from pylab import * # for drawing

cell = TC_cell()	#creating an object of TC_cell class
cell.soma.gmax_htc = 5e-5	#maximum conductance of Ih 
cell.soma.pcabar_itGHK = 0.0002

vrec = h.Vector()
trec = h.Vector() 
vrec.record(cell.soma(0.5)._ref_v) # record voltage from center
trec.record(h._ref_t) # record time

# insert a current clamp with continuously fluctuating value
# inject a continues current
h.tstop = 9e3
h.dt = 0.025
AMP = 0.5 # scales amplitude of current clamp
np.random.seed()
noise = np.random.rand(h.tstop/h.dt+1)*AMP
icc = h.IClamp(cell.soma(0.5))
icc.delay=0		#start current clamp right away
icc.dur=h.tstop 	#this means the value is played into for simulation duration
nvec = h.Vector( noise )
nvec.play(icc._ref_amp,h.dt)	#updates icc's amplitude during the simulation using nvec's values


# Hyperpolarizing current pulse
AMP2 = -0.5 # scales amplitude of current clamp, 4.5 produces sub-threshold response, 20 causes soma spikes normally, 
         # change values to see effect
np.random.seed()
#noise = np.random.rand(h.tstop/h.dt+1)*AMP
icc2 = h.IClamp(cell.soma(0.5))
icc2.delay=3000 # start current clamp right away
icc2.dur=3000 # this means the value is played into for simulation duration
icc2.amp=AMP2


h.v_init = -70 # this is initialization voltage used by h.run()
h.run() # run simulation for duration in h.tstop

# plot output
subplot(2,1,1)
plot(trec.as_numpy(),vrec.as_numpy()) # as_numpy is more efficient than converting/copying Vector to numpy format
xlim((0,h.tstop))
xlabel('Time (ms)')
ylabel('Vm (mV)')
tight_layout()

subplot(2,1,2)
plot(trec.as_numpy(),nvec.as_numpy()) # display part of the noisy current-injection amplitude
xlim((0,h.tstop))
xlabel('Time (ms)')
ylabel('i (nA)')
xlim((0,10))

show()



