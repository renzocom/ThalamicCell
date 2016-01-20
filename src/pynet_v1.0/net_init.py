from netpyne import params, init

cellRule = {'label': 'PYR_HH_rule', 'conditions': {'cellType': 'PYR', 'cellModel': 'HH'},  'sections': {}}      # cell rule dict
cellRule['sections'] = params.utils.importCell(fileName='TC_CellFile.py', cellName='TC_cell')
netParams = {}

## Population parameters
netParams['popParams'] = []  # list of populations - each item will contain dict with pop params
netParams['popParams'].append({'popLabel': 'HH_pop', 'cellType': 'PYR', 'numCells': 100, 'cellModel': 'HH'}) 
netParams['popParams'].append({'popLabel': 'background', 'rate': 30, 'noise': 0.5, 'source': 'random', 'cellModel': 'NetStim'})

netParams['cellParams'] = [] # list of cell property rules - each item will contain dict with cell properties
netParams['cellParams'].append(cellRule)

## Cell connectivity rules
netParams['connParams'] = []  

'''
netParams['connParams'].append({'preTags': {'popLabel': 'HH_pop'}, 'postTags': {'popLabel': 'HH_pop'},  #  HH -> HH
	'connFunc': 'randConn', 	# connectivity function (random)
	'maxConns': 10, 			# max number of incoming conns to cell
	'weight': 0.01, 			# synaptic weight 
	'delay': 5,					# transmission delay (ms) 
	'sec': 'soma'})				# section to connect to
'''
netParams['connParams'].append({
	'preTags': {'popLabel': 'background'}, 'postTags': {'cellType': 'PYR', 'cellModel': ['HH']}, # background -> PYR (weight=0.1)
	'connFunc': 'fullConn', 	# connectivity function (all-to-all)
	'weight': 0.1, 			# synaptic weight 
	'delay': 5,					# transmission delay (ms) 
	'sec': 'soma'})		

# Simulation options
simConfig = {}
simConfig['duration'] = 9*1e3 			# Duration of the simulation, in ms
simConfig['dt'] = 0.025 				# Internal integration timestep to use
simConfig['verbose'] = True			# Show detailed messages 
simConfig['recordTraces'] = True  		# Record cell traces or not
simConfig['recdict'] = {'V_soma':{'sec':'soma','pos':0.5,'var':'v'}}  # Dict with traces to record
simConfig['recordStep'] = 1 			# Step size in ms to save data (eg. V traces, LFP, etc)
simConfig['filename'] = 'model_output'  # Set file output name
simConfig['savePickle'] = False 		# Save params, network and sim output to pickle file
simConfig['saveTxt'] = True 		# Save params, network and sim output to pickle file
simConfig['plotRaster'] = True 			# Plot a raster
simConfig['plotTracesGids'] = [1] 		# Plot recorded traces for this list of cells


# Create network and run simulation
init.createAndSimulate(netParams = netParams, simConfig = simConfig)    
   
# import pylab; pylab.show()  # this line is only necessary in certain systems where figures appear empty
