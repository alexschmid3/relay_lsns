
using JuMP, Gurobi, Plots, Random, CSV, DataFrames, Statistics, Dates

#-----------------------------------LOAD OTHER FILES------------------------------------#

include("code/readrivigodata_online.jl")
include("code/shortestpath.jl")
include("code/shortestpath_multiple.jl")

include("code/bestnextstepheuristic.jl")

include("code/relay_online_iteration.jl")
include("code/relay_online_writeresults.jl")

include("code/lsns_enumeratesubproblems.jl")

#-------------------------------------FOUR INSTANCES------------------------------------#  

ordercapslist = [3, 6, 18, 10000]			#Cap on the number of orders introduced in each 6 hour increment in the online problem (doesn't depend on tstep or timedelta to ensure consistent instance when these parameters are adjusted)			
loclist = [20, 40, 60, 66]
driverlist = [70, 300, 1600, 3316]
trucklist = [60, 230, 1200, 2495]
seedlist = [202481, 155702, 731761, 963189]
weekstartlist = [DateTime(2019, 7, 21, 8), DateTime(2019, 7, 14, 8), DateTime(2019, 7, 7, 8), DateTime(2019, 6, 30, 8)]

#----------------------------------INSTANCE PARAMETERS----------------------------------#  	

#Select instance to run (see descriptions of instances above)
ex = 4

#Select a solution method 
# "abcg" = arc-based col gen, "otd" = orders-then-drivers, "rr" = rivigo routes, "bns" = best next step
solutionmethod = "bns"

#Definition of the instance 
iterationordercap = ordercapslist[ex]					 
maxlocs = loclist[ex]
maxdrivers = driverlist[ex]								 
numtrucks = trucklist[ex]	
weekstart = DateTime(2019, 7, 21, 8) #weekstartlist[ex]
randomseedval = seedlist[ex]
Random.seed!(randomseedval)

#Instance/decomposition parameters
tstep = 6											# Time discretization in hours
horizon = 24*7										# Length of planning horion in hours
shiftlength = 12									# Length of each driver shift in hours
lambda = 500										# Objective function = lambda * delay + miles
taxicostpct = 2.0                          			# Cost to taxi along an arc as a percentage of cost to drive a truck along that arc (should be > 1.0)
roundup_flag = 1			 					 	# 0 = round down to find discretized travel times, 1 = round up
drivershifttstep = 12								# How many hours between start of driver shifts, (ex. drivershifttstep=12 means each driver's first shift starts at time 0 or time 12, drivershifttstep=6 means a driver's first shift could start at time 0, 6, 12, or 18)
inprogressdummyarc_flag = 0							# 1 = allow in progress orders to be assigned to the dummy arc, 0 = do not (Should be assigned to 0 to ensure feasibility/continuity of online iterations)

#Subproblem decomposition parameters
timeblocks = [24,36,48,60]
klist = [1,2,3]

#Create id for this run
runid = string("initialsoln_ex", ex, "_tstep", tstep, "_week", convert(Date, weekstart),  "_rundate", today())

#Travel time calculation parameters
excludeoutliers_flag = 1							# 0 = include outliers in travel time calculation, 1 = exclude outliers
googlemapstraveltimes_flag = 1						# 1 = travel time between two locations is max(Avg from Rivigo data, Google Maps travel time) (<5% of arcs use google maps time), 0 = travel time is avg of Rivigo data
includesymmetricarcs_flag = 1						# 1 = if arc A-->B present in Rivigo data but not B-->A, create synthetic arc B-->A; 0 = do not include synthetic arcs (may cause feasibility issues)
traveltimefordelay_flag = 2 						# 0 = use rounded travel times for shortest path used in delay objective, 1 = use raw travel times (best for comparing across multiple tsteps), 2 = use rounded travel times, except on the final leg of the journey where raw is used 

#Online relay input preparation parameters
becomesavailablehours = 24 							# Number of hours before start of pick up window that each order becomes visible for planning purposes

#Online relay design parameters
timedelta = 6									# Number of hours we move forward in each online iteration, must be a multiple of tstep
onlinetimehorizon = 24*7		  	    			# Length of the online time horizon in hours (ex. run 1 week of online iterations), should be multiple of timedelta
numiterations_online = convert(Int64, onlinetimehorizon/timedelta)		
finallegdistancepenalty = 0.40						# Distance penalty assessed for orders that finish beyond the planning horizon
finallegtimepenalty = 0.30							# Time/delay penalty assessed for orders that finish beyond the planning horizon
pruneorderarcs_flag = 1								# 1 = prune order arcs that are no longer usable after each online iteration, 0 = do not
dummyendtime = 1000									# Dummy time assigned to the "beyond the horizon" nodes

#ABCG algorithm control parameters
newreducedcost_flag = 0								# 1 = use z-variable reduced costs to bound arc reduced costs for the subproblem, 0 = do not
minarcspernode = 5
globalrcthreshold = 0
consolidatepitstopsindp_flag = 0
dp_pitstopclusterradius = 70
solvedpheuristically_flag = 0
pathsperiter = 1
adaptivepathsperiter_flag = 0
pathsperiter1, pathsitercap1 = 10, 20
pathsperiter2, pathsitercap2 = 5, 20
pathsperiter3 = 1
iptimelimit = 60*12									# Max run time of IP in minutes

#Visualization/output/print statements control parameters
printstatements = 0									# Turn on/off printing 			
writeresultsfile = 1								# 1 = create the main results file, 0 = do not
writeorderoutcomesfiles = 1							# 1 = create the order outcomes file to see details of specific orders at specific iterations, 0 = do not
writecorridorfile = 1								# 1 = create the corridor file to see metrics on which arcs were used most often, 0 = do not
maketimespacevizfiles = 0							# Create one time-space network visualization per online iteration
makespatialvizfiles = 0 						 	# Create one spatial network visualization per online iteration
makeadvancedvizfiles = 0							# Create order- and driver-specific files (LOTS of files)
darkestcolor, lightestcolor = 5, 90					# The darkest and lightest colors used on the spatial maps (0-100)
showdrivers_flag = 1								# 1 = show driver assignments on time-space network visualization, 0 = do not
maketimespacevizfiles = 0 

#File names					
vizfoldername = string("visualizations/lsns/run ", runid)
csvfoldername = string("outputs/lsns/")
vizfilename = string(solutionmethod, "_")											#Folder names + file extensions added later for viz files
resultsfilename = string(csvfoldername,  runid, "/", solutionmethod, "_output.csv")
orderoutcomesfilename = string(csvfoldername, runid, "/orderoutcomes.csv")
orderoutcomesbyiterfilename = string(csvfoldername, runid, "/orderoutcomes_iterationlevel.csv")
corridorfilename = string(csvfoldername, runid, "/corridorcounter.csv")
spcorridorfilename = string(csvfoldername, runid, "/shortestpathcorridorcounter.csv")
heuristicfigurefilename = string(csvfoldername, "figures/heuristiccomparison_", runid, ".csv")
computationaltimefigurefilename = string(csvfoldername, "figures/comptimes_", runid, ".csv")
fullxsolutionfilename = string(csvfoldername, runid, "/solution/x_soln")
fullysolutionfilename = string(csvfoldername, runid, "/solution/y_soln")
fullzsolutionfilename = string(csvfoldername, runid, "/solution/z_soln")
parametersfilename = string(csvfoldername, runid, "/parameters.csv")
subproblemdatafilename = string(csvfoldername, "subproblems/spdata_", runid, ".csv")
subproblemvizfilename = string(vizfoldername, "/subproblems/spviz_")

#----------------------------IMPORT VISUALIZATIONS IF NEEDED----------------------------# 

if maketimespacevizfiles + makespatialvizfiles + makeadvancedvizfiles >= 1
	include("code/relay_online_visualization.jl")
	include("code/relayvisualization_spatialmap.jl")
end
if maketimespacevizfiles >= 1
	include("code/lsns_visualization.jl")
end
#------------------------------CREATE FOLDERS FOR OUTPUTS-------------------------------# 

#Create output folders
if !(isdir("outputs"))
	mkdir("outputs")
	mkdir("outputs/lsns")
	mkdir("outputs/lsns/figures")
	mkdir("outputs/lsns/subproblems")
elseif !(isdir("outputs/lsns/"))
	mkdir("outputs/lsns/")
	mkdir("outputs/lsns/figures")
	mkdir("outputs/lsns/subproblems")
elseif !(isdir("outputs/lsns/figures"))
	mkdir("outputs/lsns/figures")
elseif !(isdir("outputs/lsns/subproblems"))
	mkdir("outputs/lsns/subproblems")
end

if !(isdir(string(csvfoldername, runid)))
	mkdir(string(csvfoldername, runid))
	mkdir(string(csvfoldername, runid, "/solution"))
elseif !(isdir(string(csvfoldername, runid, "/solution")))
	mkdir(string(csvfoldername, runid, "/solution"))
end

#Create visualization folders
if !(isdir("visualizations"))
	mkdir("visualizations")
	mkdir("visualizations/lsns")
	mkdir(vizfoldername)
	mkdir(string(vizfoldername, "/subproblems"))
elseif !(isdir("visualizations/lsns/"))
	mkdir("visualizations/lsns/")
	mkdir(vizfoldername)
	mkdir(string(vizfoldername, "/subproblems"))
elseif !(isdir(vizfoldername))
	mkdir(vizfoldername)
	mkdir(string(vizfoldername, "/subproblems"))
end

#Create subfolders
subfoldernames = []
if maketimespacevizfiles == 1
	push!(subfoldernames, "TimeSpaceNetworkMaps")
end
if makespatialvizfiles == 1
	push!(subfoldernames, "SpatialNetworkMaps")
end
if makeadvancedvizfiles == 1
	push!(subfoldernames, "DriverMaps")
	push!(subfoldernames, "SpatialOrderMaps")
	push!(subfoldernames, "OrderMaps")
end

for sf in subfoldernames
	fullfoldername = string(vizfoldername, "/", sf)
	if !(isdir(fullfoldername))
		mkdir(fullfoldername)
	end
end

#---------------------------------OUTPUT PARAMETERS FILE--------------------------------# 

writeparametersfile(parametersfilename)

#-----------------------------------HELPER FUNCTIONS------------------------------------# 

#Removing item from list
function remove!(a, item)
    deleteat!(a, findall(x->x==item, a))
end

#Print a description of arc a, format: "(startloc, starttime) ==> (endloc, endtime)"
function arcDesc(a)
	println(nodesLookup[arcLookup[a][1]], " ==> ", nodesLookup[arcLookup[a][2]])
end

#-----------------------------------GENERATE INSTANCE-----------------------------------# 

#Initialize currentdatetime
currentdatetime = weekstart
solvedpheuristically_flag_now = solvedpheuristically_flag

#====================================================#

#Create node and arc networks
timeperiods = horizon / tstep + 1
maxviztimeperiods = timeperiods + numiterations_online * (timedelta / tstep)
hubCoords, hubsLookup, hubsReverseLookup, hubsTravelTimeIndex, numlocs = readlocations("data/hub_data_isb_connect.csv", maxlocs)
nodes, nodesLookup, N_0, N_end, numnodes = timespacentwk(numlocs, tstep, horizon)
m_0, m_end = truckdistribution(numtrucks, numlocs, N_0, N_end)
prearcs, arcLength, arcLength_raw = readarcs("data/traveltimes_outliers.csv", "data/hubdistances.csv", tstep, numlocs, hubsTravelTimeIndex, roundup_flag, excludeoutliers_flag, hubsReverseLookup, googlemapstraveltimes_flag)
arcs, arcLookup, A_plus, A_minus, A_space, A_plus_time, A_minus_time, A_minus_space, A_plus_space, numarcs, truetraveltime = arccreation(prearcs, horizon, tstep, numnodes, nodes, numlocs)

#====================================================#

#Objective coefficients
c = readobjectivecosts("data/hubdistances.csv", numarcs, numlocs, hubsReverseLookup, nodesLookup, arcLookup)
u = Dict()
for a in 1:numarcs
	u[a] = taxicostpct*c[a]  
end

#Cluster pitstops by distance for DP
clusteredpitstops = clusterpitstops(prearcs, dp_pitstopclusterradius)

#====================================================#

#Initial orders
orderwindowstart, orderwindowend = weekstart, weekstart + Dates.Hour(becomesavailablehours)
includeorderidlist = generateorderlist("data/lh_data_isb_connect_clean.csv", "data/vnt_data_isb_connect_clean.csv", iterationordercap, numlocs)
numorders, originloc, destloc, available, duedate, usedorderidlist, psseq, orderOriginalStartLoc, ordersinprogress = pullorders_initrivigoroutes("data/lh_data_isb_connect_clean.csv", "data/vnt_data_isb_connect_clean.csv", 10000, orderwindowstart, orderwindowend, tstep, horizon, prearcs, numlocs, timedelta, includeorderidlist)
orders = [i for i in 1:numorders]
highestorderindex = numorders
Origin, Destination = formatorders(numorders, originloc, destloc, available, duedate, tstep)
N_flow_i = flowbalancenodesets_i(orders, numnodes, Origin, Destination)
orderOriginalStartTime = Dict()
for i in orders
	orderOriginalStartTime[i] = nodesLookup[Origin[i][1]][2]
end

orderintransit_flag = Dict()
for i in orders
	orderintransit_flag[i] = 0
end

#Reserve trucks for the in transit orders
placeholder, loctruckcounter, trucksintransit = Dict(), zeros(numlocs), []
for i in ordersinprogress
	currentloc, availtime = originloc[i][1], available[i][1]
	try
		placeholder[currentloc, availtime] += 1
	catch
		placeholder[currentloc, availtime] = 1
	end
end
for item in placeholder
	currentloc, availtime, ttltrucks = item[1][1], item[1][2], item[2]
	push!(trucksintransit, (currentloc, availtime, ttltrucks))
	loctruckcounter[currentloc] += ttltrucks
end

#Modify m_0 if needed to accomodate the trucks that are already in transit
#Note: THIS IS CHANGING THE INITIAL STATE OF THE INSTANCE
truckexcess = m_0 - loctruckcounter
for l in 1:numlocs
	if loctruckcounter[l] > m_0[l]
		addltrucks = loctruckcounter[l] - m_0[l]
		for j in 1:addltrucks
			extraindex = argmax(truckexcess)
			truckexcess[extraindex] -= 1
			m_0[extraindex] -= 1
			m_0[l] += 1
		end
	end
end

#Trouble orders: used an arc that is longer than driver shift, and thus can't be assigned to a driver in our model
rivigoorderroutes, troubleorders = Dict(), []

#Find the routes Rivigo used 
if solutionmethod == "rr"
	#Update pitstop sequence to the current order location
	for i in orders
		completedloc = findfirst(x->hubsReverseLookup[x]==orderOriginalStartLoc[i], psseq[i])
		psseq[i] = psseq[i][completedloc:length(psseq[i])]
	end

	#Format order routes from Rivigo data
	for i in orders
		rivigoorderroutes[i] = []
		ordoriginloc = orderOriginalStartLoc[i]
		for j in 1:length(psseq[i])-1
			#Collect order routes, formatted as (startloc, endloc, arctime)
			firstloc, nextloc = hubsReverseLookup[psseq[i][j]], hubsReverseLookup[psseq[i][j+1]]
			if firstloc != nextloc
				arctime = arcLength[firstloc, nextloc]
			else
				arctime = tstep
			end
			push!(rivigoorderroutes[i], (firstloc, nextloc, arctime))
			if (arctime > shiftlength) & !(i in troubleorders)
				push!(troubleorders, i)
			end
			if firstloc != ordoriginloc
				push!(rivigoorderroutes[i], (firstloc, firstloc, tstep))
			end
		end
	end
end

#====================================================#

#Driver information
driversintransit = []
drivers, driverStartNodes, driverEndNodes, driverHomeLocs, assignedDrivers = readdrivers("data/pilots.csv", maxdrivers, numlocs, nodes, horizon)
N_flow_t, N_flow_d = flowbalancenodesets_td(drivers, numnodes, driverStartNodes, N_0, N_end)
awaylastnight = [0 for d in drivers]

#Driver shifts
alltimeswithinview = onlinetimehorizon + horizon
T_off_Monday8am, T_off, drivershift, T_off_0, T_off_constr, numshifts = createdrivershifts(shiftlength, tstep, drivershifttstep)

T_off_Monday8am_0 = Dict()
for s in 1:numshifts
	T_off_Monday8am_0[s] = []
	for t in 0:tstep:onlinetimehorizon+tstep
		if (t in setdiff([t2 for t2 in 0:tstep:onlinetimehorizon], T_off_Monday8am[s])) & !(t-tstep in setdiff([t2 for t2 in 0:tstep:onlinetimehorizon], T_off_Monday8am[s]))
			push!(T_off_Monday8am_0[s], t)
		end
	end
end

#Identify feasible arcs for each driver
homeArcSet, homeArcSet_space, availableDrivers, A_plus_d, A_minus_d, closelocs = driverArcSetsByDriver_overnight(numlocs, numarcs, numnodes, prearcs, drivers, tstep, horizon, nodes, arcs, assignedDrivers, A_minus, A_plus, T_off, drivershift, driverHomeLocs, T_off_0, shiftlength)
A_hasdriver, yupperbound, A_hasdriver_space, A_plus_hd, A_minus_hd = yarcreduction(numarcs, availableDrivers, A_space, numnodes, A_plus, A_minus)

#====================================================#

#Find the time length of every arc ("cost" of each arc in the shortest path problem)
arccosts = []
for a in 1:numarcs
	push!(arccosts, nodesLookup[arcLookup[a][2]][2] - nodesLookup[arcLookup[a][1]][2] )
end

#Calculate shortest paths in miles between all pairs of locations
distbetweenlocs, shortestpatharclists = cacheShortestDistance(numlocs, prearcs)
traveltimebetweenlocs_rdd = cacheShortestTravelTimes(numlocs, prearcs, "rdd time")
traveltimebetweenlocs_raw = cacheShortestTravelTimes(numlocs, prearcs, "raw time")
traveltimebetweenlocs_llr = cacheShortestTravelTimes(numlocs, prearcs, "llr time")

#Calculate the shortest trip times for each order i 
shortesttriptimes = []
for i in orders
	#Using shortest path distances, ignoring driver availability
	if traveltimefordelay_flag == 0
		shortestpathtime = traveltimebetweenlocs_rdd[nodesLookup[Origin[i][1]][1], nodesLookup[Destination[i][1]][1]]
	elseif traveltimefordelay_flag == 1
		shortestpathtime = traveltimebetweenlocs_raw[nodesLookup[Origin[i][1]][1], nodesLookup[Destination[i][1]][1]]
	elseif traveltimefordelay_flag == 2
		shortestpathtime = traveltimebetweenlocs_llr[nodesLookup[Origin[i][1]][1], nodesLookup[Destination[i][1]][1]]
	end
	
	push!(shortesttriptimes, shortestpathtime)
end

#Find shortest trip corridors

#Add final legs for orders that are unfinished at the end of the horizon
extendednodes, extendednumnodes, extendedarcs, extendednumarcs = copy(nodes), copy(numnodes), copy(arcs), copy(numarcs)
for l in 1:numlocs
	nodesLookup[extendednumnodes + 1] = (l, dummyendtime)
	extendednodes[l, dummyendtime] = extendednumnodes + 1
	A_minus[extendednumnodes + 1], A_plus[extendednumnodes + 1] = [], []
	global extendednumnodes += 1
end
for l1 in 1:numlocs, l2 in 1:numlocs
	n1, n2 = extendednodes[l1, horizon], extendednodes[l2, dummyendtime]
	extendedarcs[n1,n2] = extendednumarcs + 1
	arcLookup[extendednumarcs + 1] = (n1, n2)
	push!(c, distbetweenlocs[l1,l2] * (1 + finallegdistancepenalty))
	push!(A_minus[n2], extendednumarcs + 1)
	push!(A_plus[n1], extendednumarcs + 1)
	global extendednumarcs += 1
end
for i in orders
	destinationlocation = nodesLookup[Destination[i][1]][1]
	push!(Destination[i], extendednodes[destinationlocation, dummyendtime])
end

#====================================================#

#Create dummy arc 
dummyarc = extendednumarcs + 1
push!(c, 9999999)
allarcs = extendednumarcs + 1

#Finish times of arcs
arcfinishtime = []
if traveltimefordelay_flag == 0
	for a in 1:numarcs
		push!(arcfinishtime, nodesLookup[arcLookup[a][2]][2])
	end
	for a in numarcs+1:extendednumarcs
		startloc, endloc = nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1]
		push!(arcfinishtime, horizon + (1 + finallegtimepenalty) * traveltimebetweenlocs_rdd[startloc, endloc] )
	end
elseif traveltimefordelay_flag >=1
	for a in 1:numarcs
		push!(arcfinishtime, nodesLookup[arcLookup[a][1]][2] + truetraveltime[a])
	end
	for a in numarcs+1:extendednumarcs
		startloc, endloc = nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1]
		push!(arcfinishtime, horizon + (1 + finallegtimepenalty) * traveltimebetweenlocs_llr[startloc, endloc] )
	end
end
push!(arcfinishtime, 1000)

#--------------------------------------INITIALIZE---------------------------------------#  

#Metrics for output file
totalordertrips, totalordermiles, totalemptytrips, totaltaxitrips, totalemptymiles, totaltaximiles = 0, 0, 0, 0, 0, 0
total_delay, max_delay = 0, 0
#total_deviation_pct_from_shortestpath, max_deviation_pct_from_shortestpath = 0, 0
totalpastcost = 0
pastordersegments, pastdriversegments, pastdriversegments_space, pastemptysegments, pasttaxisegments = [], [], [], [], []
newordersbyiter = Dict()
newordersbyiter[0] = length(orders)
x_ip, y_ip, z_ip, w_ip, ordtime_ip = [],[],[],[],[]
ordermilesoutcomes, orderdelayoutcomes, ordermilespenalty, orderdelaypenalty = zeros(1200), zeros(1200), zeros(1200), zeros(1200)
totalawaynights, totaldriverhours = 0, 0
totalcgiterlist, iptimelistlist, rmptimelist, cgpptimelist, cgpptimeparallellist, orderarccountlist = [], [], [], [], [], []
drivernightshome, drivernightsaway = [convert(Int64, onlinetimehorizon/24) for d in drivers], [0 for d in drivers]

#Best next step specific
awayfromhome = []
endofcurrshift, duehome = [0 for d in drivers], [0 for d in drivers]
availabletrucks = Dict()
for n in 1:numnodes
	if n in N_0
		availabletrucks[n] = m_0[n]
	else	
		availabletrucks[n] = 0.0
	end
end

#---------------------------------------MAIN LOOP---------------------------------------# 

#Online iterations
for currtime in 0:timedelta:timedelta*(numiterations_online-1)

	println("------------------------------------BEGIN ITERATION CURRTIME = $currtime------------------------------------")

	#Re-order orders list to have in progress orders listed first (i.e. prioritized)
	global orders = union(intersect(orders, ordersinprogress), setdiff(orders, ordersinprogress))

	#Assign each order to its "best" next step
	officialassignments, orderassignments, driverassignments = heuristicassignments(traveltimefordelay_flag)

	#Write results to csv
	if writeresultsfile == 1
		writeresults_bns(resultsfilename, currtime, "heur")		
	end

	#Update datetime
	global currentdatetime = currentdatetime + Dates.Hour(timedelta)

	#Reset index sets
	global driversintransit = []
	global trucksintransit = []
	global T_off = []
	global T_off_0 = Dict()
	global T_off_constr = Dict()
	global A_hasdriver = []
	global yupperbound = []
	global A_hasdriver_space = []

	#Iterate forward by timedelta 
	updatedrivers_bns(timedelta, currentdatetime, weekstart, T_off_Monday8am, driverassignments)
	updateawaylastnight_bns(driverStartNodes)
	updatetrucks_bns(timedelta)
	driverArcSets_online_bns()
	updateorders_bns(timedelta, currentdatetime, orderassignments)

	#If it's not the last iteration, get next orders
	#If it is the last iteration, print final reports
	if currtime != timedelta*(numiterations_online-1)

		getnextorders(timedelta, currentdatetime, orders, "data/lh_data_isb_connect_clean.csv", "data/vnt_data_isb_connect_clean.csv")
			
	elseif currtime == timedelta*(numiterations_online-1)

		newordersbyiter[currtime] = 0

		#Assess delay and miles penalties for incomplete orders
		assessendofhorizonpenalties(currtime)

		#Write final output files
		if (writeresultsfile == 1) & (solutionmethod == "abcg")
			writeresults_final_online(resultsfilename, currtime)
		elseif (writeresultsfile == 1) & (solutionmethod == "otd")
			writeresults_final_online(resultsfilename, currtime)
		elseif (writeresultsfile == 1) & (solutionmethod == "rr")
			writeresults_final_online(resultsfilename, currtime)
		elseif (writeresultsfile == 1) & (solutionmethod == "bns")
			writeresults_bns(resultsfilename, currtime, "final")	
		end

		#Corridor file = number of trips between every pair of locations
		if writecorridorfile == 1
			writecorridorcounterfile(solutionmethod)
		end

		#Order outcome file = delivery time and distance for each order
		if writeorderoutcomesfiles == 1
			writeorderoutcomesfile(orderoutcomesfilename, solutionmethod, currtime)
		end

		if makespatialvizfiles == 1
			spatialviz_shortestpath(string(vizfoldername, "/SpatialNetworkMaps/", vizfilename, currtime, "_shortestpaths.png"), darkestcolor, lightestcolor)
		end

		#Write file for figures = heuristic comparison
		writefiguresfile(heuristicfigurefilename)

		#Save full solution
		savefullsolution_bns(fullxsolutionfilename, fullysolutionfilename, fullzsolutionfilename, currtime)

	end
	
end
				
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

#Restore initial solution
orders = [i for i in 1:highestorderindex if orderOriginalStartTime[i] < horizon]

Origin, Destination = Dict(), Dict()
for i in orders
	Origin[i] = []
	Destination[i] = []
	for t in orderOriginalStartTime[i]:tstep:horizon
		push!(Origin[i], nodes[originloc[i], t])
	end		
	for t in orderOriginalStartTime[i]+traveltimebetweenlocs_rdd[originloc[i],destloc[i]]:tstep:horizon
		push!(Destination[i], nodes[destloc[i], t])
	end
	push!(Destination[i], extendednodes[destloc[i], dummyendtime])
end

orderArcSet, orderArcSet_space, A_plus_i, A_minus_i = orderarcreduction(prearcs, shortesttriptimes)
homeArcSet, homeArcSet_space, availableDrivers, A_plus_d, A_minus_d, closelocs = restoredriverarcsets()
A_hasdriver, yupperbound, A_hasdriver_space, A_plus_hd, A_minus_hd = yarcreduction(numarcs, availableDrivers, A_space, numnodes, A_plus, A_minus)

#include("code/lsns_visualization.jl")
#include("code/lsns_enumeratesubproblems.jl")

#Get initial feasible solution
x_currsol, y_currsol, z_currsol, w_currsol, ordtime_currsol, x_currpath, z_currpath = getinitialsolution(pastordersegments, pastdriversegments, pasttaxisegments, pastemptysegments, orderdelayoutcomes)

#Add empty truck segments
bigset, bigset2 = [], []
for i in orders, a in x_currpath[i]
	global bigset = union(bigset, arcLookup[a][1])
end
for i in orders, a in x_currpath[i]
	global bigset2 = union(bigset2, arcLookup[a][2])
end
for n in N_0, homearc in setdiff(A_plus_hd[n], A_space)
	if n in bigset
		y_currsol[homearc] += m_0[n] - sum(sum(x_currsol[i,a] for a in A_plus_i[i,n]) for i in orders if n in Origin[i]) - sum(y_currsol[a] for a in A_plus_hd[n])
	else
		y_currsol[homearc] += m_0[n] - sum(y_currsol[a] for a in A_plus_hd[n])
	end
end
for n in setdiff([n2 for n2 in 1:numnodes], union(N_0, N_end))
	homearc_plus = setdiff(A_plus_hd[n], A_space)[1]
	if (n in bigset) & (n in bigset2)
		adjust = sum(sum(x_currsol[i,a] for a in A_plus_i[i,n]) for i in orders if A_plus_i[i,n] != []) + sum(y_currsol[a] for a in A_plus_hd[n]) - sum(sum(x_currsol[i,a] for a in A_minus_i[i,n]) for i in orders if A_minus_i[i,n] != []) - sum(y_currsol[a] for a in A_minus_hd[n])
		y_currsol[homearc_plus] += max(-1*adjust, 0)
	elseif (n in bigset) & !(n in bigset2)
		adjust = sum(sum(x_currsol[i,a] for a in A_plus_i[i,n]) for i in orders if A_plus_i[i,n] != []) + sum(y_currsol[a] for a in A_plus_hd[n]) - sum(y_currsol[a] for a in A_minus_hd[n])
		y_currsol[homearc_plus] += max(-1*adjust, 0)
	elseif !(n in bigset) & (n in bigset2)
		adjust = sum(y_currsol[a] for a in A_plus_hd[n]) - sum(sum(x_currsol[i,a] for a in A_minus_i[i,n]) for i in orders if A_minus_i[i,n] != []) - sum(y_currsol[a] for a in A_minus_hd[n])
		y_currsol[homearc_plus] += max(-1*adjust, 0)
	else
		adjust = sum(y_currsol[a] for a in A_plus_hd[n]) - sum(y_currsol[a] for a in A_minus_hd[n])
		y_currsol[homearc_plus] += max(-1*adjust, 0)
	end
end

#Construct subproblems of various sizes
locationgroups, locgroupid, locgroupidLookup = findlocationgroups_adjacency(klist)
subproblemlist = enumeratesubproblems(timeblocks, locationgroups)
overlappingsubprobs, adjacentsubprobs = findsubproblemoverlap(subproblemlist, overlappct, adjacentpct)
totalreoptimizations = zeros(length(subproblemlist))
overlappingreoptimizations = zeros(length(subproblemlist))
adjacentreoptimizations = zeros(length(subproblemlist))

#Evaluate each subproblem
improvementlist, improvementpctlist, solvetimelist, removelist = [], [], [], []
timewindowlist, numlocslist, numorderslist = [], [], []
counter = 0

#Initialize dataframe and features
feature_df = createdataframe()
historicaltraffic = calculatehistoricalarctraffic()
currenttraffic = calculatecurrentarctraffic()

for subprobindex in 1:length(subproblemlist)
	subprob = subproblemlist[subprobindex]
	println(subprob)
	
	subprobNodeSet, subprobArcSet, sp_times, sp_containsextended, timeblockArcSet = maptosubproblem_ntwk(subprob)
	#sp_orders, sp_compl_orders, sp_ordersinprogress, sp_Origin, sp_Destination, outsideorderdemand = maptosubproblem_orders(subprobArcSet, timeblockArcSet, x_currpath, sp_times)
	sp_orders, sp_compl_orders, sp_ordersinprogress, sp_Origin, sp_Destination, outsideorderdemand, sp_ordersupply, sp_begin_orders = maptosubproblem_orders_inandout(subprobArcSet, subprobNodeSet, timeblockArcSet, x_currpath, x_currsol, sp_times)
	if sp_orders == []
		push!(removelist, subprob)
	else
		sp_trucksupply = maptosubproblem_trucks(sp_orders, subprobNodeSet, subprobArcSet, timeblockArcSet, x_currsol, y_currsol)
		sp_drivers, sp_driversupply, sp_drivers_home = maptosubproblem_drivers(subprob, z_currpath, timeblockArcSet, subprobNodeSet, subprobArcSet)
	
		x_sp, y_sp, z_sp, w_sp, newobj_sp, oldobj_sp, solvetime_sp, ordermiles_sp, emptymiles_sp, taximiles_sp = solvesubproblem_lsns(subprobArcSet, subprobNodeSet, sp_orders, sp_drivers, sp_times, sp_compl_orders, sp_ordersinprogress, sp_Origin, sp_Destination, sp_containsextended, sp_trucksupply, sp_driversupply, x_currsol, y_currsol, z_currsol, w_currsol, ordtime_currsol, outsideorderdemand, sp_drivers_home, sp_ordersupply, sp_begin_orders)

		#vizfilename = string(subproblemvizfilename, counter)

		#visualizesubproblem(vizfilename, subprob, "orig", subprobArcSet, sp_orders, x_currsol, y_currsol, w_currsol, sp_drivers, z_currsol, showdrivers_flag, outsideorderdemand, oldobj_sp, 0)
		#visualizesubproblem(vizfilename, subprob, "reopt", subprobArcSet, sp_orders, getvalue.(x_sp), getvalue.(y_sp), getvalue.(w_sp), sp_drivers, getvalue.(z_sp), showdrivers_flag, outsideorderdemand, newobj_sp, max(0, (oldobj_sp - newobj_sp) /oldobj_sp))

		if newobj_sp < 10000000 
			addsubproblemfeatures(counter, subprobindex, subprob, sp_orders, sp_drivers, subprobNodeSet, subprobArcSet, locgroupidLookup, ordermiles_sp, emptymiles_sp, taximiles_sp, historicaltraffic, currenttraffic, oldobj_sp, newobj_sp, solvetime_sp)
		end

		global counter += 1
		println("------", counter, "------")
	end
end

CSV.write(subproblemdatafilename, feature_df)

for item in removelist
	remove!(subproblemlist, item)
end
