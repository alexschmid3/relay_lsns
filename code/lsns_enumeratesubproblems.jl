
using NearestNeighbors 

#-------------------------------------------------------------------------#

function remove!(a, item)
    deleteat!(a, findall(x->x==item, a))
end

#-------------------------------------------------------------------------#

function restoredriverarcsets()

	traveltime = cacheShortestTravelTimes(numlocs, prearcs, "rdd time")
	homeArcSet, homeArcSet_space, availableDrivers, A_plus_d, A_minus_d = Dict(), Dict(), Dict(), Dict(), Dict()
	fullhourlist = [t for t in 0:tstep:horizon-tstep]

	for d in drivers
		homeArcSet[d] = []
		homeArcSet_space[d] = []
	end
	for a in 1:numarcs
		availableDrivers[a] = []
	end
	for d in drivers, n in 1:numnodes
		A_plus_d[d,n] = []
		A_minus_d[d,n] = []
	end

	prearcs_aug = deepcopy(prearcs)
	for l in 1:numlocs
		push!(prearcs_aug, (l, l, tstep, tstep))
	end

	closelocs = Dict()
	for d in drivers
		closelocs[d] = []
	end

	for d in drivers, arc in prearcs_aug
		#Travel times between all relevant locations
		orig, dest = arc[1], arc[2]
		h = driverHomeLocs[d]
		t1 = traveltime[h, orig]
		t2 = arc[3]
		t3 = traveltime[dest, h]

		#Check earliest we can get to the arc origin location from our current location
		driverstartingloc, driverstartingtime = driverHomeLocs[d], 0
		firstshifttime = setdiff(fullhourlist, T_off[drivershift[d]])[1]
		drivinghourstoarcorig = traveltime[driverstartingloc, orig]
		totalhourstoarcorig = floor(drivinghourstoarcorig/shiftlength) * (24 - shiftlength) + drivinghourstoarcorig
		earliesttime = totalhourstoarcorig + max(firstshifttime, driverstartingtime)

		#If the arc is feasible for the driver, add it to the driver's arc list
		if ((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength))
			if !(orig in closelocs[d])
				push!(closelocs[d], orig)
			end
			if !(dest in closelocs[d])
				push!(closelocs[d], dest)
			end

			for t in setdiff([t4 for t4 in fullhourlist if t4 >= earliesttime], T_off[drivershift[d]])
				#Arc must finish before the end of the horizon and before the "next" off hour of the driver
				if t + t2 <= [t4 for t4 in union(T_off[drivershift[d]], horizon) if t4 > t][1]
					push!(homeArcSet[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
					if orig != dest
						push!(homeArcSet_space[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
					end				
					push!(availableDrivers[arcs[nodes[orig, t], nodes[dest, t + t2]]], d)
				end
			end
		end
	end

	#Add stay-at-home arcs for driver's off hours
	for d in drivers, t in setdiff(T_off[drivershift[d]], [horizon]), l in closelocs[d]
		push!(homeArcSet[d], arcs[nodes[l, t], nodes[l, t + tstep]])
		push!(availableDrivers[arcs[nodes[l, t], nodes[l, t + tstep]]], d)
	end

	#Create A_plus and A_minus lists
	for d in drivers, n in 1:numnodes, a in A_plus[n]
		if a in homeArcSet[d]
			push!(A_plus_d[d,n], a)
		end
	end
	for d in drivers, n in 1:numnodes, a in A_minus[n]
		if a in homeArcSet[d]
			push!(A_minus_d[d,n], a)
		end
	end

	return homeArcSet, homeArcSet_space, availableDrivers, A_plus_d, A_minus_d, closelocs

end

#-------------------------------------------------------------------------#

function getinitialsolution(pastordersegments, pastdriversegments, pasttaxisegments, pastemptysegments, orderdelayoutcomes)

	x_currsol, y_currsol, z_currsol, w_currsol, ordtime_currsol = Dict(), Dict(), Dict(), Dict(), Dict()
	x_currpath, z_currpath = Dict(), Dict()

	for i in orders, a in orderArcSet[i]
		x_currsol[i,a] = 0
	end
	for a in A_hasdriver
		y_currsol[a] = 0
	end
	for d in drivers, a in homeArcSet[d]
		z_currsol[d,a] = 0
	end
	for a in A_space
		w_currsol[a] = 0
	end
	for i in orders
		x_currpath[i] = []
	end
	for d in drivers
		z_currpath[d] = []
	end

	for item in pastordersegments
		i, st, et, sl, el = item
		a = extendedarcs[extendednodes[sl,st], extendednodes[el,et]]
		x_currsol[i,a] += 1
		push!(x_currpath[i], a)
	end
	for item in pastemptysegments
		numtrucks, st, et, sl, el = item
		a = extendedarcs[extendednodes[sl,st], extendednodes[el,et]]
		y_currsol[a] += numtrucks
	end
	for item in pastdriversegments
		d, st, et, sl, el = item
		a = extendedarcs[extendednodes[sl,st], extendednodes[el,et]]
		z_currsol[d,a] += 1
		push!(z_currpath[d], a)
	end
	for item in pasttaxisegments
		numtaxis, st, et, sl, el = item
		a = extendedarcs[extendednodes[sl,st], extendednodes[el,et]]
		w_currsol[a] += numtaxis
	end
	for i in orders
		ordtime_currsol[i] = orderdelayoutcomes[i]
	end

	return x_currsol, y_currsol, z_currsol, w_currsol, ordtime_currsol, x_currpath, z_currpath

end

#-------------------------------------------------------------------------#

#Enumerates all location clusters of size k (for k in locationgroupsizes), by centering one group at each location
function findlocationgroups_knn(locationgroupsizes)

	locationgroups = []

	kdtree = KDTree(transpose(hubCoords))

	for l in 1:numlocs, k in locationgroupsizes
		idxs, dists = knn(kdtree, hubCoords[l,:], k, true)
		if !(idxs in locationgroups) 
			push!(locationgroups, idxs)
		end
	end
	
	return locationgroups

end

#-------------------------------------------------------------------------#

#Enumerates all location clusters by finding the set of adjacent locations for each location
function findlocationgroups_adjacency(klist)

	seedlocationgroups = []
	locgroupid = Dict()
	locgroupidLookup = Dict()

	kdtree = KDTree(transpose(hubCoords[1:numlocs,:]))

	for k in klist
		for l in 1:numlocs
			idxs, dists = knn(kdtree, hubCoords[l,:], k, true)
			if !(idxs in seedlocationgroups) 
				push!(seedlocationgroups, idxs)
			end
		end
	end

	locationgroups = []
	locid = 1

	for corelocs in seedlocationgroups
		idxs = Set(copy(corelocs))
		for a in prearcs
			if (a[1] in corelocs) & !(a[2] in idxs)
				push!(idxs, a[2])
			end
		end
		if !(idxs in locationgroups) 
			push!(locationgroups, idxs)
			locgroupid[locid] = idxs
			locgroupidLookup[idxs] = locid
			locid += 1
		end
	end

	return locationgroups, locgroupid, locgroupidLookup

end

#-------------------------------------------------------------------------#

#Find a list of all possible subproblems and relevant descriptive information
function enumeratesubproblems(timeblocks, locationgroups)

	subproblemlist = []

	for blk in timeblocks, lg in locationgroups, t in 0:12:horizon-blk
		push!(subproblemlist, Dict("starttime" => t, "endtime" => t+blk, "locgroup" => lg))
	end
	for blk in timeblocks, lg in locationgroups
		t = horizon-blk+tstep
		push!(subproblemlist, Dict("starttime" => t, "endtime" => dummyendtime, "locgroup" => lg))
	end

	return subproblemlist

end

#-------------------------------------------------------------------------#

function findsubproblemoverlap(subproblemlist, overlappct, adjacentpct)

	overlappingsubprobs, adjacentsubprobs = Dict(), Dict()
	for sp in 1:length(subproblemlist)
		overlappingsubprobs[sp] = []
		adjacentsubprobs[sp] = []
	end

	for sp1 in 1:length(subproblemlist)
		subprob1 = subproblemlist[sp1]
		numnodes1 = (length(subprob1["locgroup"])) * ((min(subprob1["endtime"],horizon+tstep)-subprob1["starttime"])/tstep + 1)
		for sp2 in sp1+1:length(subproblemlist)
			subprob2 = subproblemlist[sp2]
			numnodes2 = (length(subprob2["locgroup"])) * ((min(subprob2["endtime"],horizon+tstep)-subprob2["starttime"])/tstep + 1)

			bothlocgroup = intersect(subprob1["locgroup"], subprob2["locgroup"])
			bothstarttime, bothendtime = max(subprob1["starttime"], subprob2["starttime"]), min(subprob1["endtime"], subprob2["endtime"], horizon+tstep)
			
			if (bothendtime > bothstarttime) & (bothlocgroup != [])
				numnodes12 = (length(bothlocgroup)) * ((bothendtime - bothstarttime)/tstep + 1)
				if (numnodes12/numnodes1 >= overlappct) || (numnodes12/numnodes2 >= overlappct)
					push!(overlappingsubprobs[sp1], sp2)
					push!(overlappingsubprobs[sp2], sp1)
				elseif (numnodes12/numnodes1 >= adjacentpct) || (numnodes12/numnodes2 >= adjacentpct)
					push!(adjacentsubprobs[sp1], sp2)
					push!(adjacentsubprobs[sp2], sp1)
				end
			end
		end
	end

	return overlappingsubprobs, adjacentsubprobs

end

#-------------------------------------------------------------------------#

function maptosubproblem_ntwk(subprob)

	subprobNodeSet, subprobArcSet, sp_times, timeblockArcSet = [], [], [], []

	#Subproblem times
	sp_times = [t for t in subprob["starttime"]:tstep:min(horizon,subprob["endtime"])]

	#Subproblem node set
	for l in subprob["locgroup"], t in sp_times
		push!(subprobNodeSet, nodes[l,t])
	end
	if subprob["endtime"] == dummyendtime
		for l in subprob["locgroup"]
			push!(subprobNodeSet, extendednodes[l,dummyendtime])
		end
		sp_containsextended = 1
		push!(sp_times, dummyendtime)
	else 
		sp_containsextended = 0
	end

	#Subproblem arc set
	for a in 1:extendednumarcs
		if (arcLookup[a][1] in subprobNodeSet) & (arcLookup[a][2] in subprobNodeSet)
			push!(subprobArcSet, a) 
		end
		if (nodesLookup[arcLookup[a][1]][2] in sp_times) & (nodesLookup[arcLookup[a][2]][2] in sp_times)
			push!(timeblockArcSet, a)
		end
	end

	return subprobNodeSet, subprobArcSet, sp_times, sp_containsextended, timeblockArcSet

end

#-------------------------------------------------------------------------#

function maptosubproblem_orders(subprobArcSet, timeblockArcSet, x_currpath, sp_times)

	excludearcs = setdiff(timeblockArcSet, subprobArcSet)
	
	#Find order list
	sp_orders, sp_ordersinprogress, sp_compl_orders = [], [], []
	sp_Origin, sp_Destination = Dict(), Dict()
	outsideorderdemand = Dict()
	for a in subprobArcSet
		outsideorderdemand[a] = 0
	end
	for i in orders
		if (intersect(x_currpath[i], excludearcs) == []) & (intersect(x_currpath[i], subprobArcSet) != [])
			push!(sp_orders, i)	
			sp_Origin[i] = []
			sp_Destination[i] = []
		end
	end

	for i in sp_orders
		initialloc, initialtime = nodesLookup[arcLookup[intersect(x_currpath[i], subprobArcSet)[1]][1]]
		finalloc, finaltime = nodesLookup[arcLookup[last(intersect(x_currpath[i], subprobArcSet))][2]]
		
		#Origins
		if initialloc != originloc[i]
			push!(sp_ordersinprogress, i)
			push!(sp_Origin[i], nodes[initialloc, initialtime])
		else
			for t in setdiff(sp_times, dummyendtime)
				push!(sp_Origin[i], nodes[initialloc, t])
			end
		end

		#Destinations
		if finalloc == destloc[i]
			push!(sp_compl_orders, i)
			for t in sp_times
				push!(sp_Destination[i], nodes[finalloc, t])
			end
		else
			push!(sp_Destination[i], nodes[finalloc, finaltime])
		end

	end

	for i in setdiff(orders, sp_orders), a in intersect(x_currpath[i], subprobArcSet)
		outsideorderdemand[a] += 1
	end

	return sp_orders, sp_compl_orders, sp_ordersinprogress, sp_Origin, sp_Destination, outsideorderdemand

end

#-------------------------------------------------------------------------#

function maptosubproblem_orders_inandout(subprobArcSet, subprobNodeSet, timeblockArcSet, x_currpath, x_currsol, sp_times)

	excludearcs = setdiff(timeblockArcSet, subprobArcSet)
	
	#Find order list
	sp_orders, sp_ordersinprogress, sp_compl_orders, sp_begin_orders = [], [], [], []
	sp_Origin, sp_Destination = Dict(), Dict()
	sp_ordersupply = Dict()
	outsideorderdemand = Dict()
	for a in subprobArcSet
		outsideorderdemand[a] = 0
	end
	for i in orders
		if (length(intersect(x_currpath[i], subprobArcSet)) >= 2) || ((intersect(x_currpath[i], excludearcs) == []) & (length(intersect(x_currpath[i], subprobArcSet)) >= 1)) 
			push!(sp_orders, i)	
			sp_Origin[i] = []
			sp_Destination[i] = []
		end
	end

	for i in sp_orders
		initialloc, initialtime = nodesLookup[arcLookup[intersect(x_currpath[i], subprobArcSet)[1]][1]]
		finalloc, finaltime = nodesLookup[arcLookup[last(intersect(x_currpath[i], subprobArcSet))][2]]
		
		#Origins
		if initialloc != originloc[i]
			push!(sp_ordersinprogress, i)
			#push!(sp_Origin[i], nodes[initialloc, initialtime])
		else
			push!(sp_begin_orders, i)
			for t in setdiff(sp_times, dummyendtime)
				push!(sp_Origin[i], nodes[initialloc, t])
			end
		end

		#Destinations
		if finalloc == destloc[i]
			push!(sp_compl_orders, i)
			for t in sp_times
				push!(sp_Destination[i], extendednodes[finalloc, t])
			end
		end

		#Order supply
		bigset, bigset2 = [], []
		for a in intersect(x_currpath[i], subprobArcSet) 
			bigset = union(bigset, arcLookup[a][1])
		end
		for a in intersect(x_currpath[i], subprobArcSet) 
			bigset2 = union(bigset2, arcLookup[a][2])
		end

		for n in subprobNodeSet
			if (n in bigset) & (n in bigset2) 
				sp_ordersupply[i,n] = sum(x_currsol[i,a] for a in intersect(subprobArcSet, A_minus_i[i,n])) - sum(x_currsol[i,a] for a in intersect(subprobArcSet, A_plus_i[i,n]))
			elseif !(n in bigset) & (n in bigset2) 
				sp_ordersupply[i,n] = sum(x_currsol[i,a] for a in intersect(subprobArcSet, A_minus_i[i,n])) 
			elseif (n in bigset) & !(n in bigset2) 
				sp_ordersupply[i,n] = - sum(x_currsol[i,a] for a in intersect(subprobArcSet, A_plus_i[i,n]))
			else
				sp_ordersupply[i,n] = 0
			end
		end

	end

	for i in setdiff(orders, sp_orders), a in intersect(x_currpath[i], subprobArcSet)
		outsideorderdemand[a] += 1
	end

	return sp_orders, sp_compl_orders, sp_ordersinprogress, sp_Origin, sp_Destination, outsideorderdemand, sp_ordersupply, sp_begin_orders

end

#-------------------------------------------------------------------------#

#Calculate the truck flow supply/demand at each node
function maptosubproblem_trucks(sp_orders, subprobNodeSet, subprobArcSet, timeblockArcSet, x_currsol, y_currsol)

	sp_trucksupply = Dict()

	ordersentering, ordersexiting, emptytrucksentering, emptytrucksexiting = Dict(), Dict(), Dict(), Dict()
	for n in subprobNodeSet
		ordersentering[n] = 0
		ordersexiting[n] = 0
		emptytrucksentering[n] = 0
		emptytrucksexiting[n] = 0
	end

	for i in sp_orders, a in intersect(timeblockArcSet, x_currpath[i])
		nstart, nend = arcLookup[a]
		if (nend in subprobNodeSet) & (nstart in subprobNodeSet)
			ordersentering[nend] += 1
			ordersexiting[nstart] += 1
		end
	end

	for n in setdiff(subprobNodeSet, union(N_0, [n2 for n2 in numnodes+1:extendednumnodes]))
		if intersect(timeblockArcSet, A_minus_hd[n]) != []
			emptytrucksentering[n] = sum(y_currsol[a] for a in intersect(subprobArcSet, A_minus_hd[n])) 
		end
	end
	for n in setdiff(subprobNodeSet,[n2 for n2 in numnodes+1:extendednumnodes])
		if intersect(timeblockArcSet, A_plus_hd[n]) != []
			emptytrucksexiting[n] = sum(y_currsol[a] for a in intersect(subprobArcSet, A_plus_hd[n])) 
		end
	end

	for n in subprobNodeSet
		sp_trucksupply[n] = ordersentering[n] + emptytrucksentering[n] - ordersexiting[n] - emptytrucksexiting[n]
	end

	#for n in subprobNodeSet
    #   println("$n ==> ", ordersentering[n], ", ", emptytrucksentering[n], ", ", ordersexiting[n], ", ", emptytrucksexiting[n])
    #end

	return sp_trucksupply

end

#-------------------------------------------------------------------------#

function maptosubproblem_drivers(subprob, z_currpath, timeblockArcSet, subprobNodeSet, subprobArcSet)

	sp_drivers, sp_drivers_home = [], []
	sp_driversupply = Dict()

	excludearcs = setdiff(timeblockArcSet, subprobArcSet)

	for d in drivers
		if intersect(z_currpath[d], subprobArcSet) != []
			push!(sp_drivers, d)	
		end
	end

	for d in sp_drivers

		if driverHomeLocs[d] in subprob["locgroup"]
			push!(sp_drivers_home, d)
		end

		bigset, bigset2 = [], []
		for a in intersect(z_currpath[d], subprobArcSet) 
			bigset = union(bigset, arcLookup[a][1])
		end
		for a in intersect(z_currpath[d], subprobArcSet) 
			bigset2 = union(bigset2, arcLookup[a][2])
		end

		for n in subprobNodeSet
			if (n in bigset) & (n in bigset2) 
				sp_driversupply[d,n] = sum(z_currsol[d,a] for a in intersect(subprobArcSet, A_minus_d[d,n])) - sum(z_currsol[d,a] for a in intersect(subprobArcSet, A_plus_d[d,n]))
			elseif !(n in bigset) & (n in bigset2) 
				sp_driversupply[d,n] = sum(z_currsol[d,a] for a in intersect(subprobArcSet, A_minus_d[d,n])) 
			elseif (n in bigset) & !(n in bigset2) 
				sp_driversupply[d,n] = - sum(z_currsol[d,a] for a in intersect(subprobArcSet, A_plus_d[d,n]))
			else
				sp_driversupply[d,n] = 0
			end
		end
  
	end

	return sp_drivers, sp_driversupply, sp_drivers_home

end

#-------------------------------------------------------------------------#

function solvesubproblem_lsns(subprobArcSet, subprobNodeSet, sp_orders, sp_drivers, sp_times, sp_compl_orders, sp_ordersinprogress, sp_Origin, sp_Destination, sp_containsextended, sp_trucksupply, sp_driversupply, x_currsol, y_currsol, z_currsol, w_currsol, ordtime_currsol, outsideorderdemand, sp_drivers_home, sp_ordersupply, sp_begin_orders)

	m = Model(Gurobi.Optimizer)
	set_optimizer_attribute(m, "TimeLimit", 60)
	set_optimizer_attribute(m, "OutputFlag", 0)

	#Variables
	@variable(m, x[i = sp_orders, intersect(orderArcSet[i], subprobArcSet)], Bin)
	@variable(m, y[intersect(A_hasdriver, subprobArcSet)] >= 0)
	@variable(m, z[d = sp_drivers, intersect(homeArcSet[d], subprobArcSet)], Bin) 
	@variable(m, w[a in intersect(subprobArcSet, A_space)] >= 0)
	@variable(m, ordtime[sp_compl_orders])

	#Objective
	@objective(m, Min, lambda * sum((ordtime[i] - shortesttriptimes[i])/shortesttriptimes[i] for i in sp_compl_orders) + sum(sum(c[a]*x[i,a] for a in intersect(orderArcSet[i], subprobArcSet)) for i in sp_orders) + sum(c[a]*y[a] for a in intersect(A_hasdriver, subprobArcSet)) + sum(u[a]*w[a] for a in intersect(subprobArcSet, A_space)) )
	
	#Order constraints
	@constraint(m, orderFlowBalance[i = sp_orders, n in setdiff([n2 for n2 in subprobNodeSet], union(sp_Origin[i], sp_Destination[i]))], sum(x[i,a] for a in intersect(subprobArcSet, A_minus_i[i,n])) - sum(x[i,a] for a in intersect(subprobArcSet, A_plus_i[i,n])) == sp_ordersupply[i,n])	
	@constraint(m, departOrigin[i = sp_begin_orders], sum(sum(x[i,a] for a in intersect(subprobArcSet, A_space, A_plus_i[i,n])) for n in sp_Origin[i]) == 1)
	@constraint(m, arriveDestin[i = sp_compl_orders], sum(sum(x[i,a] for a in intersect(subprobArcSet, A_minus_i[i,n])) for n in sp_Destination[i]) == 1)
	if sp_containsextended == 1
		for i in setdiff(sp_orders, sp_ordersinprogress)
			extendedorderarc = extendedarcs[last(Origin[i]), last(Destination[i])]
			if extendedorderarc in intersect(subprobArcSet, orderArcSet[i])
				set_normalized_coefficient(departOrigin[i], x[i,extendedorderarc], 1)
			end
		end
	end

	#Add in "stay where you are" arc for each in transit order
	#for i in intersect(sp_orders, sp_ordersinprogress), n in sp_Origin[i], a in setdiff(A_plus[n], union(A_space, dummyarc))
	#	if a in orderArcSet[i]
	#		set_normalized_coefficient(departOrigin[i], x[i,a], 1)
	#	end
	#end

	#Order delivery constraints
	@constraint(m, deliveryTime[i in sp_compl_orders], ordtime[i] - sum(sum(arcfinishtime[a] * x[i,a] for a in intersect(subprobArcSet,A_minus_i[i,n])) for n in sp_Destination[i]) == - orderOriginalStartTime[i])

	#Truck constraints
	@constraint(m, truckFlowBalance[n in setdiff(subprobNodeSet, [n2 for n2 in numnodes+1:extendednumnodes])], sum(sum(x[i,a] for a in intersect(subprobArcSet, setdiff(A_minus_i[i,n], dummyarc))) for i in sp_orders) + sum(y[a] for a in intersect(subprobArcSet, A_minus_hd[n])) - sum(sum(x[i,a] for a in intersect(subprobArcSet, setdiff(A_plus_i[i,n], dummyarc))) for i in sp_orders) - sum(y[a] for a in intersect(subprobArcSet, A_plus_hd[n])) == sp_trucksupply[n])
	
	#Driver constraints
	@constraint(m, driverFlowBalance[d in sp_drivers, n in setdiff(subprobNodeSet, [n2 for n2 in numnodes+1:extendednumnodes]) ], sum(z[d,a] for a in intersect(subprobArcSet, A_minus_d[d,n])) - sum(z[d,a] for a in intersect(subprobArcSet, A_plus_d[d,n])) == sp_driversupply[d,n])
	@constraint(m, driverAvailability[a in intersect(subprobArcSet, A_space)], sum(z[d,a] for d in intersect(sp_drivers, availableDrivers[a])) == w[a] + outsideorderdemand[a])
	for i in sp_orders, a in intersect(subprobArcSet, orderArcSet_space[i])
		set_normalized_coefficient(driverAvailability[a], x[i,a], -1)
	end
	for a in intersect(subprobArcSet, A_hasdriver_space)
		set_normalized_coefficient(driverAvailability[a], y[a], -1)
	end

	#Force drivers to return home at least every other day
	startoffhours_beforewindow = [if [t2 for t2 in T_off_0[d] if t2 < sp_times[1]] == [] 2*horizon else last([t2 for t2 in T_off_0[d] if t2 < sp_times[1]]) end for d in drivers]
	startoffhours_afterwindow = [if [t2 for t2 in T_off_0[d] if t2 >= last(sp_times)] == [] 2*horizon else [t2 for t2 in T_off_0[d] if t2 >= last(sp_times)][1] end for d in drivers]
	for d in sp_drivers_home
		if intersect(sp_times[1:length(sp_times)-1], T_off_0[d]) != []
			if length(intersect(sp_times[1:length(sp_times)-1], T_off_0[d])) >= 2
				@constraint(m, [t = [t for t in intersect(sp_times[1:length(sp_times)-1], T_off_0[d])[1:length(intersect(sp_times[1:length(sp_times)-1], T_off_0[d]))-1]]], sum(z[d,arcs[nodes[(driverHomeLocs[d],t2)], nodes[(driverHomeLocs[d],t2+tstep)]]] for t2 in intersect(sp_times, [t3 for t3 in t:tstep:t+24],T_off_0[d])) >= 1) 
			end
			if (startoffhours_beforewindow[d] != 2*horizon) & (intersect(sp_times, T_off_0[d])[1]+tstep <= maximum(sp_times) )
				@constraint(m, z[d, arcs[nodes[driverHomeLocs[d], intersect(sp_times, T_off_0[d])[1]], nodes[driverHomeLocs[d],intersect(sp_times, T_off_0[d])[1]+tstep]]] + z_currsol[d, arcs[nodes[driverHomeLocs[d], startoffhours_beforewindow[d]], nodes[driverHomeLocs[d], startoffhours_beforewindow[d]+tstep]]] >= 1) 
			end
			if (startoffhours_afterwindow[d] != 2*horizon) & (last(intersect(sp_times[1:length(sp_times)-1], T_off_0[d]))+tstep <= maximum(sp_times) )
				@constraint(m, z[d, arcs[nodes[driverHomeLocs[d],last(intersect(sp_times[1:length(sp_times)-1], T_off_0[d]))], nodes[driverHomeLocs[d],last(intersect(sp_times[1:length(sp_times)-1], T_off_0[d]))+tstep]]] + z_currsol[d, arcs[nodes[driverHomeLocs[d], startoffhours_afterwindow[d]], nodes[driverHomeLocs[d], startoffhours_afterwindow[d]+tstep]]] >= 1)
			end
		end
	end

	#====================================================#
	
	#optimize!(m)
	#getobjectivevalue(m)
	#Add in warm start from current solution? Only if subproblem is slow

	#====================================================#

	#Solve IP
	status = optimize!(m)

	if termination_status(m) == MOI.OPTIMAL
		solvetime = solve_time(m)
		newobj = getobjectivevalue(m)
	elseif termination_status(m) == MOI.TIME_LIMIT
		println("Time out in the subproblem")
		solvetime = solve_time(m)
		newobj = getobjectivevalue(m)
	else
		println("Error in solving!")
		solvetime = solve_time(m)
		newobj = 10000000
	end

	if sp_compl_orders == []
		oldobj = sum(sum(c[a]*x_currsol[i,a] for a in intersect(orderArcSet[i], subprobArcSet)) for i in sp_orders) + sum(c[a]*y_currsol[a] for a in intersect(A_hasdriver, subprobArcSet)) + sum(u[a]*w_currsol[a] for a in intersect(subprobArcSet, A_space)) 
	else
		oldobj = lambda * sum((ordtime_currsol[i] - shortesttriptimes[i])/shortesttriptimes[i] for i in sp_compl_orders) + sum(sum(c[a]*x_currsol[i,a] for a in intersect(orderArcSet[i], subprobArcSet)) for i in sp_orders) + sum(c[a]*y_currsol[a] for a in intersect(A_hasdriver, subprobArcSet)) + sum(u[a]*w_currsol[a] for a in intersect(subprobArcSet, A_space)) 
	end

	println("New = ", newobj, " vs. old = ", oldobj, " in ", solvetime)

	#====================================================#

	if newobj < 10000000
		ordermiles = sum(sum(c[a]*getvalue(x[i,a]) for a in intersect(orderArcSet[i], subprobArcSet)) for i in sp_orders) 
		emptymiles = sum(c[a]*getvalue(y[a]) for a in intersect(A_hasdriver, subprobArcSet)) 
		taximiles = sum(u[a]*getvalue(w[a]) for a in intersect(subprobArcSet, A_space)) 
	else
		ordermiles = 0
		emptymiles = 0
		taximiles = 0
	end

	#====================================================#

	return x, y, z, w, ordtime, newobj, oldobj, solvetime, ordermiles, emptymiles, taximiles

end

#-------------------------------------------------------------------------#

function updatesolution(x, y, z, w, ordtime, subprobArcSet, sp_orders, sp_drivers, sp_compl_orders)

	for i in sp_orders, a in intersect(orderArcSet[i], subprobArcSet)
		x_currsol[i,a] = getvalue(x[i,a])
	end
	for i in sp_orders
		x_currpath[i] = []
		for a in orderArcSet[i]
			if x_currsol[i,a] == 1
				push!(x_currpath[i], a)
			end
		end
	end

	for a in intersect(A_hasdriver, subprobArcSet)
		y_currsol[a] = getvalue(y[a])
	end

	for d in sp_drivers, a in intersect(homeArcSet[d], subprobArcSet)
		z_currsol[d,a] = getvalue(z[d,a])
	end
	for d in sp_drivers
		z_currpath[d] = []
		for a in homeArcSet[d]
			if z_currsol[d,a] == 1
				push!(z_currpath[d], a)
			end
		end
	end

	for a in intersect(subprobArcSet, A_space)
		w_currsol[a] = getvalue(w[a])
	end

	for i in sp_compl_orders
		ordtime_currsol[i] = getvalue(ordtime[i])
	end

end

#-------------------------------------------------------------------------#

function createdataframe()

	df = DataFrame(instance_id = [],
			subproblem_id = [],
			neighborhood = [],
			starttime = [],
			endtime = [],
			timewindowlength = [],
			numlocs_sp = [],
			numorders_sp = [],
			numorders_global = [],
			numdrivers_sp = [],
			numtrucks_sp = [],
			nodes_sp = [],
			arcs_sp = [],
			arcdensity = [],
			neighborhoodarea = [],
			ordermiles = [],
			emptymiles = [],
			taximiles = [],
			avgdelay = [],
			historicaltrafficdifference_max = [],
			historicaltrafficdifference_avg = [],
			orderhours = [],
			orderidletime_total = [],
			orderidletime_avg = [],
			orderidletime_max = [],
			orderidletime_pct = [],
			trafficimbalance_max = [],
			trafficimbalance_avg = [],
			avgordershortestpathmiles = [],
			avgorderdetourmiles = [],
			avgsegmentlengthhours = [],
			driverutilization = [],
			longorders = [],
			shortorders = [],
			instanceoptimalitygap = [],
			arcreducedcost_avg = [],
			arcreducedcost_max = [],
			reoptimized_count = [],
			adjacentneighborhoodcondition = [],
			adjacentneighborhoodrecentlyoptimized = [],
			overlappingneighborhoodrecentlyoptimized = [],
			oldobj = [],
			newobj = [],
			improvement = [],
			improvementpct = [],
			solvetimelist = [],
			overallimprovementsofar = []
           )

	return df

end

#-------------------------------------------------------------------------#

function calculatehistoricalarctraffic()

	#Import data
	hubdata = CSV.read("data/hub_data_isb_connect.csv", DataFrame)
	orderdata = CSV.read("data/lh_data_isb_connect_clean.csv", DataFrame)
	numhubs, numords = size(hubdata)[1], size(orderdata)[1]

	#Weirdly complicated way to read in the order data 
	orderarcs = []
	for i in 1:numords
		ind = 9
		currloc = orderdata[i,ind]
		while (ind < 19) & (!ismissing(orderdata[i,ind + 1]))
			try
				push!(orderarcs, (hubsReverseLookup[orderdata[i,ind]], hubsReverseLookup[orderdata[i,ind + 1]]))
				ind += 1
				currloc = orderdata[i,ind]
			catch
				ind += 1
				currloc = orderdata[i,ind]
			end
		end
	end

	#Initialize arc counter
	arcCounter = Dict()
	for arc in prearcs
		if (arc[1] in 1:numlocs) & (arc[2] in 1:numlocs)
			arcCounter[arc[1],arc[2]] = 0
		end
	end

	#Count the number of trips between every pair of pit stops
	for i in 1:length(orderarcs)
		startPoint = orderarcs[i][1]
		endPoint = orderarcs[i][2]
		if (startPoint in 1:numlocs) & (endPoint in 1:numlocs)
			arcCounter[(startPoint, endPoint)] += 1
		end
	end

	#Find the arcs with the most and least traffic (number of orders)
	mx = maximum(values(arcCounter)) 
	mn = minimum(values(arcCounter)) 

	#Calculate the color of each arc and add to list used for the drawing
	arcCounter = sort(collect(arcCounter), by=x->x[2])
	trafficindex = Dict()
	for arc in arcCounter
		trafficindex[arc[1]] = (arc[2] - mn)/(mx - mn)
	end

	return trafficindex

end

#-------------------------------------------------------------------------#

function calculatecurrentarctraffic()

	bigset = []
	for i in orders
		bigset = union(bigset, orderArcSet[i])
	end

	arcCounter = Dict()
	for arc in prearcs
		if (arc[1] in 1:numlocs) & (arc[2] in 1:numlocs)
			arcCounter[arc[1],arc[2]] = 0
		end
	end

	for a in intersect(A_hasdriver, A_space)
		sl, el = nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1]
		if (sl != el) & (a in bigset)
			arcCounter[sl,el] += sum(x_currsol[i,a] for i in orders if a in orderArcSet[i]) + y_currsol[a]
		end
	end

	mx = maximum(values(arcCounter)) 
	mn = minimum(values(arcCounter)) 

	#Calculate the color of each arc and add to list used for the drawing
	arcCounter = sort(collect(arcCounter), by=x->x[2])
	trafficindex = Dict()
	for arc in arcCounter
		trafficindex[arc[1]] = (arc[2] - mn)/(mx - mn)
	end

	return trafficindex

end	

#-------------------------------------------------------------------------#

function calculatestaticfeatures(subprob, sp_orders, sp_drivers, subprobNodeSet, subprobArcSet, locgroupidLookup)

	neighborhood = locgroupidLookup[subprob["locgroup"]] 
	timewindowlength = min(horizon+tstep, subprob["endtime"]) - subprob["starttime"]
	numlocs_sp = length(subprob["locgroup"])
	numorders_sp = length(sp_orders)
	numorders_global = length(orders)
	numdrivers_sp = length(sp_drivers)
	numtrucks_sp = 0
	nodes_sp = length(subprobNodeSet)
	arcs_sp = length(subprobArcSet)
	arcdensity = arcs_sp / (numlocs_sp * (numlocs_sp-1) * (timewindowlength/tstep) )

	xcoords = hubCoords[:,1][collect(subprob["locgroup"])]
	ycoords = hubCoords[:,2][collect(subprob["locgroup"])]
	neighborhoodarea = (maximum(xcoords) - minimum(xcoords)) * (maximum(ycoords) - minimum(ycoords))

	staticfeatures = [neighborhood subprob["starttime"] subprob["endtime"] timewindowlength numlocs_sp numorders_sp numorders_global numdrivers_sp numtrucks_sp nodes_sp arcs_sp arcdensity neighborhoodarea]

	return staticfeatures

end

#-------------------------------------------------------------------------#

function calculatedynamicfeatures(subprob, sp_orders, sp_drivers, subprobArcSet, ordermiles, emptymiles, taximiles, historicaltraffic, currenttraffic)

	bigset, bigset2 = [], []
	for i in sp_orders
		bigset = union(bigset, setdiff(intersect(subprobArcSet, orderArcSet[i]), A_space))
		bigset2 = union(bigset2, intersect(subprobArcSet, orderArcSet[i], A_space))
	end

	#Dynamic features
	avgdelay = sum((orderdelayoutcomes[i] - shortesttriptimes[i])/shortesttriptimes[i] for i in sp_orders) 
	orderhours = 0
	if bigset2 != []
		orderhours += sum(sum(arcLength[nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1]]*x_currsol[i,a] for a in intersect(subprobArcSet, A_space, orderArcSet[i])) for i in sp_orders if intersect(subprobArcSet, A_space, orderArcSet[i]) != []) 
	end
	orderidletime_total = 0
	orderidletime_max = 0
	if bigset != []
		orderhours += sum(sum(tstep*x_currsol[i,a] for a in setdiff(intersect(subprobArcSet, orderArcSet[i]), A_space)) for i in sp_orders if setdiff(intersect(subprobArcSet, orderArcSet[i]), A_space) != [])
		orderidletime_total += sum(sum(tstep*x_currsol[i,a] for a in setdiff(intersect(subprobArcSet, orderArcSet[i]), A_space)) for i in sp_orders if setdiff(intersect(subprobArcSet, orderArcSet[i]), A_space) != [])
		orderidletime_max += maximum([sum(tstep*x_currsol[i,a] for a in setdiff(intersect(subprobArcSet, orderArcSet[i]), A_space)) for i in sp_orders if setdiff(intersect(subprobArcSet, orderArcSet[i]), A_space) != []])
	end
	orderidletime_avg = orderidletime_total / length(sp_orders)
	if orderhours == 0
		orderidletime_pct = 0
	else
		orderidletime_pct = orderidletime_total / orderhours
	end

	historicaltrafficdifference_max = maximum([abs(historicaltraffic[sl,el] - currenttraffic[sl,el]) for (sl,el) in intersect(keys(historicaltraffic),keys(currenttraffic)) if (sl in subprob["locgroup"]) & (el in subprob["locgroup"])])
	historicaltrafficdifference_avg = mean([abs(historicaltraffic[sl,el] - currenttraffic[sl,el]) for (sl,el) in intersect(keys(historicaltraffic),keys(currenttraffic)) if (sl in subprob["locgroup"]) & (el in subprob["locgroup"])]) 	
	trafficimbalance_max = maximum([abs(currenttraffic[sl,el] - currenttraffic[el,sl]) for (sl,el) in keys(currenttraffic) if (sl < el) &  (sl in subprob["locgroup"]) & (el in subprob["locgroup"])])
	trafficimbalance_avg = mean([abs(currenttraffic[sl,el] - currenttraffic[el,sl]) for (sl,el) in keys(currenttraffic) if (sl < el) &  (sl in subprob["locgroup"]) & (el in subprob["locgroup"])])

	#Slightly dynamic
	avgordershortestpathmiles = sum(distbetweenlocs[originloc[i],destloc[i]] for i in sp_orders) / length(sp_orders)
	longorders = sum([1 for i in sp_orders if distbetweenlocs[originloc[i],destloc[i]] > 500])	
	shortorders = sum([1 for i in sp_orders if distbetweenlocs[originloc[i],destloc[i]] <= 500])	
	avgorderdetourmiles = sum(sum(c[a] * x_currsol[i,a] for a in orderArcSet[i]) - distbetweenlocs[originloc[i],destloc[i]] for i in sp_orders) / length(sp_orders)
	avgsegmentlengthhours = 0
	if bigset2 != []
		avgsegmentlengthhours += sum(sum(arcLength[nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1]]*x_currsol[i,a] for a in intersect(subprobArcSet, orderArcSet[i], A_space)) for i in sp_orders if intersect(subprobArcSet, orderArcSet[i], A_space) != [])/sum(sum(x_currsol[i,a] for a in intersect(subprobArcSet, orderArcSet[i], A_space)) for i in sp_orders if intersect(subprobArcSet, orderArcSet[i], A_space) != [])
	end
	driverutilization = 2 * sum(sum(z_currsol[d,a] for a in intersect(homeArcSet[d], subprobArcSet, A_space)) for d in sp_drivers if intersect(homeArcSet[d], subprobArcSet, A_space) != [])/sum(sum(z_currsol[d,a] for a in intersect(homeArcSet[d], subprobArcSet)) for d in sp_drivers if intersect(homeArcSet[d], subprobArcSet) != [] )

	dynamicfeatures = [ordermiles emptymiles taximiles avgdelay historicaltrafficdifference_max historicaltrafficdifference_avg orderhours orderidletime_total orderidletime_avg orderidletime_max orderidletime_pct trafficimbalance_max trafficimbalance_avg avgordershortestpathmiles avgorderdetourmiles avgsegmentlengthhours driverutilization longorders shortorders]

	return dynamicfeatures

end

#-------------------------------------------------------------------------#

function calculatealgorithmfeatures(subprobindex)
	
	reoptimized_count = totalreoptimizations[subprobindex]
	adjacentneighborhoodcondition = 0
	adjacentneighborhoodrecentlyoptimized = adjacentreoptimizations[subprobindex]
	overlappingneighborhoodrecentlyoptimized = overlappingreoptimizations[subprobindex]

	algorithmfeatures = [reoptimized_count adjacentneighborhoodcondition adjacentneighborhoodrecentlyoptimized overlappingneighborhoodrecentlyoptimized]

	return algorithmfeatures

end

#-------------------------------------------------------------------------#

function calculateoptimizationfeatures()
	
	instanceoptimalitygap = 0
	arcreducedcost_avg = 0
	arcreducedcost_max = 0

	optimizationfeatures = [instanceoptimalitygap arcreducedcost_avg arcreducedcost_max]

	return optimizationfeatures

end

#-------------------------------------------------------------------------#

function calculatepredictors(oldobj, newobj, solvetime)

	improvement = oldobj - newobj
	improvementpct = (oldobj - newobj)/oldobj
	
	predictors = [oldobj newobj improvement improvementpct solvetime]

	return predictors

end

#-------------------------------------------------------------------------#

function addsubproblemfeatures(counter, subprobindex, subprob, sp_orders, sp_drivers, subprobNodeSet, subprobArcSet, locgroupidLookup, ordermiles_sp, emptymiles_sp, taximiles_sp, historicaltraffic, currenttraffic, oldobj_sp, newobj_sp, solvetime_sp)

	idlist = [instanceid counter]
	staticfeatures = calculatestaticfeatures(subprob, sp_orders, sp_drivers, subprobNodeSet, subprobArcSet, locgroupidLookup)
	dynamicfeatures = calculatedynamicfeatures(subprob, sp_orders, sp_drivers, subprobArcSet, ordermiles_sp, emptymiles_sp, taximiles_sp, historicaltraffic, currenttraffic) 
	optimizationfeatures = calculateoptimizationfeatures()
	algorithmfeatures = calculatealgorithmfeatures(subprobindex)
	predictors = calculatepredictors(oldobj_sp, newobj_sp, solvetime_sp)
	overallimprovement = [objectiveimprovement_lsns]

	newrow = hcat(idlist, staticfeatures, dynamicfeatures, optimizationfeatures, algorithmfeatures, predictors, overallimprovement)

	push!(feature_df, newrow)

end

