
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function findshortestpath_driver(d)

	#Create copies of all important sets (so they can be modified for the shortest path problem)
	arcsSP, arcLookupSP, nodesLookupSP = deepcopy(arcs), deepcopy(arcLookup), deepcopy(nodesLookup)
	homeArcSetSP = deepcopy(homeArcSet[d])
	lengthSP = Dict()
	for a in 1:numarcs
		lengthSP[a] = nodesLookup[arcLookup[a][2]][2] - nodesLookup[arcLookup[a][1]][2] 
	end

	#Create dummy origin and destination nodes + arcs 
	driverorigin = driverStartNodes[d]
	dummydest = numnodes + 1
	numnodesSP = numnodes + 1
	nodesLookupSP[dummydest] = (0,0)

	#Add arcs from all home nodes to the dummy destination
	index = numarcs + 1
	for t in 0:tstep:horizon
		n = nodes[driverHomeLocs[d],t]
		arcsSP[n, dummydest] = index
		arcLookupSP[index] = (n, dummydest) 
		lengthSP[index] = 0
		push!(homeArcSetSP, index)
		index += 1
	end
	numarcsSP = length(lengthSP)
	arclistSP = [a for a in 1:length(lengthSP)]

	#Initialize shortest path algorithm (Bellman-Ford)
	currdistance = repeat([999999999.0],outer=[numnodesSP])
	currdistance[driverorigin] = 0
	prevnode, prevarc = zeros(numnodesSP), zeros(numnodesSP)
	
	for k in 1:horizon/tstep+1 #Max path length
		for a in homeArcSetSP #1:numarcsSP
			n_end, n_start = arcLookupSP[a][2], arcLookupSP[a][1]
			if currdistance[n_end] > currdistance[n_start] + lengthSP[a] + .000001
				currdistance[n_end] = currdistance[n_start] + lengthSP[a]
				prevnode[n_end] = n_start
				prevarc[n_end] = a
			end
		end
	end

	#Format the shortest path output
	shortestpathnodes_rev = [dummydest]
	shortestpatharcs_rev = []
	node = dummydest
	while node != driverorigin
		push!(shortestpatharcs_rev, Int(prevarc[node]))
		node = Int(prevnode[node])
		push!(shortestpathnodes_rev, node)
	end
	shortestpathnodes = reverse(shortestpathnodes_rev[2:length(shortestpathnodes_rev)]) 
	shortestpatharcs = reverse(shortestpatharcs_rev[2:length(shortestpatharcs_rev)]) 

	endtime = nodesLookup[last(shortestpathnodes)][2]

	return currdistance[dummydest], shortestpathnodes, shortestpatharcs, endtime

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function heuristicassignments(traveltimefordelay_flag)

	wklydelta = mod(Dates.value(Dates.Hour(currentdatetime - weekstart)), 168)

	#====================================================#

	if traveltimefordelay_flag == 0
		traveltimebetweenlocs = traveltimebetweenlocs_rdd
	elseif traveltimefordelay_flag == 1
		traveltimebetweenlocs = traveltimebetweenlocs_raw
	elseif traveltimefordelay_flag == 2
		traveltimebetweenlocs = traveltimebetweenlocs_llr
	end

	#====================================================#

	officialassignments, orderassignments, driverassignments = [], Dict(), Dict()
	for i in orders
		orderassignments[i] = 0
	end
	#Set default arc for drivers to be stay where you are
	for d in drivers
		if !(d in driversintransit)
			driverassignments[d] = setdiff(A_plus[driverStartNodes[d]], A_space)[1]
		elseif d in driversintransit
			driverassignments[d] = 0
		end
	end

	nowarcs = []
	for a in 1:numarcs
		if nodesLookup[arcLookup[a][1]][2] == 0
			push!(nowarcs, a)
		end
	end

	driversForHire = deepcopy(availableDrivers)
	#Prioritize drivers: 1. Drivers away from home where the arc would take them home, 2. Other drivers away from home, 3. Drivers at home
	for a in nowarcs
		endloc = nodesLookup[arcLookup[a][2]][1]
		driversForHire[a] = union(intersect(driversForHire[a], intersect(awayfromhome, assignedDrivers[endloc])), intersect(driversForHire[a], setdiff(awayfromhome, assignedDrivers[endloc])), setdiff(driversForHire[a], awayfromhome))
	end

	driversassignedthisiter = []
	for i in orders
		#Find current location of order
		currloc, availtime = nodesLookup[Origin[i][1]]
		destinloc = nodesLookup[last(Destination[i])][1]

		if availtime == 0
			if (i in ordersinprogress) & (availabletrucks[nodes[currloc,0]] > 0)
				#Identify possible next arcs for the order
				possiblenextarcs = []
				for a in A_plus[nodes[currloc,0]]
					if !(a in A_space) || (length(driversForHire[a]) > 0)
						push!(possiblenextarcs, a)
					end
				end

				#Evaluate next arcs in terms of added miles and delivery time
				penalties = []
				for a in possiblenextarcs
					nextloc, nexttime = nodesLookup[arcLookup[a][2]]
					d = driversForHire[a][1]
					#Calculate the "objective penalty"/cost of taking this particular arc
					extratime = nexttime + traveltimebetweenlocs[nextloc, destinloc] - traveltimebetweenlocs[currloc, destinloc]
					if (d in awayfromhome) & (driverHomeLocs[d] == nextloc)
						#If the driver is currently away and this arc will bring them home, then this arc is "free" because driver is headed back anyway
						extramiles = 0 + distbetweenlocs[nextloc, destinloc] - distbetweenlocs[currloc, destinloc]
					else
						extramiles = c[a] + distbetweenlocs[nextloc, destinloc] - distbetweenlocs[currloc, destinloc]
					end
					objectivepenalty = lambda * extratime + extramiles
					push!(penalties, objectivepenalty)
				end

				#Find the best arc for order i
				nextarc = possiblenextarcs[argmin(penalties)]

				#Assign order
				push!(officialassignments, (i, nextarc, 0))
				orderassignments[i] = nextarc
				global totalpastcost += c[nextarc]
				global totalordertrips += 1
				global totalordermiles += c[nextarc]
				ordermilesoutcomes[i] += c[nextarc]

				#Add order to visualization lists
				starttime, endtime, startloc, endloc = nodesLookup[arcLookup[nextarc][1]][2], nodesLookup[arcLookup[nextarc][2]][2], nodesLookup[arcLookup[nextarc][1]][1], nodesLookup[arcLookup[nextarc][2]][1]
				push!(pastordersegments, (i, starttime + wklydelta, endtime + wklydelta, startloc, endloc))

				#Assign driver
				if nextarc in A_space
					drvr = popfirst!(driversForHire[nextarc])
					driverassignments[drvr] = nextarc
					push!(driversassignedthisiter, drvr)
			
					#Add driver to visualization lists
					push!(pastdriversegments, (drvr, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
					push!(pastdriversegments_space, (drvr, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
				
					#Remove assigned driver from the available list for other arcs
					for a in nowarcs
						remove!(driversForHire[a], drvr)
					end

				end

				#Update trucks
				availabletrucks[arcLookup[nextarc][1]] -= 1
				availabletrucks[arcLookup[nextarc][2]] += 1

			elseif !(i in ordersinprogress) & (availabletrucks[nodes[currloc,0]] > 0)
				#Identify possible next arcs for the order
				possiblenextarcs = []
				for a in A_plus[nodes[currloc,0]]
					if !(a in A_space) || (length(driversForHire[a]) > 0)
						push!(possiblenextarcs, a)
					end
				end

				#Evaluate next arcs in terms of added miles and delivery time
				penalties = []
				for a in possiblenextarcs
					nextloc, nexttime = nodesLookup[arcLookup[a][2]]

					#Calculate the "objective penalty"/cost of taking this particular arc
					extratime = nexttime + traveltimebetweenlocs[nextloc, destinloc] - traveltimebetweenlocs[currloc, destinloc]
					d = driversForHire[a][1]
					if (d in awayfromhome) & (driverHomeLocs[d] == nextloc)
						#If the driver is currently away and this arc will bring them home, then this arc is "free" because driver is headed back anyway
						extramiles = 0 + distbetweenlocs[nextloc, destinloc] - distbetweenlocs[currloc, destinloc]
					else
						extramiles = c[a] + distbetweenlocs[nextloc, destinloc] - distbetweenlocs[currloc, destinloc]
					end
					objectivepenalty = lambda * extratime + extramiles
					push!(penalties, objectivepenalty)
				end

				#Find the best arc for order i
				nextarc = possiblenextarcs[argmin(penalties)]

				#If the best next step is for the order to leave its origin location, assign a driver
				#Otherwise, assign nothing --> do not pick up the order this iteration
				if nextarc in A_space

					#Assign driver and order
					drvr = popfirst!(driversForHire[nextarc])
					push!(officialassignments, (i, nextarc, drvr))
					orderassignments[i] = nextarc
					driverassignments[drvr] = nextarc
					push!(driversassignedthisiter, drvr)
					global totalpastcost += c[nextarc]
					global totalordertrips += 1
					global totalordermiles += c[nextarc]

					#Update trucks
					availabletrucks[arcLookup[nextarc][1]] -= 1
					availabletrucks[arcLookup[nextarc][2]] += 1

					#Remove assigned driver from the available list for other arcs
					for a in nowarcs
						remove!(driversForHire[a], drvr)
					end

					#Add to visualization
					starttime, endtime, startloc, endloc = nodesLookup[arcLookup[nextarc][1]][2], nodesLookup[arcLookup[nextarc][2]][2], nodesLookup[arcLookup[nextarc][1]][1], nodesLookup[arcLookup[nextarc][2]][1]
					push!(pastordersegments, (i, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
					push!(pastdriversegments, (drvr, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
					push!(pastdriversegments_space, (drvr, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
				end

			elseif (i in ordersinprogress)

				println("Infeasible! No truck for this order")

			end
		end
	end

	#====================================================#

	#Send away drivers home if they were not assigned
	for d in setdiff(awayfromhome, driversassignedthisiter)

		if !(0 in T_off[drivershift[d]])
			#Find shortest path home on the homeArcSet of driver d (takes into account off hours)
			timetohome, shortestpathnodes, shortestpatharcs, endtime = findshortestpath_driver(d)

			#Check if they need to be sent home right now or if they can wait an additional iteration to find a ride
			if (endtime == endofcurrshift[d]) || (endtime == duehome[d])
				#Send the driver home
				a = shortestpatharcs[1]
				startloc = nodesLookup[arcLookup[a][1]][1]
				#Check the number of trucks
				if availabletrucks[nodes[startloc,0]] > 0
					push!(officialassignments, ("empty", a, d))
					driverassignments[d] = a
					availabletrucks[arcLookup[a][1]] -= 1
					availabletrucks[arcLookup[a][2]] += 1
					global totalpastcost += c[a]
					global totalemptytrips += 1
					global totalemptymiles += c[a]

					#Add to visualization
					starttime, endtime, startloc, endloc = nodesLookup[arcLookup[a][1]][2], nodesLookup[arcLookup[a][2]][2], nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1]
					push!(pastemptysegments, (1, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
					push!(pastdriversegments, (d, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
					push!(pastdriversegments_space, (d, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
				else
					push!(officialassignments, ("taxi", a, d))
					driverassignments[d] = a
					global totalpastcost += u[a]
					global totaltaxitrips += 1
					global totaltaximiles += u[a]

					#Add to visualization
					starttime, endtime, startloc, endloc = nodesLookup[arcLookup[a][1]][2], nodesLookup[arcLookup[a][2]][2], nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1]
					push!(pasttaxisegments, (1, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
					push!(pastdriversegments, (d, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
					push!(pastdriversegments_space, (d, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
				end
			end
		end
	end

	#====================================================#

	for d in drivers
		if driverassignments[d] == setdiff(A_plus[driverStartNodes[d]], A_space)[1]
			starttime, endtime, startloc, endloc = nodesLookup[arcLookup[driverassignments[d]][1]][2], nodesLookup[arcLookup[driverassignments[d]][2]][2], nodesLookup[arcLookup[driverassignments[d]][1]][1], nodesLookup[arcLookup[driverassignments[d]][2]][1]
			push!(pastdriversegments, (d, starttime + wklydelta, endtime + wklydelta, startloc, endloc))
		end
	end

	#====================================================#

	return officialassignments, orderassignments, driverassignments

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#