
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function driverArcSets_online(numlocs, numarcs, numnodes, prearcs, drivers, tstep, horizon, nodes, arcs, assignedDrivers, A_minus, A_plus, T_off, drivershift, driverHomeLocs, T_off_0, shiftlength)

	#Find the time since the start of the original horizon
	wklydelta = mod(Dates.value(Dates.Hour(currentdatetime - weekstart)), 168)

	#====================================================#

	#Find and store the shortest travel time between each pair of locations
	traveltime = cacheShortestTravelTimes(numlocs, prearcs, "rdd time")
	fullhourlist = [t for t in 0:tstep:horizon-tstep]

	#Initialize lists
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
	for d in drivers
		closelocs[d] = []
	end

	#Add arcs between each location and itself
	prearcs_aug = deepcopy(prearcs)
	for l in 1:numlocs
		push!(prearcs_aug, (l, l, tstep, tstep, 0))
	end
	
	#Main loop for driver arc sets
	for d in drivers
		#Find the hours of the first shift for the driver
		firstshift = []
		for t in setdiff(fullhourlist, T_off[drivershift[d]])[1]:tstep:horizon
			if t in T_off[drivershift[d]]
				break
			else
				push!(firstshift, t)
			end 
		end

		#If the driver is not currently home, add any arcs reachable during their shifts, adjusting the first shift for the different start location
		if nodesLookup[driverStartNodes[d]][1] != driverHomeLocs[d]
			for arc in prearcs_aug
				orig, dest = arc[1], arc[2]
				h = driverHomeLocs[d]
				t1 = traveltime[h, orig]
				t2 = arc[3]
				t3 = traveltime[dest, h]

				if ((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength))
					if !(orig in closelocs[d])
						push!(closelocs[d], orig)
					end
					if !(dest in closelocs[d])
						push!(closelocs[d], dest)
					end
				end
			end

			for arc in prearcs_aug
				orig, dest = arc[1], arc[2]
				h = driverHomeLocs[d]
				currloc = driverStartNodes[d]

				for t in setdiff(fullhourlist, T_off[drivershift[d]])
					if t in firstshift
						t1 = traveltime[currloc, orig]
						t2 = arc[3]
						t3 = traveltime[dest, h]
						#Arc must finish before the end of the horizon and before the "next" off hour of the driver
						if (t + t2 <= [t4 for t4 in union(T_off[drivershift[d]], horizon) if t4 > t][1]) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
							push!(homeArcSet[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							if orig != dest
								push!(homeArcSet_space[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							end				
							push!(availableDrivers[arcs[nodes[orig, t], nodes[dest, t + t2]]], d)
						end
					else
						t1 = traveltime[h, orig]
						t2 = arc[3]
						t3 = traveltime[dest, h]
						#Arc must finish before the end of the horizon and before the "next" off hour of the driver
						if (t + t2 <= [t4 for t4 in union(T_off[drivershift[d]], horizon) if t4 > t][1]) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
							push!(homeArcSet[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							if orig != dest
								push!(homeArcSet_space[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							end				
							push!(availableDrivers[arcs[nodes[orig, t], nodes[dest, t + t2]]], d)
						end
					end
				end

			end

		#If the driver is currently home, add any arcs reachable during their shifts
		else 
			for arc in prearcs_aug
				orig, dest = arc[1], arc[2]
				h = driverHomeLocs[d]
				t1 = traveltime[h, orig]
				t2 = arc[3]
				t3 = traveltime[dest, h]

				if ((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength))
					if !(orig in closelocs[d])
						push!(closelocs[d], orig)
					end
					if !(dest in closelocs[d])
						push!(closelocs[d], dest)
					end

					for t in setdiff(fullhourlist, T_off[drivershift[d]])
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

	#Add arcs for in-transit drivers
	for item in driversintransit
		d, l, availtime = item
		for t in 0:tstep:availtime - tstep
			a = arcs[nodes[l,t], nodes[l,t+tstep]]
			if !(a in homeArcSet[d])
				push!(homeArcSet[d], a)
			end
		end
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function driverArcSets_online_bns()

	#Find the time since the start of the original horizon
	wklydelta = mod(Dates.value(Dates.Hour(currentdatetime - weekstart)), 168)

	#====================================================#

	#Find and store the shortest travel time between each pair of locations
	traveltime = cacheShortestTravelTimes(numlocs, prearcs, "rdd time")
	fullhourlist = [t for t in 0:tstep:horizon-tstep]

	#Initialize lists
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
	for d in drivers
		closelocs[d] = []
	end

	#Add arcs between each location and itself
	prearcs_aug = deepcopy(prearcs)
	for l in 1:numlocs
		push!(prearcs_aug, (l, l, tstep, tstep, 0))
	end
	
	#Main loop for driver arc sets
	for d in drivers
		#Find the hours of the first shift for the driver
		firstshift = []
		for t in setdiff(fullhourlist, T_off[drivershift[d]])[1]:tstep:horizon
			if t in T_off[drivershift[d]]
				break
			else
				push!(firstshift, t)
			end 
		end

		#If the driver is not currently home, add any arcs reachable during their shifts, adjusting the first shift for the different start location
		if (awaylastnight[d] == 1)
			for arc in prearcs_aug
				orig, dest = arc[1], arc[2]
				h = driverHomeLocs[d]
				t1 = traveltime[h, orig]
				t2 = arc[3]
				t3 = traveltime[dest, h]

				if ((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength))
					if !(orig in closelocs[d])
						push!(closelocs[d], orig)
					end
					if !(dest in closelocs[d])
						push!(closelocs[d], dest)
					end
				end
			end

			for arc in prearcs_aug
				orig, dest = arc[1], arc[2]
				h = driverHomeLocs[d]
				currloc, availtime = nodesLookup[driverStartNodes[d]]

				for t in setdiff(fullhourlist, T_off[drivershift[d]])
					if (t in firstshift) & (t >= availtime)
						t1 = traveltime[currloc, orig]
						t2 = arc[3]
						t3 = traveltime[dest, h]
						
						#Arc must finish before the end of the horizon and before the "next" off hour of the driver
						#if (t + t2 <= [t4 for t4 in union(T_off[drivershift[d]], horizon) if t4 > t][1]) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
						if (firstshift[1] + t1 <= t) & (t1 + t2 <= last(firstshift) + tstep) & (t1 + t2 + t3 <= last(firstshift) - firstshift[1] + tstep) 
							push!(homeArcSet[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							if orig != dest
								push!(homeArcSet_space[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							end				
							push!(availableDrivers[arcs[nodes[orig, t], nodes[dest, t + t2]]], d)
						end
					elseif t >= availtime
						t1 = traveltime[h, orig]
						t2 = arc[3]
						t3 = traveltime[dest, h]
						#Arc must finish before the end of the horizon and before the "next" off hour of the driver
						if (t + t2 <= [t4 for t4 in union(T_off[drivershift[d]], horizon) if t4 > t][1]) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
							push!(homeArcSet[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							if orig != dest
								push!(homeArcSet_space[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							end				
							push!(availableDrivers[arcs[nodes[orig, t], nodes[dest, t + t2]]], d)
						end
					end
				end

			end

		elseif nodesLookup[driverStartNodes[d]][1] != driverHomeLocs[d]
			for arc in prearcs_aug
				orig, dest = arc[1], arc[2]
				h = driverHomeLocs[d]
				t1 = traveltime[h, orig]
				t2 = arc[3]
				t3 = traveltime[dest, h]

				if ((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength))
					if !(orig in closelocs[d])
						push!(closelocs[d], orig)
					end
					if !(dest in closelocs[d])
						push!(closelocs[d], dest)
					end
				end
			end

			for arc in prearcs_aug
				orig, dest = arc[1], arc[2]
				h = driverHomeLocs[d]
				currloc, availtime = nodesLookup[driverStartNodes[d]]

				for t in setdiff(fullhourlist, T_off[drivershift[d]])
					if (t in firstshift) & (t >= availtime)
						t1 = traveltime[currloc, orig]
						t2 = arc[3]
						t3 = traveltime[dest, h]
						#Arc must finish before the end of the horizon and before the "next" off hour of the driver
						#if (t + t2 <= [t4 for t4 in union(T_off[drivershift[d]], horizon) if t4 > t][1]) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
						if (firstshift[1] + t1 <= t) & (t1 + t2 <= last(firstshift) + tstep) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
							push!(homeArcSet[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							if orig != dest
								push!(homeArcSet_space[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							end				
							push!(availableDrivers[arcs[nodes[orig, t], nodes[dest, t + t2]]], d)
						end
					elseif t >= availtime
						t1 = traveltime[h, orig]
						t2 = arc[3]
						t3 = traveltime[dest, h]
						#Arc must finish before the end of the horizon and before the "next" off hour of the driver
						if (t + t2 <= [t4 for t4 in union(T_off[drivershift[d]], horizon) if t4 > t][1]) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
							push!(homeArcSet[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							if orig != dest
								push!(homeArcSet_space[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							end				
							push!(availableDrivers[arcs[nodes[orig, t], nodes[dest, t + t2]]], d)
						end
					end
				end

			end

		#If the driver is currently home, add any arcs reachable during their shifts
		else 
			for arc in prearcs_aug
				orig, dest = arc[1], arc[2]
				h = driverHomeLocs[d]
				t1 = traveltime[h, orig]
				t2 = arc[3]
				t3 = traveltime[dest, h]

				if ((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength))
					if !(orig in closelocs[d])
						push!(closelocs[d], orig)
					end
					if !(dest in closelocs[d])
						push!(closelocs[d], dest)
					end
				end
			end

			for arc in prearcs_aug
				orig, dest = arc[1], arc[2]
				h = driverHomeLocs[d]
				currloc, availtime = nodesLookup[driverStartNodes[d]]

				for t in setdiff(fullhourlist, T_off[drivershift[d]])
					if (t in firstshift) & (t >= availtime)
					t1 = traveltime[currloc, orig]
					t2 = arc[3]
					t3 = traveltime[dest, h]
						#Arc must finish before the end of the horizon and before the "next" off hour of the driver
						#if (t + t2 <= [t4 for t4 in union(T_off[drivershift[d]], horizon) if t4 > t][1]) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
						if (firstshift[1] + t1 <= t) & (t1 + t2 <= last(firstshift) + tstep) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
							push!(homeArcSet[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							if orig != dest
								push!(homeArcSet_space[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							end				
							push!(availableDrivers[arcs[nodes[orig, t], nodes[dest, t + t2]]], d)
						end
					elseif t >= availtime
						t1 = traveltime[h, orig]
						t2 = arc[3]
						t3 = traveltime[dest, h]
						#Arc must finish before the end of the horizon and before the "next" off hour of the driver
						if (t + t2 <= [t4 for t4 in union(T_off[drivershift[d]], horizon) if t4 > t][1]) & (((t1 + t2 <= shiftlength) & (t3 <= shiftlength)) || ((t1 <= shiftlength) & (t2 + t3 <= shiftlength)))
							push!(homeArcSet[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							if orig != dest
								push!(homeArcSet_space[d], arcs[nodes[orig, t], nodes[dest, t + t2]])
							end				
							push!(availableDrivers[arcs[nodes[orig, t], nodes[dest, t + t2]]], d)
						end
					end
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

	#Add arcs for in-transit drivers
	#for item in driversintransit
	#	d, l, availtime = item
	#	for t in 0:tstep:availtime - tstep
	#		a = arcs[nodes[l,t], nodes[l,t+tstep]]
	#		if !(a in homeArcSet[d])
	#			push!(homeArcSet[d], a)
	#		end
	#	end
	#end
end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function yarcreduction_online(numarcs, availableDrivers, A_space, numnodes, A_plus, A_minus)

	for a in 1:numarcs
		ub = 99999
		if a in A_space
			ub = length(availableDrivers[a])
		end
		push!(yupperbound, ub)
		if (a in A_space) & (ub > 0)
			push!(A_hasdriver, a)
			push!(A_hasdriver_space, a)
		elseif !(a in A_space)
			push!(A_hasdriver, a)
		end
	end

	for n in 1:numnodes
		A_plus_hd[n] = []
		A_minus_hd[n] = []
		for a in A_plus[n]
			if a in A_hasdriver
				push!(A_plus_hd[n], a)
			end
		end
		for a in A_minus[n]
			if a in A_hasdriver
				push!(A_minus_hd[n], a)
			end
		end
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updatedrivers(timedelta, newcurrentdatetime, weekstart, T_off_Monday8am, z, currtime)

	#Update the time 
	#wklydelta = mod(Dates.value(Dates.Hour(currentdatetime - weekstart)), 168)
	wklydelta = mod(Dates.value(Dates.Hour(newcurrentdatetime - weekstart)), 168)

	#====================================================#

	#UPDATE INITIAL DRIVER LOCATIONS

	#Find drivers arriving at each location at time = timedelta
	missingdrivers = [d for d in drivers]
	for d in drivers, l in 1:numlocs
		n = nodes[l, timedelta]
		if (A_minus_d[d,n] != []) && (sum(getvalue(z[d,a]) for a in A_minus_d[d,n]) > 0.001)
			driverStartNodes[d] = nodes[l, 0]
			remove!(missingdrivers, d)
		end
	end

	#If no initial location found, then the driver must be in transit at the current time
	#We assign the driver to start at the end location of their in progress trip, then assign them to drive an empty truck until they 
	#reach the end point of the trip, at which point they become available to be assigned to a new order (or begin their off hours)
	for d in missingdrivers
		for t in timedelta:tstep:timedelta+shiftlength, l in 1:numlocs
			n = nodes[l, t]
			#IF driver will next arrive at location l THEN set new current location
			if (A_minus_d[d,n] != []) && (sum(getvalue(z[d,a]) for a in A_minus_d[d,n]) > 0.001)
				currentloc, availtime = l, t - timedelta
				driverStartNodes[d] = nodes[l, 0]
				push!(driversintransit, (d, currentloc, availtime))
				break
			end
		end
	end

	#====================================================#

	#Update driver flow balance node set
	for d in drivers
		N_flow_d[d] = setdiff([n for n in 1:numnodes], union(N_end, driverStartNodes[d]))
	end

	#====================================================#

	#UPDATE DRIVER SCHEDULES

	#Adjust driver shift schedule for the new date and time
	for shift in T_off_Monday8am
		shiftedshift = []
		for hr in shift
			if wklydelta <= hr <= wklydelta+horizon-tstep
				push!(shiftedshift, hr - wklydelta)
			end
		end
		push!(T_off, shiftedshift)
	end

	#Create other driver shift lists (used to write constraints)
	for d in drivers
		T_off_0[d], T_off_constr[d] = [], []
		for t in 0:tstep:horizon
			if (t in T_off[drivershift[d]]) & !(t-tstep in T_off[drivershift[d]])
				push!(T_off_0[d], t)
				if (t + 24 <= horizon) & (intersect([t3 for t3 in t:tstep:t+24],T_off_0[d]) != [])
					push!(T_off_constr[d], t)  
				end
			end
		end
	end
	
	#====================================================#

	for d in drivers, t in currtime+tstep:tstep:currtime+timedelta
		if t in T_off_Monday8am_0[drivershift[d]]
			if (A_minus_d[d,nodes[driverHomeLocs[d],t-currtime]] != []) && (sum(getvalue(z[d,a]) for a in A_minus_d[d,nodes[driverHomeLocs[d],t-currtime]]) > 0.001)
				1==1
			else	
				drivernightshome[d] -= 1
				drivernightsaway[d] += 1
			end
		end
	end

	#====================================================#

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updatedrivers_bns(timedelta, newcurrentdatetime, weekstart, T_off_Monday8am, driverassignments)

	#Update the time 
	#wklydelta = mod(Dates.value(Dates.Hour(currentdatetime - weekstart)), 168)
	wklydelta = mod(Dates.value(Dates.Hour(newcurrentdatetime - weekstart)), 168)

	#====================================================#

	#Adjust driver shift schedule for the new date and time
	for shift in T_off_Monday8am
		shiftedshift = []
		for hr in shift
			if wklydelta <= hr <= wklydelta+horizon-tstep
				push!(shiftedshift, hr - wklydelta)
			end
		end
		push!(T_off, shiftedshift)
	end

	#====================================================#

	#Update driver initial locations
	for d in drivers
		a = driverassignments[d]
		if a == 0
			endloc, availtime = nodesLookup[driverStartNodes[d]]
			driverStartNodes[d] = nodes[endloc, max(0, availtime - timedelta)]
		else
			endloc, endtime = nodesLookup[arcLookup[a][2]]
			if endtime <= timedelta
				#Update location
				driverStartNodes[d] = nodes[endloc, max(0, endtime - timedelta)]

				#Update awayfromhome
				if (endloc != driverHomeLocs[d]) & !(d in awayfromhome)
					push!(awayfromhome, d)
				elseif endloc == driverHomeLocs[d]
					remove!(awayfromhome, d)
				end
			else
				#Update location
				driverStartNodes[d] = nodes[endloc, max(0, endtime - timedelta)]
				push!(driversintransit, d)

				#Update awayfromhome - we won't be assigning this driver a new arc next iteration, so we don't need to worry about it
				remove!(awayfromhome, d)
			end
		end
	end

	#====================================================#

	#UPDATE DRIVER SCHEDULES

	#Adjust driver shift schedule for the new date and time
	for shift in T_off_Monday8am
		shiftedshift = []
		for hr in shift
			if wklydelta <= hr <= wklydelta+horizon-tstep
				push!(shiftedshift, hr - wklydelta)
			end
		end
		push!(T_off, shiftedshift)
	end

	#Create other driver shift lists (used to write constraints)
	for d in drivers
		T_off_0[d], T_off_constr[d] = [], []
		for t in 0:tstep:horizon
			if (t in T_off[drivershift[d]]) & !(t-tstep in T_off[drivershift[d]])
				push!(T_off_0[d], t)
				if (t + 24 <= horizon) & (intersect([t3 for t3 in t:tstep:t+24],T_off_0[d]) != [])
					push!(T_off_constr[d], t)  
				end
			end
		end
	end

	#====================================================#

	#Update endofcurrshift
	fullhourslist = [t for t in 0:tstep:horizon]
	for d in drivers
		activehours = setdiff(fullhourslist, T_off[drivershift[d]])
		for t in activehours[1]:tstep:horizon
			if !(t in activehours)
				endofcurrshift[d] = t
				break
			end
		end
	end

	#Update duedate
	for d in drivers
		if awaylastnight[d] == 1
			duehome[d] = endofcurrshift[d] 
		else
			activehours = setdiff(fullhourslist, union(T_off[drivershift[d]], [t for t in 0:tstep:endofcurrshift[d]-tstep]))
			for t in activehours[1]:tstep:horizon
				if !(t in activehours)
					duehome[d] = t
					break
				end
			end
		end
	end
	
end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updatetrucks(timedelta, newcurrentdatetime, weekstart, T_off_Monday8am, x, y, z, A_minus_irest)

	#Update the time 
	wklydelta = mod(Dates.value(Dates.Hour(newcurrentdatetime - weekstart)), 168)

	#====================================================#

	#Create set of arcs that begin before the time horizon
	earlyarcs = []
	for a in 1:numarcs
		if (nodesLookup[arcLookup[a][1]][2] < timedelta) & (nodesLookup[arcLookup[a][2]][2] > timedelta)
			push!(earlyarcs, a)
		end
	end

	#Find any trucks that are currently in transit
	intransittruckcount = [0 for l in 1:numlocs]
	for t in timedelta:tstep:timedelta+shiftlength, l in 1:numlocs
		n = nodes[l, t]
		totaltrucks = 0
		for i in orders
			if intersect(earlyarcs, A_minus_irest[i,n]) != []
				totaltrucks += sum(getvalue(x[i,a]) for a in intersect(earlyarcs, A_minus_irest[i,n]))
			end
		end
		if intersect(earlyarcs, A_minus_hd[n]) != []
			totaltrucks += sum(getvalue(y[a]) for a in intersect(earlyarcs, A_minus_hd[n])) 
		end
		totaltrucks = round(totaltrucks)
		#totaltrucks = sum(sum(getvalue(x[i,a]) for a in intersect(earlyarcs, A_minus_irest[i,n])) for i in orders) + sum(getvalue(y[a]) for a in intersect(earlyarcs, A_minus_hd[n])) 
		if totaltrucks > 0.001
			intransittruckcount[l] += totaltrucks
			push!(trucksintransit, (l, t - timedelta, totaltrucks))
		end
	end

	#Update truck initial locations
	for l in 1:numlocs
		n = nodes[l, timedelta]
		enteringtrucks = 0 
		for i in orders
			if setdiff(A_minus_irest[i,n], dummyarc) != []
				enteringtrucks += sum(getvalue(x[i,a]) for a in setdiff(A_minus_irest[i,n], dummyarc)) 
			end
		end
		if A_minus_hd[n] != []
			enteringtrucks += sum(getvalue(y[a]) for a in A_minus_hd[n]) 
		end
		#enteringtrucks += sum(sum(getvalue(x[i,a]) for a in setdiff(A_minus_i[i,n], dummyarc)) for i in orders) + sum(getvalue(y[a]) for a in A_minus_hd[n])
		enteringtrucks = round(enteringtrucks)
		#Total trucks starting at location l = the entering trucks plus any trucks found to be in transit to location l
		m_0[l] = enteringtrucks + intransittruckcount[l]
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updatetrucks_bns(timedelta)

	#Move unused trucks to the next node
	for l in 1:numlocs
		n1, n2 = nodes[l,0], nodes[l,timedelta]
		availabletrucks[n2] += availabletrucks[n1]
		availabletrucks[n1] = 0
	end

	#Iterate all nodes forward by timedelta
	for l in 1:numlocs, t in 0:tstep:horizon-timedelta
		newnode = nodes[l,t]
		oldnode = nodes[l,t+timedelta]
		availabletrucks[newnode] = availabletrucks[oldnode] 
	end
	for l in 1:numlocs, t in horizon-timedelta+tstep:horizon
		newnode = nodes[l,t]
		availabletrucks[newnode] = 0
	end

end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updateorders(x, y, z, timedelta, currentdatetime, A_minus_irest, A_plus_irest)

	#Update the time 
	totaldelta = Dates.value(Dates.Hour(currentdatetime - weekstart))

	#====================================================#

	#Find current location of orders from last time (may be in transit) and update origin/destination windows
	completedorders = []
	for i in orders
		orderintransit_flag[i] = 0 
	end
	
	for i in orders
		originstarttime = nodesLookup[Origin[i][1]][2]

		#If order path infeasible in current iteration
		if getvalue(x[i, dummyarc]) > 0.001
			#Iterate all origin nodes forward by timedelta
			originloc2, originaltime = nodesLookup[Origin[i][1]]
			newstarttime = max(0,  originaltime - timedelta)

			#If order in progress, move the origin node forward by timedelta
			if i in ordersinprogress
				newnode = nodes[originloc2, newstarttime]
				Origin[i] = [newnode]

			#If order not in progress, move start of origin forward by timedelta, but leaving end of origin as the horizon
			else
				for t in originaltime-tstep:-tstep:newstarttime
					newnode = nodes[originloc2, t]
					pushfirst!(Origin[i], newnode)
				end
			end
		
			#Iterate all destination nodes forward by timedelta
			newDestinationList = []
			n = Destination[i][1]
			destloc, originaltime = nodesLookup[n]
			for t in max(0,originaltime-timedelta):tstep:horizon
				newnode = nodes[destloc, t]
				push!(newDestinationList, newnode)
			end
			push!(newDestinationList, extendednodes[destloc, dummyendtime])
			Destination[i] = newDestinationList

		#Else if the order was available before the start of the new time horizon
		elseif originstarttime < timedelta

			#Find current location of order

			#Initialize with origin location and time
			currentloc, availtime, endwindowtime, mostrecentarc = nodesLookup[Origin[i][1]][1], 0.0, nodesLookup[last(Origin[i])][2] - timedelta, 0
			
			#Set time list for search 
			#Start at timedelta and go forward, if nothing found, look from timedelta and go backward
			#We want to find the most recent arc first, but if none are found, the order may have completed between time 0 and timedelta
			timelist = [t for t in timedelta:tstep:timedelta+shiftlength]
			for t in timedelta-tstep:-tstep:tstep
				push!(timelist, t)
			end

			#Find the arc most recently traveled by order, if it exists
			for t in timelist, l in 1:numlocs
				n = nodes[l, t]
				#IF (order just arrived or will arrive at location l) & (order left previous location in the past) 
				#i.e. the order has already traveled to or is currently in transit to loc l, THEN set new current location
				#println("$i, $l, $t")

				#if (A_minus_irest[i,n] != []) && (sum(getvalue(x[i,a]) for a in A_minus_irest[i,n]) > 0.001) && (sum(sum(sum(getvalue(x[i,a]) for a in A_plus_irest[i,nodes[l2,t2]]) for l2 in 1:numlocs if A_plus_irest[i,nodes[l2,t2]] != []) for t2 in originstarttime:tstep:timedelta-tstep) > 0.001)
				if (A_minus_irest[i,n] != []) && (sum(getvalue(x[i,a]) for a in A_minus_irest[i,n]) > 0.001)  
			
					#Identify possible nodes to check
					nodestochecklist = []
					for t2 in originstarttime:tstep:timedelta-tstep, l2 in 1:numlocs
						if A_plus_irest[i,nodes[l2,t2]] != []
							push!(nodestochecklist, nodes[l2,t2])
						end
					end

					if (nodestochecklist != []) && (sum(sum(getvalue(x[i,a]) for a in A_plus_irest[i,n2]) for n2 in nodestochecklist) > 0.001)
						currentloc, availtime, endwindowtime = l, t - timedelta, t - timedelta

						#Find the arc
						for n2 in nodestochecklist, a in A_plus_irest[i,n2]
							if getvalue(x[i,a]) > 0.001
								mostrecentarc = a
							end
						end

						if availtime != 0
							orderintransit_flag[i] = 1
						end
						break
					end
				end
			end

			if currentloc == orderOriginalStartLoc[i]
				endwindowtime = horizon
			end

			#Check whether order was completed
			if currentloc == nodesLookup[Destination[i][1]][1]

				push!(completedorders, i)

				if traveltimefordelay_flag == 0
					arcendtime = availtime
				elseif traveltimefordelay_flag >= 1
					arcendtime = arcfinishtime[mostrecentarc] - timedelta
				end

				orddelay = ((arcendtime + totaldelta - orderOriginalStartTime[i]) - shortesttriptimes[i])/shortesttriptimes[i]
				global totalpastcost += lambda * orddelay
				global total_delay += orddelay
				global max_delay = max(max_delay,orddelay )

				remove!(ordersinprogress, i)
				orderdelayoutcomes[i] += arcendtime + totaldelta - orderOriginalStartTime[i]
			else
				#Create new origin set
				newOriginList = []
				for t in availtime:tstep:endwindowtime
					newnode = nodes[currentloc, t]
					push!(newOriginList, newnode)
				end
				Origin[i] = newOriginList

				#Mark if the order has left its destination
				if currentloc != orderOriginalStartLoc[i]
					#Add to ordersinprogress
					if !(i in ordersinprogress)
						push!(ordersinprogress, i)
					end
				end

				#Iterate all destination nodes forward by timedelta
				newDestinationList = []
				n = Destination[i][1]
				destloc, originaltime = nodesLookup[n]
				for t in max(0,originaltime-timedelta):tstep:horizon
					newnode = nodes[destloc, t]
					push!(newDestinationList, newnode)
				end
				push!(newDestinationList, extendednodes[destloc, dummyendtime])
				Destination[i] = newDestinationList
				
				#Remove traversed arc from order routes
				if (solutionmethod == "rr") && !(i in troubleorders)
					completedloc = findfirst(x->hubsReverseLookup[x]==currentloc, psseq[i])
					psseq[i] = psseq[i][completedloc:length(psseq[i])]
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
			
		#Else, the order becomes available after the start of the new time horizon
		else
			#Iterate all origin nodes forward by timedelta
			originloc2, originaltime = nodesLookup[Origin[i][1]]
			newstarttime = max(0,  originaltime - timedelta)

			#If order in progress, move the origin node forward by timedelta
			if i in ordersinprogress
				newnode = nodes[originloc2, newstarttime]
				Origin[i] = [newnode]

			#If order not in progress, move start of origin forward by timedelta, but leaving end of origin as the horizon
			else
				for t in originaltime-tstep:-tstep:newstarttime
					newnode = nodes[originloc2, t]
					pushfirst!(Origin[i], newnode)
				end
			end

			#Iterate all destination nodes forward by timedelta
			newDestinationList = []
			n = Destination[i][1]
			destloc, originaltime = nodesLookup[n]
			for t in max(0,originaltime-timedelta):tstep:horizon
				newnode = nodes[destloc, t]
				push!(newDestinationList, newnode)
			end
			push!(newDestinationList, extendednodes[destloc, dummyendtime])
			Destination[i] = newDestinationList
		end
	end

	#====================================================#

	#Remove completed orders
	for i in completedorders
		remove!(orders, i)
	end
 
	#====================================================#

	#Update order flow balance node sets
	for i in orders
		N_flow_i[i] = []
		for n in 1:numnodes
			if !(n in Origin[i]) & !(n in Destination[i])
				push!(N_flow_i[i], n)
			end
		end
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updateorders_bns(timedelta, currentdatetime, orderassignments)

	#Update the time 
	totaldelta = Dates.value(Dates.Hour(currentdatetime - weekstart))

	#====================================================#

	#Find current location of orders from last time (may be in transit) and update origin/destination windows
	completedorders = []
	for i in orders
		orderintransit_flag[i] = 0 
	end
	
	for i in orders
		originstarttime = nodesLookup[Origin[i][1]][2]

		#Else if the order was available before the start of the new time horizon
		if originstarttime == 0

			#Find current location of order
			a = orderassignments[i]
			#If not assigned
			if a == 0
				#Update the origin location forward by timedelta
				origloc, origtime = nodesLookup[Origin[i][1]] 
				Origin[i] = [nodes[origloc, max(0, origtime - timedelta)]]
			else
				endloc, availtime = nodesLookup[arcLookup[a][2]]

				#Check if order was completed
				if endloc == nodesLookup[Destination[i][1]][1]
					push!(completedorders, i)

					if traveltimefordelay_flag == 0
						arcendtime = availtime
					elseif traveltimefordelay_flag >= 1
						arcendtime = arcfinishtime[a] - timedelta
					end

					orddelay = ((arcendtime + totaldelta - orderOriginalStartTime[i]) - shortesttriptimes[i])/shortesttriptimes[i]
					global totalpastcost += lambda * orddelay
					global total_delay += orddelay
					global max_delay = max(max_delay,orddelay )

					remove!(ordersinprogress, i)
					orderdelayoutcomes[i] += arcendtime + totaldelta - orderOriginalStartTime[i]
				else
					#Update order origin to new location
					Origin[i] = [nodes[endloc, max(0, availtime - timedelta)]]
					
					#Mark if order has left origin location
					if (endloc != orderOriginalStartLoc[i]) & !(i in ordersinprogress)
						push!(ordersinprogress, i)
					end

				end

			end
			
		#Else, the order becomes available after the start of the new time horizon
		else
			#Iterate all origin nodes forward by timedelta
			originloc2, originaltime = nodesLookup[Origin[i][1]]
			newstarttime = max(0,  originaltime - timedelta)

			#If order in progress, move the origin node forward by timedelta
			if i in ordersinprogress
				newnode = nodes[originloc2, newstarttime]
				Origin[i] = [newnode]

			#If order not in progress, move start of origin forward by timedelta, but leaving end of origin as the horizon
			else
				for t in originaltime-tstep:-tstep:newstarttime
					newnode = nodes[originloc2, newstarttime]
					pushfirst!(Origin[i], newnode)
				end
			end

			#Iterate all destination nodes forward by timedelta
			newDestinationList = []
			n = Destination[i][1]
			destloc, originaltime = nodesLookup[n]
			for t in max(0,originaltime-timedelta):tstep:horizon
				newnode = nodes[destloc, t]
				push!(newDestinationList, newnode)
			end
			push!(newDestinationList, extendednodes[destloc, dummyendtime])
			Destination[i] = newDestinationList
		end
	end

	#====================================================#

	#Remove completed orders
	for i in completedorders
		remove!(orders, i)
	end
 
end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updateorderarcsets_abcg(timedelta)

	#UPDATE ORDER ARC SETS (bring forward by timedelta)
	for i in orders
		orderArcSet_space[i] = []
		for n in 1:extendednumnodes
			A_minus_i[i,n], A_plus_i[i,n] = [], []
		end
		for a in 1:length(orderArcSet[i])
			if orderArcSet[i][a] == dummyarc
				#Do not remove or alter dummy arc 
				nplus, nminus = Origin[i][1], Destination[i][1]
				push!(A_minus_i[i,nminus], dummyarc)
				push!(A_plus_i[i,nplus], dummyarc)
			elseif orderArcSet[i][a] > numarcs 
				#Remove arc from arc set because it is no longer connected to the others
				orderArcSet[i][a] = 0
				#Do not remove or alter extended horizon arcs
				#push!(A_minus_i[i, nminus], a)
				#push!(A_plus_i[i, nplus], a)
			else
				startloc, starttime = nodesLookup[arcLookup[orderArcSet[i][a]][1]][1], nodesLookup[arcLookup[orderArcSet[i][a]][1]][2] - timedelta
				endloc, endtime = nodesLookup[arcLookup[orderArcSet[i][a]][2]][1], nodesLookup[arcLookup[orderArcSet[i][a]][2]][2] - timedelta

				if (endtime >= 0) & (starttime >= 0)
					#Replace old arc with new arc
					newarc = arcs[nodes[startloc, starttime], nodes[endloc, endtime]]
					orderArcSet[i][a] = newarc
					if newarc in A_space
						push!(orderArcSet_space[i], newarc)
					end
					nplus, nminus = nodes[startloc, starttime], nodes[endloc, endtime]
					push!(A_minus_i[i, nminus], newarc)
					push!(A_plus_i[i, nplus], newarc)
				else
					#Placeholder for deletion from arc set
					orderArcSet[i][a] = 0
				end
			end

		end

		#Remove obsolete arcs
		remove!(orderArcSet[i], 0)

	end

	#====================================================#

	if pruneorderarcs_flag == 1
		#Prune order arc sets that are not possible given the updated order location 
		for i in ordersinprogress
			currentloc, availabletime = nodesLookup[Origin[i][1]]
			for a in 1:length(orderArcSet[i])
				if orderArcSet[i][a] <= numarcs
					arcstartloc, arcstarttime = nodesLookup[arcLookup[orderArcSet[i][a]][1]]
					if traveltimebetweenlocs_rdd[currentloc, arcstartloc] > arcstarttime - availabletime
						#Placeholder for deletion from arc set
						remove!(orderArcSet_space[i], orderArcSet[i][a])
						for n in 1:numnodes
							remove!(A_minus_i[i,n], orderArcSet[i][a])
							remove!(A_plus_i[i,n], orderArcSet[i][a])
						end
						orderArcSet[i][a] = 0
					end
				end
			end
			#Remove impossible arcs
			remove!(orderArcSet[i], 0)
		end

		#Prune order arc sets for orders that are not in progress, but are within the pick up window
		for i in setdiff(orders, ordersinprogress)
			currentloc, availabletime = nodesLookup[Origin[i][1]]
			if availabletime == 0
				for l in setdiff([l for l in 1:numlocs], currentloc)
					n = nodes[l,0]
					for a in setdiff(A_plus_i[i,n], union([a for a in numarcs+1:dummyarc]))
						remove!(orderArcSet[i], a)
						remove!(orderArcSet_space[i], a)
						nend = arcLookup[a][2]
						remove!(A_minus_i[i,nend], a)
					end
					A_plus_i[i,n] = []
				end
			end
		end

		#Prune any arcs that are no longer reachable after the first pruning
		for i in orders
			currentloc, availabletime = nodesLookup[Origin[i][1]]
			firstevaluationnode = nodes[1, availabletime + tstep]
			for n in firstevaluationnode:numnodes
				if setdiff(A_minus_i[i,n], union([a for a in numarcs+1:dummyarc])) == []
					for a in setdiff(A_plus_i[i,n], union([a for a in numarcs+1:dummyarc]))
						remove!(orderArcSet[i], a)
						remove!(orderArcSet_space[i], a)
						nend = arcLookup[a][2]
						remove!(A_minus_i[i,nend], a)
					end
					A_plus_i[i,n] = []
				end
			end
		end	
	end

	#====================================================#

end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function getnextorders(timedelta, currentdatetime, orders, lh_data_file, vnt_data_file)

	totaltimedelta = Dates.value(Dates.Hour(currentdatetime - weekstart))

	#====================================================#

	#Find new orders in the time horizon
	numexistingorders = length(orders)
	maxneworders = 10000 #iterationordercap #popfirst!(ordercaps)

	if maxneworders > 0

		orderwindowstart, orderwindowend = weekstart + Dates.Hour(totaltimedelta) + Dates.Hour(becomesavailablehours) - Dates.Hour(timedelta) + Dates.Millisecond(10), weekstart + Dates.Hour(totaltimedelta) + Dates.Hour(becomesavailablehours)
		numneworders, originloc_ol, destloc_ol, available_ol, duedate_ol, orderidlist_new, psseq_ol, trueorderorigins = pullorders_rivigoroutes(lh_data_file, vnt_data_file, maxneworders, orderwindowstart, orderwindowend, currentdatetime + Dates.Hour(timedelta), tstep, horizon, prearcs, numlocs, timedelta, includeorderidlist)
		newordersbyiter[totaltimedelta] = numneworders

		for orderid in orderidlist_new
			push!(usedorderidlist, orderid)
		end

		#Add to order list
		for i in highestorderindex+1:highestorderindex+numneworders
			push!(orders, i)
		end
		global highestorderindex += numneworders

		#Format Origin and Destination for each order
		currentindex = length(Origin)
		for i in 1:numneworders
			orderindex = currentindex + i
			Origin[orderindex] = []
			Destination[orderindex] = []
			for t in available_ol[i][1]:tstep:available_ol[i][2]
				push!(Origin[orderindex], nodes[originloc_ol[i], t])
			end
			for t in duedate_ol[i][1]:tstep:duedate_ol[i][2]
				push!(Destination[orderindex], nodes[destloc_ol[i], t])
			end
			push!(originloc, originloc_ol[i])
			push!(destloc, destloc_ol[i])
		end

		#Add pitstop sequences to psseq list
		for i in 1:numneworders
			orderindex = currentindex + i
			push!(psseq, psseq_ol[i])
		end

		#Update order flow balance node sets
		for i in 1:numneworders
			orderindex = currentindex + i
			N_flow_i[orderindex] = []
			for n in 1:numnodes
				if !(n in Origin[orderindex]) & !(n in Destination[orderindex])
					push!(N_flow_i[orderindex], n)
				end
			end
		end

		#Add order start times in terms of the original time horizon
		for i in 1:numneworders
			orderindex = currentindex + i
			orderOriginalStartTime[orderindex] = nodesLookup[Origin[orderindex][1]][2] + totaltimedelta
			push!(orderOriginalStartLoc, trueorderorigins[i] )
		end

		#Record shortest possible trip time for each order to be used in objective function
		for i in 1:numneworders
			orderindex = currentindex + i
			
			if traveltimefordelay_flag == 0
				shortestpathtime = traveltimebetweenlocs_rdd[nodesLookup[Origin[orderindex][1]][1], nodesLookup[Destination[orderindex][1]][1]]
			elseif traveltimefordelay_flag == 1
				shortestpathtime = traveltimebetweenlocs_raw[nodesLookup[Origin[orderindex][1]][1], nodesLookup[Destination[orderindex][1]][1]]
			elseif traveltimefordelay_flag == 2
				shortestpathtime = traveltimebetweenlocs_llr[nodesLookup[Origin[orderindex][1]][1], nodesLookup[Destination[orderindex][1]][1]]
			end

			push!(shortesttriptimes, shortestpathtime)
		end

		#====================================================#

		#Create dummy arcs for arc lists
		if solutionmethod != "bns"
			for i in 1:numneworders
				orderindex = currentindex + i
				orderArcSet[orderindex] = [dummyarc]
				orderArcSet_space[orderindex] = []
			end
			for i in 1:numneworders, n in 1:numnodes
				orderindex = currentindex + i
				if n == Origin[orderindex][1]
					A_plus_i[orderindex,n] = [dummyarc]
					A_minus_i[orderindex,n] = []
				elseif n == last(Destination[orderindex])
					A_plus_i[orderindex,n] = []
					A_minus_i[orderindex,n] = [dummyarc]
				else 
					A_plus_i[orderindex,n] = []
					A_minus_i[orderindex,n] = []
				end
			end
		end

		#====================================================#

		#Add dummy end destination for new orders
		for i in 1:numneworders
			orderindex = currentindex + i
			destinationlocation = nodesLookup[Destination[orderindex][1]][1]
			push!(Destination[orderindex], extendednodes[destinationlocation, dummyendtime])
			if solutionmethod != "bns"
				A_minus_i[orderindex, extendednodes[destinationlocation, dummyendtime]] = []
				if (solutionmethod == "otd") || (solutionmethod == "rr")
					for n2 in N_end
						arc_ext = extendedarcs[n2, extendednodes[destinationlocation, dummyendtime]]
						push!(orderArcSet[orderindex], arc_ext)
						push!(A_plus_i[orderindex, n2], arc_ext)
						push!(A_minus_i[orderindex, extendednodes[destinationlocation, dummyendtime]], arc_ext)
					end
				end
			end
		end

		#====================================================#

		if (solutionmethod == "otd") || (solutionmethod == "rr")

			#Append to orderArcSet
			prearcs_aug = deepcopy(prearcs)
			for l in 1:numlocs
				push!(prearcs_aug, (l, l, tstep, tstep))
			end
			
			for i in 1:numneworders, arc in prearcs_aug
				orderindex = currentindex + i
				arcorig, arcdest = arc[1], arc[2]
				if (arcdest != orderOriginalStartLoc[orderindex]) & (arcorig != nodesLookup[Destination[orderindex][1]][1])
					startloc, starttime = nodesLookup[Origin[orderindex][1]]
					endloc = nodesLookup[Destination[orderindex][1]][1]
					t1 = traveltimebetweenlocs_rdd[startloc, arcorig]
					t2 = arc[3]
					t3 = traveltimebetweenlocs_rdd[arcdest, endloc]

					for t in 0:tstep:horizon-t2
						if starttime + t1 <= t
							push!(orderArcSet[orderindex], arcs[nodes[arcorig, t], nodes[arcdest, t + t2]])
							if arcorig != arcdest
								push!(orderArcSet_space[orderindex], arcs[nodes[arcorig, t], nodes[arcdest, t + t2]])
							end				
						end
					end
				end
			end

			#Append to A_plus and A_minus lists
			for i in 1:numneworders
				orderindex = currentindex + i
				for n in 1:numnodes, a in A_plus[n]
					if a in orderArcSet[orderindex]
						push!(A_plus_i[orderindex,n], a)
					end
				end
			end
			for i in 1:numneworders
				orderindex = currentindex + i
				for n in 1:numnodes, a in A_minus[n]
					if a in orderArcSet[orderindex]
						push!(A_minus_i[orderindex,n], a)
					end
				end
			end

		end

		#====================================================#

		if solutionmethod == "rr"

			for i in 1:numneworders
				orderindex = currentindex + i
				rivigoorderroutes[orderindex] = []
				ordoriginloc = orderOriginalStartLoc[orderindex]
				for j in 1:length(psseq[orderindex])-1
					#Collect order routes, formatted as (startloc, endloc, arctime)
					firstloc, nextloc = hubsReverseLookup[psseq[orderindex][j]], hubsReverseLookup[psseq[orderindex][j+1]]
					if firstloc != nextloc
						arctime = arcLength[firstloc, nextloc]
					else
						arctime = tstep
					end
					push!(rivigoorderroutes[orderindex], (firstloc, nextloc, arctime))
					if firstloc != ordoriginloc
						push!(rivigoorderroutes[orderindex], (firstloc, firstloc, tstep))
					end
				end
			end

		end

		#====================================================#

	else

		newordersbyiter[totaltimedelta] = 0

	end
	
end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updateawaylastnight(z)

	for d in drivers
		#If driver is currently off and currently at home, set to 0
		if (0 in T_off[drivershift[d]]) & (getvalue(z[d, arcs[nodes[driverHomeLocs[d],0], nodes[driverHomeLocs[d],tstep]]]) > 0.001)
			awaylastnight[d] = 0
		#If driver is currently off and currently away from home, set to 1
		elseif (0 in T_off[drivershift[d]]) & !(getvalue(z[d, arcs[nodes[driverHomeLocs[d],0], nodes[driverHomeLocs[d],tstep]]]) > 0.001)
			if awaylastnight[d] == 0
				global totalawaynights += 1
			end
			awaylastnight[d] = 1
		end
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updateawaylastnight_bns(driverStartNodes)

	for d in drivers
		currloc = nodesLookup[driverStartNodes[d]][1]
		#If driver is currently off and currently at home, set to 0
		if (0 in T_off[drivershift[d]]) & (currloc == driverHomeLocs[d])
			awaylastnight[d] = 0
		#If driver is currently off and currently away from home, set to 1
		elseif (0 in T_off[drivershift[d]]) & (currloc != driverHomeLocs[d])
			awaylastnight[d] = 1
		end
	end

end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function updatepastsegments(timedelta, x, y, z, w, restrictedArcSet)

	wklydelta = mod(Dates.value(Dates.Hour(currentdatetime - weekstart)), 168)
	
	#====================================================#

	#Find the set of arcs that are locked in after the next iteration
	lockedarcs = []
	for a in 1:numarcs
		if nodesLookup[arcLookup[a][1]][2] < timedelta
			push!(lockedarcs, a)
		end
	end

	#====================================================#

	#Add order segments
	for i in orders, a in intersect(lockedarcs, setdiff(restrictedArcSet[i], dummyarc))
		if getvalue(x[i,a]) > 0.001
			#Add new current segment = (i, starttime, endtime, startloc, endloc)
			arcstartloc, arcendloc = nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1]
			push!(pastordersegments, (i, nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, arcstartloc, arcendloc))
			#Update the total past cost with the completed segment
			global totalpastcost += c[a]
			global totalordertrips += 1
			global totalordermiles += c[a]
			ordermilesoutcomes[i] += c[a]
		end
	end

	#Add driver segments
	for d in drivers, a in intersect(lockedarcs, homeArcSet[d])
		if getvalue(z[d,a]) > 0.001
			#Add new current segment = (dd, starttime, endtime, startloc, endloc)
			arcstartloc, arcendloc = nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1]
			push!(pastdriversegments, (d, nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, arcstartloc, arcendloc))
			if a in A_space
				push!(pastdriversegments_space, (d, nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ))
			end
			if arcstartloc != arcendloc
				global totaldriverhours += arcLength_raw[arcstartloc, arcendloc]
			end
		end
	end

	#Add empty segments
	for a in intersect(lockedarcs, A_hasdriver_space)
		if getvalue(y[a]) > 0.001
			#Add new current segment = (numtrucks, starttime, endtime, startloc, endloc)
			push!(pastemptysegments, (getvalue(y[a]), nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ))
			global totalpastcost += c[a] * getvalue(y[a])
			global totalemptytrips += getvalue(y[a])
			global totalemptymiles += c[a] * getvalue(y[a])
		end
	end

	#Add taxi segments
	for a in intersect(lockedarcs, A_space)
		if getvalue(w[a]) > 0.001
			#Add new current segment = (numtrucks, starttime, endtime, startloc, endloc)
			push!(pasttaxisegments, (getvalue(w[a]), nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ))
			global totalpastcost += u[a] * getvalue(w[a])
			global totaltaxitrips += getvalue(w[a])
			global totaltaximiles += u[a] * getvalue(w[a])
		end
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function orderarcreduction_online(prearcs, shortesttriptime)
		
	for i in orders
		orderArcSet[i] = [dummyarc]
		orderArcSet_space[i] = []
	end
	for i in orders, n in 1:numnodes
		if n == Origin[i][1]
			A_plus_i[i,n] = [dummyarc]
			A_minus_i[i,n] = []
		elseif n == last(Destination[i])
			A_plus_i[i,n] = []
			A_minus_i[i,n] = [dummyarc]
		else 
			A_plus_i[i,n] = []
			A_minus_i[i,n] = []
		end
	end

	for i in orders
		destinationlocation = nodesLookup[Destination[i][1]][1]
		#push!(Destination[i], extendednodes[destinationlocation, dummyendtime])
		A_minus_i[i, extendednodes[destinationlocation, dummyendtime]] = []
		for n2 in N_end
			arc_ext = extendedarcs[n2, extendednodes[destinationlocation, dummyendtime]]
			push!(orderArcSet[i], arc_ext)
			push!(A_plus_i[i, n2], arc_ext)
			push!(A_minus_i[i, extendednodes[destinationlocation, dummyendtime]], arc_ext)
		end
	end

	prearcs_aug = deepcopy(prearcs)
	for l in 1:numlocs
		push!(prearcs_aug, (l, l, tstep, tstep))
	end

	for i in orders, arc in prearcs_aug
		arcorig, arcdest, arctraveltime = arc[1], arc[2], arc[3]

		if (i in ordersinprogress) & (arctraveltime <= shiftlength) & (((arcorig == arcdest) & (arcorig == nodesLookup[Origin[i][1]][1]) ) || ((arcdest != orderOriginalStartLoc[i]) & (arcorig != nodesLookup[Destination[i][1]][1])))
			startloc, starttime = nodesLookup[Origin[i][1]]
			endloc = nodesLookup[Destination[i][1]][1]
			#startloc, starttime = nodesLookup[last(Origin[i])]
			t1 = traveltimebetweenlocs_rdd[startloc, arcorig]
			t2 = arc[3]
			t3 = traveltimebetweenlocs_rdd[arcdest, endloc]

			for t in 0:tstep:horizon-t2
				if starttime + t1 <= t
					push!(orderArcSet[i], arcs[nodes[arcorig, t], nodes[arcdest, t + t2]])
					if arcorig != arcdest
						push!(orderArcSet_space[i], arcs[nodes[arcorig, t], nodes[arcdest, t + t2]])
					end				
				end
			end
		elseif !(i in ordersinprogress) & (arcdest != orderOriginalStartLoc[i]) & (arcorig != nodesLookup[Destination[i][1]][1]) & (arctraveltime <= shiftlength)
			startloc, starttime = nodesLookup[Origin[i][1]]
			endloc = nodesLookup[Destination[i][1]][1]
			#startloc, starttime = nodesLookup[last(Origin[i])]
			t1 = traveltimebetweenlocs_rdd[startloc, arcorig]
			t2 = arc[3]
			t3 = traveltimebetweenlocs_rdd[arcdest, endloc]

			for t in 0:tstep:horizon-t2
				if starttime + t1 <= t
					push!(orderArcSet[i], arcs[nodes[arcorig, t], nodes[arcdest, t + t2]])
					if arcorig != arcdest
						push!(orderArcSet_space[i], arcs[nodes[arcorig, t], nodes[arcdest, t + t2]])
					end				
				end
			end
		end
	end

	#Create A_plus and A_minus lists
	for i in orders, n in 1:numnodes, a in A_plus[n]
		if a in orderArcSet[i]
			push!(A_plus_i[i,n], a)
		end
	end
	for i in orders, n in 1:numnodes, a in A_minus[n]
		if a in orderArcSet[i]
			push!(A_minus_i[i,n], a)
		end
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function assessendofhorizonpenalties(currtime)

	#finallegdistancepenalty = 0.4
	#finallegtimepenalty = 0.3

	for i in orders

		l1, l2 = nodesLookup[Origin[i][1]][1], nodesLookup[Destination[i][1]][1]

		#Add distance penalties
		global totalpastcost += distbetweenlocs[l1,l2] * (1 + finallegdistancepenalty)
		global totalordermiles += distbetweenlocs[l1,l2] * (1 + finallegdistancepenalty)

		#Add distance penalty to outcomes for each order
		ordermilesoutcomes[i] += distbetweenlocs[l1,l2] * (1 + finallegdistancepenalty)
		ordermilespenalty[i] += distbetweenlocs[l1,l2] * (1 + finallegdistancepenalty)

		#Calculate delay penalty 
		posthorizonstarttime = max(0, orderOriginalStartTime[i] - currtime)

		if traveltimefordelay_flag == 0
			ttpenalty = traveltimebetweenlocs_rdd[l1,l2]
		elseif traveltimefordelay_flag >= 1
			ttpenalty = traveltimebetweenlocs_llr[l1,l2]
		end

		#Add delay penalties
		if orderOriginalStartTime[i] <= currtime
			orderdelay = ((currtime + ttpenalty * (1 + finallegtimepenalty) - orderOriginalStartTime[i])- shortesttriptimes[i])/shortesttriptimes[i]
			ordertime = currtime + ttpenalty * (1 + finallegtimepenalty) - orderOriginalStartTime[i]
		else
			orderdelay = ((ttpenalty * (1 + finallegtimepenalty))- shortesttriptimes[i])/shortesttriptimes[i]
			ordertime = ttpenalty * (1 + finallegtimepenalty) 
		end
		global totalpastcost += lambda * orderdelay
		global total_delay += orderdelay
		global max_delay = max( max_delay, orderdelay)

		#Add segment to the list
		if orderOriginalStartTime[i] < horizon
			push!(pastordersegments, (i, horizon, dummyendtime, l1, l2))
		end
		
		#Add delay penalty to outcomes for each order
		orderdelayoutcomes[i] += ordertime
		orderdelaypenalty[i] += ordertime

	end

	#for i in orders
	#	orddelay = ((ttpenalty * (1 + finallegtimepenalty))- shortesttriptimes[i])/shortesttriptimes[i] 
	#	push!(coolprints, string("Order $i completed at t = ", currtime + ttpenalty * (1 + finallegtimepenalty)," with delay = ", orddelay," --> currtime = $currtime (post-horizon)"))
	#	push!(coolprints, string("--> start time = ", orderOriginalStartTime[i], ", shortesttriptime = " shortesttriptimes[i]))
	#end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
