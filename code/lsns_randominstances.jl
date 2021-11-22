
#---------------------------------------------------------------------------------------#

#Make a list of all origin-destination pairs to be used for random selection
function findorigindestinationdistribution(lh_filename)

	data_agg = CSV.read(lh_filename, DataFrame)
	
	origdestchooser = []
	for i in 1:size(data_agg)[1]
		orig, dest = data_agg[!,26][i], data_agg[!,27][i]
		if (orig != dest) & (1 <= orig <= numlocs) & (1 <= dest <= numlocs) 
			push!(origdestchooser, (orig, dest))
		end
	end

	return origdestchooser
		
end

#---------------------------------------------------------------------------------------#

#Make a list of the number of orders each iteration to be used for random selection
function findorderpertimeperioddistribution(lh_filename)
	
	data_agg = CSV.read(lh_filename, DataFrame)
	
	ordercounter = Dict()
	
	for i in 1:size(data_agg)[1]
		orig, dest = data_agg[!,26][i], data_agg[!,27][i]
		if (orig != dest) & (1 <= orig <= numlocs) & (1 <= dest <= numlocs) 			
			pickup_ts = DateTime(1970) + Dates.Millisecond(data_agg[!,6][i])
			start_avail_ts = floor(pickup_ts - Dates.Hour(8), Dates.Day) + Dates.Hour(8) + timedelta*floor(Dates.Millisecond(floor(Dates.value(pickup_ts - (floor(pickup_ts - Dates.Hour(8), Dates.Day) + Dates.Hour(8)))/timedelta)), Dates.Hour)
			start_avail = convert(Int64,(start_avail_ts - weekstart) / (Millisecond(1) * 1000 * 3600))
			try
				ordercounter[start_avail] += 1
			catch
				ordercounter[start_avail] = 1
			end
		end
	end
	
	numrandomorderschooser = Dict()
	for t in 0:tstep:24-tstep 
		numrandomorderschooser[t] = []
	end
	for item in keys(ordercounter)
		push!(numrandomorderschooser[mod(item,24)], ordercounter[item])
	end

	return numrandomorderschooser

end

#---------------------------------------------------------------------------------------#

function getrandomorders(lh_filename, orderwindowstart, orderwindowend, origdestchooser, numrandomorderschooser, iterationordercap)

	traveltime = cacheShortestTravelTimes(numlocs, prearcs, "rdd time")

	data_agg = CSV.read(lh_filename, DataFrame)
		
	originloc, destloc, available, duedate, trueoriginloc, orderinprog = [], [], [], [], [], []

	#Generate orders
	for timeblock in orderwindowstart:tstep:orderwindowend
		neworders = rand(numrandomorderschooser[mod(timeblock,24)])
		for i in 1:min(iterationordercap, neworders)

			#Randomly select origin and destination
			orig, dest = rand(origdestchooser)
							
			start_avail = timeblock
			end_avail = horizon
			start_due = min(horizon, start_avail + traveltime[orig, dest])
			end_due = horizon

			push!(originloc, orig)
			push!(destloc, dest)
			push!(available, (start_avail, end_avail))
			push!(duedate, (start_due, end_due))
			push!(trueoriginloc, orig)

		end		
	end

	numorders = length(originloc)

	return numorders, originloc, destloc, available, duedate, trueoriginloc, orderinprog

end

#---------------------------------------------------------------------------------------#

function getnextorders_randominstance(timedelta, currentdatetime, orders, lh_data_file)

	totaltimedelta = Dates.value(Dates.Hour(currentdatetime - weekstart))

	#====================================================#

	#Find new orders in the time horizon
	numexistingorders = length(orders)
	maxneworders = 10000 #iterationordercap #popfirst!(ordercaps)

	if maxneworders > 0

		orderwindowstart, orderwindowend = totaltimedelta + becomesavailablehours, totaltimedelta + becomesavailablehours
		numneworders, originloc_ol, destloc_ol, available_ol, duedate_ol, trueorderorigins, trash = getrandomorders(lh_data_file, orderwindowstart, orderwindowend, origdestchooser, numrandomorderschooser, iterationordercap)

		newordersbyiter[totaltimedelta] = numneworders

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
				if (solutionmethod == "otd") 
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

		if (solutionmethod == "otd") 

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

	else

		newordersbyiter[totaltimedelta] = 0

	end
	
end