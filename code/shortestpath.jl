
function remove!(a, item)
    deleteat!(a, findall(x->x==item, a))
end

function findshortestpath(i, arccosts, numnodes, numarcs, arcs, arcLookup, A_plus, A_minus, Origin, Destination, nodesLookup, A_space, tstep)

	#Create dummy origin and destination nodes + arcs 
	dummyorig = numnodes + 1
	dummydest = numnodes + 2
	numnodesSP = numnodes + 2
	arcsSP, arcLookupSP, A_plusSP, A_minusSP, nodesLookupSP = deepcopy(arcs), deepcopy(arcLookup), deepcopy(A_plus), deepcopy(A_minus), deepcopy(nodesLookup)
	cSP = Dict()
	for a in 1:numarcs
		cSP[a] = arccosts[a]
	end

	A_plusSP[dummyorig], A_plusSP[dummydest] = [], []
	nodesLookupSP[dummyorig], nodesLookupSP[dummydest] = (0,0), (0,0)

	#Remove the time arcs in Origin[i] (i.e. can't leave late from pick up window)
	for n in Origin[i]
		n2, t2 = nodesLookup[n][1], nodesLookup[n][2] + tstep
		if t2 <= horizon
			a = arcs[n, nodes[n2, t2]]
			remove!(A_plusSP[n], a)
		end
	end

	index = numarcs + 1

	for n in Origin[i]
		arcsSP[dummyorig, n] = index
		push!(A_minusSP[n], index)
		arcLookupSP[index] = (dummyorig, n)
		cSP[index] = 0
		push!(A_plusSP[dummyorig], index)
		index += 1
	end

	for n in Destination[i]
		arcsSP[n, dummydest] = index
		push!(A_plusSP[n], index)
		arcLookupSP[index] = (n, dummydest) 
		cSP[index] = 0
		push!(A_minusSP[n], index)
		index += 1
	end

	#Initialize shortest path algorithm (Dijkstra's)
	visitednodes = zeros(numnodesSP)
	currdistance = repeat([999999999.0],outer=[numnodesSP])
	currdistance[dummyorig] = 0
	currnode = dummyorig
	prevnode, prevarc = zeros(numnodesSP), zeros(numnodesSP)
	nopathexistsflag = 0
	
	#Find shortest path from dummy origin to dummy destination
	while (visitednodes[dummydest] == 0) & (nopathexistsflag == 0)

		#Assess all neighbors of current node
		for a in A_plusSP[currnode]
			n = arcLookupSP[a][2]
			if visitednodes[n] == 0

				newdist = currdistance[currnode] + cSP[a]
				
				if newdist < currdistance[n]
					currdistance[n] = newdist
					prevnode[n] = currnode
					prevarc[n] = a
				end
			end
		end

		#Mark the current node as visited
		visitednodes[currnode] = 1

		#Update the current node 
		currdistance_unvisited = copy(currdistance)
		for n in 1:numnodesSP
			if visitednodes[n] == 1
				currdistance_unvisited[n] = 999999999
			end
		end
		currnode = argmin(currdistance_unvisited)

		#If all remaining nodes have tentative distance of 999999999 and the algorithm has not terminated, then there is no path from origin to destination
		if minimum(currdistance_unvisited) == 999999999
			nopathexistsflag = 1
		end

	end

	#if nopathexistsflag == 1
	#	println("==================================================")
	#	println("==================================================")
	#	println("nopathexistsflag = ", nopathexistsflag, " for order $i")
	#	println("==================================================")
	#	println("==================================================")
	#end

	#Format the shortest path output
	patharcs = [0 for a in 1:numarcs]
	node = Int(prevnode[dummydest])

	while Int(prevnode[node]) != dummyorig
		patharcs[Int(prevarc[node])] = 1
		node = Int(prevnode[node])
	end

	#Format the shortest path output
	shortestpathnodes_rev = [dummydest]
	shortestpatharcs_rev = []
	node = dummydest
	while node != dummyorig
		push!(shortestpatharcs_rev, Int(prevarc[node]))
		node = Int(prevnode[node])
		push!(shortestpathnodes_rev, node)
	end
	shortestpathnodes = reverse(shortestpathnodes_rev[2:length(shortestpathnodes_rev)-1]) 
	shortestpatharcs = reverse(shortestpatharcs_rev[2:length(shortestpatharcs_rev)-1]) 

	return currdistance[dummydest], patharcs, shortestpathnodes, shortestpatharcs

end

#-----------------------------------------------------------------------#

function findshortestpath_orderdelivery(i, arccosts, numnodes, numarcs, arcs, arcLookup, A_plus, A_minus, Origin, Destination, nodesLookup, A_space, tstep)

	#Create dummy origin and destination nodes + arcs 
	dummyorig = numnodes + 1
	dummydest = numnodes + 2
	numnodesSP = numnodes + 2
	arcsSP, arcLookupSP, A_plusSP, A_minusSP, nodesLookupSP = deepcopy(arcs), deepcopy(arcLookup), deepcopy(A_plus), deepcopy(A_minus), deepcopy(nodesLookup)
	cSP = Dict()
	for a in 1:numarcs
		cSP[a] = arccosts[a]
	end

	A_plusSP[dummyorig], A_plusSP[dummydest] = [], []
	nodesLookupSP[dummyorig], nodesLookupSP[dummydest] = (0,0), (0,0)

	#Remove the time arcs in Origin[i] (i.e. can't leave late from pick up window)
	for n in Origin[i]
		n2, t2 = nodesLookup[n][1], nodesLookup[n][2] + tstep
		if t2 <= horizon
			a = arcs[n, nodes[n2, t2]]
			remove!(A_plusSP[n], a)
		end
	end

	#"Remove" too long arcs
	for a in 1:numarcs
		if cSP[a] > 12
			cSP[a] = 1000
		end
	end

	index = numarcs + 1

	for j in 1:length(Origin[i])
		n = Origin[i][j]
		arcsSP[dummyorig, n] = index
		push!(A_minusSP[n], index)
		arcLookupSP[index] = (dummyorig, n)
		cSP[index] = tstep * (j-1) #Cost of the arc is the time from the start of the pickup window
		push!(A_plusSP[dummyorig], index)
		index += 1
	end

	for n in Destination[i]
		arcsSP[n, dummydest] = index
		push!(A_plusSP[n], index)
		arcLookupSP[index] = (n, dummydest) 
		cSP[index] = 0
		push!(A_minusSP[n], index)
		index += 1
	end

	#Initialize shortest path algorithm (Dijkstra's)
	visitednodes = zeros(numnodesSP)
	currdistance = repeat([999999999.0],outer=[numnodesSP])
	currdistance[dummyorig] = 0
	currnode = dummyorig
	prevnode, prevarc = zeros(numnodesSP), zeros(numnodesSP)
	nopathexistsflag = 0
	
	#Find shortest path from dummy origin to dummy destination
	while (visitednodes[dummydest] == 0) & (nopathexistsflag == 0)

		#Assess all neighbors of current node
		for a in A_plusSP[currnode]
			n = arcLookupSP[a][2]
			if visitednodes[n] == 0

				newdist = currdistance[currnode] + cSP[a]
				
				if newdist < currdistance[n]
					currdistance[n] = newdist
					prevnode[n] = currnode
					prevarc[n] = a
				end
			end
		end

		#Mark the current node as visited
		visitednodes[currnode] = 1

		#Update the current node 
		currdistance_unvisited = copy(currdistance)
		for n in 1:numnodesSP
			if visitednodes[n] == 1
				currdistance_unvisited[n] = 999999999
			end
		end
		currnode = argmin(currdistance_unvisited)

		#If all remaining nodes have tentative distance of 999999999 and the algorithm has not terminated, then there is no path from origin to destination
		if minimum(currdistance_unvisited) == 999999999
			nopathexistsflag = 1
		end

	end

	#if nopathexistsflag == 1
	#	println("==================================================")
	#	println("==================================================")
	#	println("nopathexistsflag = ", nopathexistsflag, " for order $i")
	#	println("==================================================")
	#	println("==================================================")
	#end

	#Format the shortest path output
	patharcs = [0 for a in 1:numarcs]
	node = Int(prevnode[dummydest])

	while Int(prevnode[node]) != dummyorig
		patharcs[Int(prevarc[node])] = 1
		node = Int(prevnode[node])
	end

	#Format the shortest path output
	shortestpathnodes_rev = [dummydest]
	shortestpatharcs_rev = []
	node = dummydest
	while node != dummyorig
		push!(shortestpatharcs_rev, Int(prevarc[node]))
		node = Int(prevnode[node])
		push!(shortestpathnodes_rev, node)
	end
	shortestpathnodes = reverse(shortestpathnodes_rev[2:length(shortestpathnodes_rev)-1]) 
	shortestpatharcs = reverse(shortestpatharcs_rev[2:length(shortestpatharcs_rev)-1]) 

	return currdistance[dummydest], patharcs, shortestpathnodes, shortestpatharcs

end

#-----------------------------------------------------------------------#

function findshortestpath_print(i, arccosts, numnodes, numarcs, arcs, arcLookup, A_plus, A_minus, Origin, Destination, nodesLookup, A_space, tstep)

	#Create dummy origin and destination nodes + arcs 
	dummyorig = numnodes + 1
	dummydest = numnodes + 2
	numnodesSP = numnodes + 2
	arcsSP, arcLookupSP, A_plusSP, A_minusSP, nodesLookupSP = deepcopy(arcs), deepcopy(arcLookup), deepcopy(A_plus), deepcopy(A_minus), deepcopy(nodesLookup)
	cSP = Dict()
	for a in 1:numarcs
		cSP[a] = arccosts[a]
	end

	A_plusSP[dummyorig], A_plusSP[dummydest] = [], []
	nodesLookupSP[dummyorig], nodesLookupSP[dummydest] = (0,0), (0,0)

	#Remove the time arcs in Origin[i] (i.e. can't leave late from pick up window)
	for n in Origin[i]
		n2, t2 = nodesLookup[n][1], nodesLookup[n][2] + tstep
		a = arcs[n, nodes[n2, t2]]
		remove!(A_plusSP[n], a)
	end

	index = numarcs + 1

	for n in Origin[i]
		arcsSP[dummyorig, n] = index
		push!(A_minusSP[n], index)
		arcLookupSP[index] = (dummyorig, n)
		cSP[index] = 0
		push!(A_plusSP[dummyorig], index)
		index += 1
	end

	for n in Destination[i]
		arcsSP[n, dummydest] = index
		push!(A_plusSP[n], index)
		arcLookupSP[index] = (n, dummydest) 
		cSP[index] = 0
		push!(A_minusSP[n], index)
		index += 1
	end

	#Initialize shortest path algorithm (Dijkstra's)
	visitednodes = zeros(numnodesSP)
	currdistance = repeat([999999999.0],outer=[numnodesSP])
	currdistance[dummyorig] = 0
	currnode = dummyorig
	prevnode, prevarc = zeros(numnodesSP), zeros(numnodesSP)
	nopathexistsflag = 0
	
	#Find shortest path from dummy origin to dummy destination
	while (visitednodes[dummydest] == 0) & (nopathexistsflag == 0)
		if i == 8
			println("----------------------------------------------")
			println("currnode = ", currnode)
		end

		#Assess all neighbors of current node
		if i == 8
			println("A_plusSP[currnode] = ", A_plusSP[currnode])
		end
		for a in A_plusSP[currnode]
			n = arcLookupSP[a][2]
			if i == 8
				println("- - - - - - -")
				println("n  = ", n )
				println("A_plusSP[currnode] = ", A_plusSP[currnode])
				println("visitednodes[n] = ", visitednodes[n])
			end
			if visitednodes[n] == 0

				newdist = currdistance[currnode] + cSP[a]
				if i == 8
					println("newdist = ", newdist)
					println("currdistance[n] = ", currdistance[n])
				end
				if newdist < currdistance[n]
					currdistance[n] = newdist
					prevnode[n] = currnode
					prevarc[n] = a
				end
				if i == 8
					println("updated currdistance[n] ", currdistance[n])
				end
			end
		end

		#Mark the current node as visited
		visitednodes[currnode] = 1

		#Update the current node 
		currdistance_unvisited = copy(currdistance)
		for n in 1:numnodesSP
			if visitednodes[n] == 1
				currdistance_unvisited[n] = 999999999
			end
		end
		currnode = argmin(currdistance_unvisited)

		#If all remaining nodes have tentative distance of 999999 and the algorithm has not terminated, then there is no path from origin to destination
		if minimum(currdistance_unvisited) == 999999999
			nopathexistsflag = 1
		end

	end

	if nopathexistsflag == 1
		println("==================================================")
		println("==================================================")
		println("nopathexistsflag = ", nopathexistsflag, " for order $i")
		println("==================================================")
		println("==================================================")
	end

	#Format the shortest path output
	patharcs = [0 for a in 1:numarcs]
	node = Int(prevnode[dummydest])

	while Int(prevnode[node]) != dummyorig
		patharcs[Int(prevarc[node])] = 1
		node = Int(prevnode[node])
	end

	#Format the shortest path output
	shortestpathnodes_rev = [dummydest]
	shortestpatharcs_rev = []
	node = dummydest
	while node != dummyorig
		push!(shortestpatharcs_rev, Int(prevarc[node]))
		node = Int(prevnode[node])
		push!(shortestpathnodes_rev, node)
	end
	shortestpathnodes = reverse(shortestpathnodes_rev[2:length(shortestpathnodes_rev)-1]) 
	shortestpatharcs = reverse(shortestpatharcs_rev[2:length(shortestpatharcs_rev)-1]) 

	return currdistance[dummydest], patharcs, shortestpathnodes, shortestpatharcs

end