
#---------------------------------------------------------------------------------------#
#-------------------------------------WRITE RESULTS-------------------------------------#  
#---------------------------------------------------------------------------------------#

function writeresults_abcg_online(filename, cg_iter, rmp_obj, rmp_time, pp_time, pp_time_par, x, y, z, w, ordtime, currtime, minreducedcost)

	bigOrderArcSet = []
	for i in orders
       bigOrderArcSet = union(bigOrderArcSet, orderArcSet[i])
    end

	#Calculate necessary quantities
	totalorders = highestorderindex
	completedorders = totalorders - length(orders)
	newordersthisiter = newordersbyiter[currtime]

	obj_delay = sum((getvalue(ordtime[i]) - shortesttriptimes[i])/shortesttriptimes[i] for i in orders) 
	obj_dist = sum(sum(c[a]*getvalue(x[i,a]) for a in orderArcSet[i]) for i in orders) + sum(c[a]*getvalue(y[a]) for a in A_hasdriver) + sum(u[a]*getvalue(w[a]) for a in A_space) 
	infeasible_order_count = sum(getvalue(x[i,dummyarc]) for i in orders)
	if intersect(bigOrderArcSet, [a for a in numarcs+1:extendednumarcs]) == []
		incomplete_order_count = 0
	else
		incomplete_order_count = sum(sum(getvalue(x[i,a]) for a in intersect(orderArcSet[i], [a for a in numarcs+1:extendednumarcs])) for i in orders if intersect(orderArcSet[i], [a for a in numarcs+1:extendednumarcs]) != [])
	end

	delay_per_order = total_delay/completedorders
	#avg_deviation_pct_from_shortestpath = total_deviation_pct_from_shortestpath/completedorders
	totalnightsathome = sum(sum(getvalue(z[d,arcs[nodes[(driverHomeLocs[d],t2)], nodes[(driverHomeLocs[d],t2+tstep)]]]) for t2 in T_off_0[d]) for d in drivers)
	totalnightsaway = sum(length(T_off_0[d]) for d in drivers) - sum(sum(getvalue(z[d,arcs[nodes[(driverHomeLocs[d],t2)], nodes[(driverHomeLocs[d],t2+tstep)]]]) for t2 in T_off_0[d]) for d in drivers)

	#----------------------------WRITE ORDER TIMES TO CSV----------------------------#

	df = DataFrame(ID = [runid],
			example = [ex],
			timehorizon = [horizon], 
			timestep = [tstep], 
			numlocs = [numlocs], 
			numdrivers = [length(drivers)], 
			numtrucks = [numtrucks], 
			numarcs = [numarcs], 
			lambdaobj = [lambda],
			timedelta = [currtime],
			totalcompletedorders = [completedorders],
			neworders = [newordersthisiter],
			numorders = [length(orders)], 
			abcgiteration = [cg_iter],
			heuristicdp = [solvedpheuristically_flag_now],
			totalabcgiteration = [""],
			minreducedcost = [minreducedcost],
			cumuloptval = [""],
			totalpastcost = [totalpastcost],
			optval = [rmp_obj], 
			optval_delay = [obj_delay], 
			optval_dist = [obj_dist], 
			infeasible_order_count = [infeasible_order_count],
			incomplete_order_count = [incomplete_order_count],
			ordertrips = [totalordertrips], 
			emptytrips = [totalemptytrips], 
			taxitrips = [totaltaxitrips],
			ordermiles = [totalordermiles], 
			emptymiles = [totalemptymiles], 
			taximiles = [totaltaximiles], 
			delay_per_order = [delay_per_order], 
			max_order_delay = [max_delay],
			#avg_deviation_pct_from_shortestpath = [avg_deviation_pct_from_shortestpath], 
			#max_deviation_pct_from_shortestpath = [max_deviation_pct_from_shortestpath],
			nightsathome = [totalnightsathome],
			nightsaway = [totalnightsaway],
			ip_time = [""],
			rmp_time = [rmp_time],
			pp_time = [pp_time],
			pptime_parallel = [pp_time_par],
			order_arc_count = [sum(length(orderArcSet[i]) for i in orders)]
           )

	if (currtime == 0) & (cg_iter == 1)
		CSV.write(filename, df)
	else
		CSV.write(filename, df, append=true)
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function writeresults_ip_online(filename, total_cg_iter, ip_obj, ip_time, cg_rmptimes, cg_pptimes, cg_pptimes_par, x, y, z, w, ordtime, currtime)

	bigOrderArcSet = []
	for i in orders
       bigOrderArcSet = union(bigOrderArcSet, orderArcSet[i])
    end

	#Calculate necessary quantities
	totalorders = highestorderindex
	completedorders = totalorders - length(orders)
	newordersthisiter = newordersbyiter[currtime]

	obj_delay = sum((getvalue(ordtime[i]) - shortesttriptimes[i])/shortesttriptimes[i] for i in orders) 
	obj_dist = sum(sum(c[a]*getvalue(x[i,a]) for a in orderArcSet[i]) for i in orders) + sum(c[a]*getvalue(y[a]) for a in A_hasdriver) + sum(u[a]*getvalue(w[a]) for a in A_space) 
	infeasible_order_count = sum(getvalue(x[i,dummyarc]) for i in orders)
	if intersect(bigOrderArcSet, [a for a in numarcs+1:extendednumarcs]) == []
		incomplete_order_count = 0
	else
		incomplete_order_count = sum(sum(getvalue(x[i,a]) for a in intersect(orderArcSet[i], [a for a in numarcs+1:extendednumarcs])) for i in orders if intersect(orderArcSet[i], [a for a in numarcs+1:extendednumarcs]) != [])
	end

	delay_per_order = total_delay/completedorders
	totalnightsathome = sum(sum(getvalue(z[d,arcs[nodes[(driverHomeLocs[d],t2)], nodes[(driverHomeLocs[d],t2+tstep)]]]) for t2 in T_off_0[d]) for d in drivers)
	totalnightsaway = sum(length(T_off_0[d]) for d in drivers) - sum(sum(getvalue(z[d,arcs[nodes[(driverHomeLocs[d],t2)], nodes[(driverHomeLocs[d],t2+tstep)]]]) for t2 in T_off_0[d]) for d in drivers)

	rmp_time = sum(cg_rmptimes)
	cg_pp_time = sum(cg_pptimes)
	cg_pptime_parallel = sum(cg_pptimes_par)

	orderarccnt = sum(length(orderArcSet[i]) for i in orders)

	#----------------------------SAVE TIMES FOR REPOSRTING---------------------------#

	push!(totalcgiterlist, total_cg_iter)
	push!(iptimelistlist, ip_time)
	push!(rmptimelist, rmp_time)
	push!(cgpptimelist, cg_pp_time)
	push!(cgpptimeparallellist, cg_pptime_parallel)
	push!(orderarccountlist, orderarccnt)

	#----------------------------WRITE ORDER TIMES TO CSV----------------------------#

	df = DataFrame(ID = [runid],
			example = [ex],
			timehorizon = [horizon], 
			timestep = [tstep], 
			numlocs = [numlocs], 
			numdrivers = [length(drivers)], 
			numtrucks = [numtrucks], 
			numarcs = [numarcs], 
			lambdaobj = [lambda],
			timedelta = [currtime],
			totalcompletedorders = [completedorders],
			neworders = [newordersthisiter],
			numorders = [length(orders)], 
			abcgiteration = ["IP"],
			heuristicdp = [solvedpheuristically_flag_now],
			totalabcgiteration = [total_cg_iter],
			minreducedcost = [""],
			cumuloptval = [""],
			totalpastcost = [totalpastcost],
			optval = [ip_obj], 
			optval_delay = [obj_delay], 
			optval_dist = [obj_dist], 
			infeasible_order_count = [infeasible_order_count],
			incomplete_order_count = [incomplete_order_count],
			ordertrips = [totalordertrips], 
			emptytrips = [totalemptytrips], 
			taxitrips = [totaltaxitrips],
			ordermiles = [totalordermiles], 
			emptymiles = [totalemptymiles], 
			taximiles = [totaltaximiles], 
			delay_per_order = [delay_per_order], 
			max_order_delay = [max_delay],
			nightsathome = [totalnightsathome],
			nightsaway = [totalnightsaway],
			ip_time = [ip_time],
			cg_rmp_time = [rmp_time],
			cg_pp_time = [cg_pp_time],
			cg_pptime_parallel = [cg_pptime_parallel],
			order_arc_count = [orderarccnt]
           )

	CSV.write(filename, df, append=true)

end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function writeresults_final_online(filename, currtime)

	delay_per_order = total_delay/highestorderindex

	#----------------------------WRITE ORDER TIMES TO CSV----------------------------#

	df = DataFrame(ID = [runid],
			example = [ex],
			timehorizon = [horizon], 
			timestep = [tstep], 
			numlocs = [numlocs], 
			numdrivers = [length(drivers)], 
			numtrucks = [numtrucks], 
			numarcs = [numarcs], 
			lambdaobj = [lambda],
			timedelta = [currtime],
			totalcompletedorders = [highestorderindex],
			neworders = [""],
			numorders = [""], 
			abcgiteration = ["final"],
			heuristicdp = [""],
			totalabcgiteration = [""],
			minreducedcost = [""],
			cumuloptval = [totalpastcost],
			totalpastcost = [totalpastcost],
			optval = [""], 
			optval_delay = [""], 
			optval_dist = [""], 
			infeasible_order_count = [""],
			incomplete_order_count = [""],
			ordertrips = [""], 
			emptytrips = [""], 
			taxitrips = [""],
			ordermiles = [totalordermiles], 
			emptymiles = [""], 
			taximiles = [""], 
			delay_per_order = [delay_per_order], 
			max_order_delay = [max_delay],
			nightsathome = [""],
			nightsaway = [""],
			ip_time = [""],
			cg_rmp_time = [""],
			cg_pp_time = [""],
			cg_pptime_parallel = [""],
			order_arc_count = [""]
           )

	CSV.write(filename, df, append=true)

end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function writeresults_otd(filename, ip_obj, totaltime_routeorders, totaltime_assigndrivers, restrictedArcSet, x, y, z, w, ordtime, currtime)

	bigOrderArcSet = []
	for i in orders
       bigOrderArcSet = union(bigOrderArcSet, restrictedArcSet[i])
    end

	#Calculate necessary quantities
	totalorders = highestorderindex
	completedorders = totalorders - length(orders)
	newordersthisiter = newordersbyiter[currtime]

	obj_delay = sum((getvalue(ordtime[i]) - shortesttriptimes[i])/shortesttriptimes[i] for i in orders) 
	obj_dist = sum(sum(c[a]*getvalue(x[i,a]) for a in restrictedArcSet[i]) for i in orders) + sum(c[a]*getvalue(y[a]) for a in A_hasdriver) + sum(u[a]*getvalue(w[a]) for a in A_space) 
	infeasible_order_count = sum(getvalue(x[i,dummyarc]) for i in orders)
	if intersect(bigOrderArcSet, [a for a in numarcs+1:extendednumarcs]) == []
		incomplete_order_count = 0
	else
		incomplete_order_count = sum(sum(getvalue(x[i,a]) for a in intersect(restrictedArcSet[i], [a for a in numarcs+1:extendednumarcs])) for i in orders if intersect(restrictedArcSet[i], [a for a in numarcs+1:extendednumarcs]) != [])
	end

	delay_per_order = total_delay/completedorders

	totalnightsathome = sum(sum(getvalue(z[d,arcs[nodes[(driverHomeLocs[d],t2)], nodes[(driverHomeLocs[d],t2+tstep)]]]) for t2 in T_off_0[d]) for d in drivers)
	totalnightsaway = sum(length(T_off_0[d]) for d in drivers) - sum(sum(getvalue(z[d,arcs[nodes[(driverHomeLocs[d],t2)], nodes[(driverHomeLocs[d],t2+tstep)]]]) for t2 in T_off_0[d]) for d in drivers)

	#----------------------------WRITE ORDER TIMES TO CSV----------------------------#

	df = DataFrame(ID = [runid],
			example = [ex],
			timehorizon = [horizon], 
			timestep = [tstep], 
			numlocs = [numlocs], 
			numdrivers = [length(drivers)], 
			numtrucks = [numtrucks], 
			numarcs = [numarcs], 
			lambdaobj = [lambda],
			timedelta = [currtime],
			totalcompletedorders = [completedorders],
			neworders = [newordersthisiter],
			numorders = [length(orders)], 
			iteration = ["drivers"],
			cumuloptval = [totalpastcost + ip_obj],
			totalpastcost = [totalpastcost],
			optval = [ip_obj], 
			optval_delay = [obj_delay], 
			optval_dist = [obj_dist], 
			infeasible_order_count = [infeasible_order_count],
			incomplete_order_count = [incomplete_order_count],
			ordertrips = [totalordertrips], 
			emptytrips = [totalemptytrips], 
			taxitrips = [totaltaxitrips],
			ordermiles = [totalordermiles], 
			emptymiles = [totalemptymiles], 
			taximiles = [totaltaximiles], 
			delay_per_order = [delay_per_order], 
			max_order_delay = [max_delay],
			nightsathome = [totalnightsathome],
			nightsaway = [totalnightsaway],
			orders_ip_time = [totaltime_routeorders],
			drivers_ip_time = [totaltime_assigndrivers],
			order_arc_count = [sum(length(restrictedArcSet[i]) for i in orders)]
           )

	if currtime == 0
		CSV.write(filename, df)
	else
		CSV.write(filename, df, append=true)
	end

end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function writeresults_rr(filename, ip_obj, totaltime_assigndrivers, restrictedArcSet, x, y, z, w, ordtime, currtime)

	bigOrderArcSet = []
	for i in orders
       bigOrderArcSet = union(bigOrderArcSet, restrictedArcSet[i])
    end

	#Calculate necessary quantities
	totalorders = highestorderindex
	completedorders = totalorders - length(orders)
	newordersthisiter = newordersbyiter[currtime]

	obj_delay = sum((getvalue(ordtime[i]) - shortesttriptimes[i])/shortesttriptimes[i] for i in orders) 
	obj_dist = sum(sum(c[a]*getvalue(x[i,a]) for a in restrictedArcSet[i]) for i in orders) + sum(c[a]*getvalue(y[a]) for a in A_hasdriver) + sum(u[a]*getvalue(w[a]) for a in A_space) 
	infeasible_order_count = sum(getvalue(x[i,dummyarc]) for i in orders)
	if intersect(bigOrderArcSet, [a for a in numarcs+1:extendednumarcs]) == []
		incomplete_order_count = 0
	else
		incomplete_order_count = sum(sum(getvalue(x[i,a]) for a in intersect(restrictedArcSet[i], [a for a in numarcs+1:extendednumarcs])) for i in orders if intersect(restrictedArcSet[i], [a for a in numarcs+1:extendednumarcs]) != [])
	end

	delay_per_order = total_delay/completedorders

	totalnightsathome = sum(sum(getvalue(z[d,arcs[nodes[(driverHomeLocs[d],t2)], nodes[(driverHomeLocs[d],t2+tstep)]]]) for t2 in T_off_0[d]) for d in drivers)
	totalnightsaway = sum(length(T_off_0[d]) for d in drivers) - sum(sum(getvalue(z[d,arcs[nodes[(driverHomeLocs[d],t2)], nodes[(driverHomeLocs[d],t2+tstep)]]]) for t2 in T_off_0[d]) for d in drivers)

	#----------------------------WRITE ORDER TIMES TO CSV----------------------------#

	df = DataFrame(ID = [runid],
			example = [ex],
			timehorizon = [horizon], 
			timestep = [tstep], 
			numlocs = [numlocs], 
			numdrivers = [length(drivers)], 
			numtrucks = [numtrucks], 
			numarcs = [numarcs], 
			lambdaobj = [lambda],
			timedelta = [currtime],
			totalcompletedorders = [completedorders],
			neworders = [newordersthisiter],
			numorders = [length(orders)], 
			iteration = ["drivers"],
			cumuloptval = [totalpastcost + ip_obj],
			totalpastcost = [totalpastcost],
			optval = [ip_obj], 
			optval_delay = [obj_delay], 
			optval_dist = [obj_dist], 
			infeasible_order_count = [infeasible_order_count],
			incomplete_order_count = [incomplete_order_count],
			ordertrips = [totalordertrips], 
			emptytrips = [totalemptytrips], 
			taxitrips = [totaltaxitrips],
			ordermiles = [totalordermiles], 
			emptymiles = [totalemptymiles], 
			taximiles = [totaltaximiles], 
			delay_per_order = [delay_per_order], 
			max_order_delay = [max_delay],
			nightsathome = [totalnightsathome],
			nightsaway = [totalnightsaway],
			ip_time = [totaltime_assigndrivers],
			order_arc_count = [sum(length(restrictedArcSet[i]) for i in orders)]
           )

	if currtime == 0
		CSV.write(filename, df)
	else
		CSV.write(filename, df, append=true)
	end

end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function writeresults_bns(filename, currtime, iteration)

	#Calculate necessary quantities
	totalorders = highestorderindex
	completedorders = totalorders - length(orders)
	newordersthisiter = newordersbyiter[currtime]

	delay_per_order = total_delay/completedorders
	
	#----------------------------WRITE ORDER TIMES TO CSV----------------------------#

	df = DataFrame(ID = [runid],
			example = [ex],
			timehorizon = [horizon/24], 
			timestep = [tstep], 
			numlocs = [numlocs], 
			numdrivers = [length(drivers)], 
			numtrucks = [numtrucks], 
			lambdaobj = [lambda],
			timedelta = [currtime],
			totalcompletedorders = [completedorders],
			neworders = [newordersthisiter],
			numorders = [length(orders)], 
			iteration = [iteration],
			totalpastcost = [totalpastcost],
			ordertrips = [totalordertrips], 
			emptytrips = [totalemptytrips], 
			taxitrips = [totaltaxitrips],
			ordermiles = [totalordermiles], 
			emptymiles = [totalemptymiles], 
			taximiles = [totaltaximiles], 
			delay_per_order = [delay_per_order], 
			max_order_delay = [max_delay]
           )

	if currtime == 0
		CSV.write(filename, df)
	else
		CSV.write(filename, df, append=true)
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#  

function writeorderoutcomesfile(orderoutcomesfilename, solutionmethod, currtime)

	df = DataFrame(ID = [runid for i in 1:highestorderindex],
		example = [ex for i in 1:highestorderindex],
		method = [solutionmethod for i in 1:highestorderindex],
		orderid = [i for i in 1:highestorderindex],
		orderdist = ordermilesoutcomes[1:highestorderindex],
		orderpenaltydist = ordermilespenalty[1:highestorderindex], 
		orderdeliverytime = orderdelayoutcomes[1:highestorderindex],
		shortesttriptime = shortesttriptimes[1:highestorderindex]
       )

	CSV.write(orderoutcomesfilename, df)

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#  

function writeorderoutcomes_byiter(orderoutcomesbyiterfilename, currtime, x, y, z, w, ordtime, restrictedArcSet)

	infeas_interim, orderdist_interim, orderpenaltydist_interim, orderdeliverytime_interim, orderdelay_interim, orderids_interim = [], [], [], [], [], []
	orderdist_complete, orderpenaltydist_complete, ordercompleted_flag = [], [], []

	for i in 1:highestorderindex
		if i in orders
			push!(infeas_interim, getvalue(x[i, dummyarc]) )
			if intersect(restrictedArcSet[i], 1:numarcs) == []
				push!(orderdist_interim, ordermilesoutcomes[i])
			else
				push!(orderdist_interim, ordermilesoutcomes[i] + sum(c[a] * getvalue(x[i,a]) for a in intersect(restrictedArcSet[i], 1:numarcs) ) )
			end
			if intersect(restrictedArcSet[i], numarcs+1:extendednumarcs) == []
				push!(orderpenaltydist_interim, ordermilespenalty[i] )
			else
				push!(orderpenaltydist_interim, ordermilespenalty[i] + sum(c[a] * getvalue(x[i,a]) for a in intersect(restrictedArcSet[i], numarcs+1:extendednumarcs)))
			end
			push!(orderdeliverytime_interim, getvalue(ordtime[i]) )
			push!(orderdelay_interim, (getvalue(ordtime[i]) - shortesttriptimes[i])/shortesttriptimes[i] )
			push!(orderids_interim, i)
			push!(orderdist_complete, ordermilesoutcomes[i])
			push!(orderpenaltydist_complete, ordermilespenalty[i])
			push!(ordercompleted_flag, 0)
		else
			push!(infeas_interim, 0)
			push!(orderdist_interim, ordermilesoutcomes[i])
			push!(orderpenaltydist_interim, ordermilespenalty[i])
			push!(orderdeliverytime_interim, orderdelayoutcomes[i])
			push!(orderdelay_interim, (orderdelayoutcomes[i] - shortesttriptimes[i])/shortesttriptimes[i] )
			push!(orderids_interim, i)
			push!(orderdist_complete, ordermilesoutcomes[i])
			push!(orderpenaltydist_complete, ordermilespenalty[i])
			push!(ordercompleted_flag, 1)
		end
	end

	#Add empty row
	push!(orderids_interim, "empty")
	push!(infeas_interim, 0)
	push!(orderdist_interim, totalemptymiles + sum(c[a] * getvalue(y[a]) for a in intersect(A_hasdriver, 1:numarcs) ))
	push!(orderpenaltydist_interim, 0)
	push!(orderdelay_interim, 0)
	push!(orderdeliverytime_interim, 0)
	push!(orderdist_complete, totalemptymiles)
	push!(orderpenaltydist_complete, 0)
	push!(ordercompleted_flag, 0)

	#Add taxi row
	push!(orderids_interim, "taxi")
	push!(infeas_interim, 0)
	push!(orderdist_interim, totaltaximiles + sum(u[a] * getvalue(w[a]) for a in intersect(A_space, 1:numarcs) ))
	push!(orderpenaltydist_interim, 0)
	push!(orderdelay_interim, 0)
	push!(orderdeliverytime_interim, 0)
	push!(orderdist_complete, totaltaximiles)
	push!(orderpenaltydist_complete, 0)
	push!(ordercompleted_flag, 0)

	df = DataFrame(ID = [runid for i in 1:highestorderindex+2],
		example = [ex for i in 1:highestorderindex+2],
		method = ["abcg" for i in 1:highestorderindex+2],
		orderid = orderids_interim,
		iteration = [currtime for i in 1:highestorderindex+2],
		ordercompleted_flag = ordercompleted_flag,
		infeasibleflag = infeas_interim,
		orderdist = orderdist_interim,
		orderdist_complete = orderdist_complete,
		orderpenaltydist = orderpenaltydist_interim,
		orderpenaltydist_complete = orderpenaltydist_complete,
		orderdeliverytime = orderdeliverytime_interim,
		orderdelay = orderdelay_interim
       )

	if currtime == 0
		CSV.write(orderoutcomesbyiterfilename, df)
	else
		CSV.write(orderoutcomesbyiterfilename, df, append=true)
	end

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#

function writecorridorcounterfile(solutionmethod)

	segmentcounterDict = Dict()
	for pa in prearcs
		o, d, rddtime, rawtime, miles = pa
		segmentcounterDict[o, d] = [rddtime, rawtime, miles, 0, 0, 0, 0]
	end
	for sgmt in pastordersegments
		o, d = sgmt[4], sgmt[5]
		if o != d
			segmentcounterDict[o, d][4] += 1
			segmentcounterDict[o, d][5] += 1
		end
	end
	for sgmt in pastemptysegments
		ttltrucks, o, d = sgmt[1], sgmt[4], sgmt[5]
		if o != d
			segmentcounterDict[o, d][4] += ttltrucks
			segmentcounterDict[o, d][6] += ttltrucks
		end
	end
	for sgmt in pasttaxisegments
		ttltaxis, o, d = sgmt[1], sgmt[4], sgmt[5]
		if o != d
			segmentcounterDict[o, d][4] += ttltaxis
			segmentcounterDict[o, d][7] += ttltaxis
		end
	end

	df = DataFrame(ID = [runid for a in 1:length(segmentcounterDict)],
				example = [ex for a in 1:length(segmentcounterDict)],
				method = [solutionmethod for a in 1:length(segmentcounterDict)],
				originloc = [item[1][1] for item in segmentcounterDict],
				destloc = [item[1][2] for item in segmentcounterDict],
				arctime_rdd = [item[2][1] for item in segmentcounterDict],
				arctime_raw = [item[2][2] for item in segmentcounterDict],
				arcdist = [item[2][3] for item in segmentcounterDict],
				totaltrips = [item[2][4] for item in segmentcounterDict],
				ordertrips = [item[2][5] for item in segmentcounterDict],
				emptytrips = [item[2][6] for item in segmentcounterDict],
				taxitrips = [item[2][7] for item in segmentcounterDict]
	           )

	CSV.write(corridorfilename, df)

	#---------------------------------------------------------------------------------------#

	SPsegmentcounterDict = Dict()
	for pa in prearcs
		o, d, rddtime, rawtime, miles = pa
		SPsegmentcounterDict[o, d] = [rddtime, rawtime, miles, 0, 0, 0, 0]
	end

	for i in 1:length(originloc)
		arclist = shortestpatharclists[originloc[i], destloc[i]]
		for item in arclist
			if item[1] != item[2]
				SPsegmentcounterDict[item[1], item[2]][4] += 1
				SPsegmentcounterDict[item[1], item[2]][5] += 1
			end
		end
	end

	df2 = DataFrame(ID = [runid for a in 1:length(SPsegmentcounterDict)],
				example = [ex for a in 1:length(SPsegmentcounterDict)],
				method = [solutionmethod for a in 1:length(SPsegmentcounterDict)],
				originloc = [item[1][1] for item in SPsegmentcounterDict],
				destloc = [item[1][2] for item in SPsegmentcounterDict],
				arctime_rdd = [item[2][1] for item in SPsegmentcounterDict],
				arctime_raw = [item[2][2] for item in SPsegmentcounterDict],
				arcdist = [item[2][3] for item in SPsegmentcounterDict],
				totaltrips = [item[2][4] for item in SPsegmentcounterDict],
				ordertrips = [item[2][5] for item in SPsegmentcounterDict],
				emptytrips = [item[2][6] for item in SPsegmentcounterDict],
				taxitrips = [item[2][7] for item in SPsegmentcounterDict]
	           )

	CSV.write(spcorridorfilename, df2)

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#

function writefiguresfile(heuristicfigurefilename)

	df = DataFrame(ID = [runid],
			Instance = [ex],
			Method = [solutionmethod],			
			TotalWeeklyCost = [totalpastcost],	
			OrdersCompletedInHorizon = [highestorderindex - length(orders)],	
			OrderMiles = [totalordermiles],
			EmptyMiles = [totalemptymiles],
			TaxiMiles = [totaltaximiles],
			AverageDelayAsPctOfShortestPath = [total_delay/highestorderindex],
			MaxDelay = [max_delay],
			AvgNightsHomePerDriver = [sum(drivernightshome[d] for d in drivers)/length(drivers)],	
			AvgNightsAwayPerDriver = [sum(drivernightsaway[d] for d in drivers)/length(drivers)],		
			TotalDriverHours = [totaldriverhours]
			)

	CSV.write(heuristicfigurefilename, df) 

end

#---------------------------------------------------------------------------------------#  
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function writecomputationaltimesfile(filename, currtime)

	#--------------------------------------------------------------------------------#

	df = DataFrame(ID = [runid],
			example = [ex],
			timehorizon = [horizon], 
			timestep = [tstep], 
			lambdaobj = [lambda],
			itertimedelta = [timedelta],
			totalonlineiter = [numiterations_online],
			totalpastcost = [totalpastcost],
			meanabcgiteration = [mean(totalcgiterlist)],
			totaliptime = [sum(iptimelistlist)],
			totalcgrmptime = [sum(rmptimelist)],
			totalcgpptime = [sum(cgpptimelist)],
			totalcgpptimeparallel = [sum(cgpptimeparallellist)],
			avgitertime = [mean(iptimelistlist+rmptimelist+cgpptimeparallellist)],
			minitertime = [minimum(iptimelistlist+rmptimelist+cgpptimeparallellist)],
			meditertime = [median(iptimelistlist+rmptimelist+cgpptimeparallellist)],
			maxitertime = [maximum(iptimelistlist+rmptimelist+cgpptimeparallellist)],
			avgorderarccount = [mean(orderarccountlist)]
           )

	CSV.write(filename, df) #append=true)

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function savefullsolution_online(fullxsolutionfilename, fullysolutionfilename, fullzsolutionfilename, currtime, x, y, z, orderArcSet)

	xlist, zlist = [], []
	for i in orders, a in setdiff(orderArcSet[i], dummyarc)
		push!(xlist, (i,a))
	end
	for d in drivers, a in homeArcSet[d]
		push!(zlist, (d,a))
	end

	#Save x solution
	df_x = DataFrame(varname = ["x" for item in xlist], 
				   currtime = [currtime for item in xlist],
				   i = [item[1] for item in xlist],
				   a = [item[2] for item in xlist],
				   arcstartloc = [nodesLookup[arcLookup[item[2]][1]][1] for item in xlist],
				   arcendloc = [nodesLookup[arcLookup[item[2]][2]][1] for item in xlist],
				   arcstarttime = [nodesLookup[arcLookup[item[2]][1]][2] + currtime for item in xlist],
				   arcendtime =  [nodesLookup[arcLookup[item[2]][2]][2] + currtime for item in xlist],
				   value = [getvalue(x[item]) for item in xlist]
					)

	CSV.write(string(fullxsolutionfilename, currtime, ".csv"), df_x)

	#Save y solution
	df_y = DataFrame(varname = ["y" for item in A_hasdriver], 
				   currtime = [currtime for item in A_hasdriver],
				   a = [item for item in A_hasdriver],
				   arcstartloc = [nodesLookup[arcLookup[item][1]][1] for item in A_hasdriver],
				   arcendloc = [nodesLookup[arcLookup[item][2]][1] for item in A_hasdriver],
				   arcstarttime = [nodesLookup[arcLookup[item][1]][2] + currtime for item in A_hasdriver],
				   arcendtime =  [nodesLookup[arcLookup[item][2]][2] + currtime for item in A_hasdriver],
				   value = [getvalue.(y[item]) for item in A_hasdriver]
					)

	CSV.write(string(fullysolutionfilename, currtime, ".csv"), df_y)

	#Save z solution
	df_z = DataFrame(varname = ["z" for item in zlist], 
				   currtime = [currtime for item in zlist],
				   d = [item[1] for item in zlist],
				   a = [item[2] for item in zlist],
				   arcstartloc = [nodesLookup[arcLookup[item[2]][1]][1] for item in zlist],
				   arcendloc = [nodesLookup[arcLookup[item[2]][2]][1] for item in zlist],
				   arcstarttime = [nodesLookup[arcLookup[item[2]][1]][2] + currtime for item in zlist],
				   arcendtime =  [nodesLookup[arcLookup[item[2]][2]][2] + currtime for item in zlist],
				   value = [getvalue(z[item]) for item in zlist]
					)

	CSV.write(string(fullzsolutionfilename, currtime, ".csv"), df_z)

end

#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------#

function savefullsolution_bns(fullxsolutionfilename, fullysolutionfilename, fullzsolutionfilename, currtime)

	#Save x solution
	df_x = DataFrame(varname = ["x" for item in pastordersegments], 
				   currtime = [currtime for item in pastordersegments],
				   i = [item[1] for item in pastordersegments],
				   a = ["" for item in pastordersegments],
				   arcstartloc = [item[4] for item in pastordersegments],
				   arcendloc = [item[5] for item in pastordersegments],
				   arcstarttime = [item[2] for item in pastordersegments],
				   arcendtime =  [item[3] + currtime for item in pastordersegments],
				   value = [1 for item in pastordersegments]
					)

	CSV.write(string(fullxsolutionfilename, currtime, ".csv"), df_x)

	#Save y solution
	df_y = DataFrame(varname = ["y" for item in pastemptysegments], 
				   currtime = [currtime for item in pastemptysegments],
				   a = ["" for item in pastemptysegments],
				   arcstartloc = [item[4] for item in pastemptysegments],
				   arcendloc = [item[5] for item in pastemptysegments],
				   arcstarttime = [item[2] for item in pastemptysegments],
				   arcendtime =  [item[3] for item in pastemptysegments],
				   value = [item[1] for item in pastemptysegments]
					)

	CSV.write(string(fullysolutionfilename, currtime, ".csv"), df_y)

	#Save z solution
	df_z = DataFrame(varname = ["z" for item in pastdriversegments], 
				   currtime = [currtime for item in pastdriversegments],
				   d = [item[1] for item in pastdriversegments],
				   a = ["" for item in pastdriversegments],
				   arcstartloc = [item[4] for item in pastdriversegments],
				   arcendloc = [item[5] for item in pastdriversegments],
				   arcstarttime = [item[2] for item in pastdriversegments],
				   arcendtime =  [item[3] for item in pastdriversegments],
				   value = [1 for item in pastdriversegments]
					)

	CSV.write(string(fullzsolutionfilename, currtime, ".csv"), df_z)

end

#---------------------------------------------------------------------------------------#

function writeparametersfile(filename)

	parametervalues = [ex, solutionmethod, iterationordercap, maxlocs, maxdrivers, numtrucks, weekstart, randomseedval, tstep, horizon, shiftlength, lambda, taxicostpct, roundup_flag, drivershifttstep, inprogressdummyarc_flag, excludeoutliers_flag, googlemapstraveltimes_flag, includesymmetricarcs_flag, traveltimefordelay_flag, becomesavailablehours, timedelta, onlinetimehorizon, numiterations_online, finallegdistancepenalty, finallegtimepenalty, pruneorderarcs_flag, dummyendtime, newreducedcost_flag, minarcspernode, globalrcthreshold, consolidatepitstopsindp_flag, dp_pitstopclusterradius, solvedpheuristically_flag, pathsperiter, adaptivepathsperiter_flag, pathsperiter1, pathsitercap1, pathsperiter2, pathsitercap2, pathsperiter3, printstatements, writeresultsfile, writeorderoutcomesfiles, writecorridorfile, maketimespacevizfiles, makespatialvizfiles, makeadvancedvizfiles, darkestcolor, lightestcolor, showdrivers_flag]
	stringofparams = "ex, solutionmethod, iterationordercap, maxlocs, maxdrivers, numtrucks, weekstart, randomseedval, tstep, horizon, shiftlength, lambda, taxicostpct, roundup_flag, drivershifttstep, inprogressdummyarc_flag, excludeoutliers_flag, googlemapstraveltimes_flag, includesymmetricarcs_flag, traveltimefordelay_flag, becomesavailablehours, timedelta, onlinetimehorizon, numiterations_online, finallegdistancepenalty, finallegtimepenalty, pruneorderarcs_flag, dummyendtime, newreducedcost_flag, minarcspernode, globalrcthreshold, consolidatepitstopsindp_flag, dp_pitstopclusterradius, solvedpheuristically_flag, pathsperiter, adaptivepathsperiter_flag, pathsperiter1, pathsitercap1, pathsperiter2, pathsitercap2, pathsperiter3, printstatements, writeresultsfile, writeorderoutcomesfiles, writecorridorfile, maketimespacevizfiles, makespatialvizfiles, makeadvancedvizfiles, darkestcolor, lightestcolor, showdrivers_flag"	
	parameternames = split(stringofparams, ", ")

	df = DataFrame(parametername = parameternames,
					parametervalue = parametervalues
					)

	CSV.write(filename, df)

end

#---------------------------------------------------------------------------------------#
#-------------------------------------VISUALIZATION-------------------------------------#  
#---------------------------------------------------------------------------------------#

function onlineviz(currtime, x, y, z, w, restrictedArcSet)

	wklydelta = mod(Dates.value(Dates.Hour(currentdatetime - weekstart)), 168)

	#====================================================#

	#Create new lookups relative to original time horizon
	viztimeperiods = timeperiods + wklydelta/tstep
	viznodesLookup, viznodes = Dict(), Dict()
	index = 1
	for t in 0:tstep:horizon+wklydelta, l in 1:numlocs
		viznodes[l,t] = index
		viznodesLookup[index] = (l,t)
		index += 1
	end

	#====================================================#

	#Get segments for the visualization
	currentordersegments, currentdriversegments, currentdriversegments_space, currentemptysegments, currenttaxisegments = [], [], [], [], []

	#Add order segments
	for i in orders, a in setdiff(restrictedArcSet[i], union(dummyarc, [a for a in numarcs+1:extendednumarcs]))
		if getvalue(x[i,a]) > 0.001
			#Add new current segment = (i, starttime, endtime, startloc, endloc)
			push!(currentordersegments, (i, nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ))
		end
	end

	#Add driver segments
	for d in drivers, a in homeArcSet[d]
		if getvalue(z[d,a]) > 0.001
			#Add new current segment = (d, starttime, endtime, startloc, endloc)
			push!(currentdriversegments, (d, nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ))
			if a in A_space
				push!(currentdriversegments_space, (d, nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ))
			end
		end
	end

	#Add empty segments
	for a in A_hasdriver_space
		if getvalue(y[a]) > 0.001
			#Add new current segment = (numtrucks, starttime, endtime, startloc, endloc)
			push!(currentemptysegments, (getvalue(y[a]), nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ))
		end
	end

	#Add taxi segments
	for a in A_space
		if getvalue(w[a]) > 0.001
			#Add new current segment = (numtrucks, starttime, endtime, startloc, endloc)
			push!(currenttaxisegments, (getvalue(w[a]), nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ))
		end
	end

	#====================================================#

	orderarcsegments = []

	#Add order arc segments (arcs generated through column generation)
	for i in orders, a in setdiff(restrictedArcSet[i], union(dummyarc, [a for a in numarcs+1:extendednumarcs]))
		#Add new current segment = (i, starttime, endtime, startloc, endloc)
		push!(orderarcsegments, (i, nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ) )
	end

	homearcsegments = []
	#Add driver arc segments
	for d in drivers, a in setdiff(homeArcSet[d], union(dummyarc, [a for a in numarcs+1:extendednumarcs]))
		#Add new current segment = (d, starttime, endtime, startloc, endloc)
		push!(homearcsegments, (d, nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ) )
	end

	#====================================================#

	#Visualize

	if maketimespacevizfiles == 1
		timespaceviz_online(string(vizfoldername, "/TimeSpaceNetworkMaps/", vizfilename, currtime, ".png"), showdrivers_flag, wklydelta, viznodesLookup, viznodes, viztimeperiods, currentordersegments, currentdriversegments, currentdriversegments_space, currentemptysegments, currenttaxisegments)
	end

	if makeadvancedvizfiles == 1
		for i in 1:highestorderindex
			timespaceviz_online_byorder(string(vizfoldername, "/OrderMaps/", vizfilename, "order", i, "_", currtime, ".png"), i, wklydelta, viznodesLookup, viznodes, viztimeperiods, currentordersegments, orderarcsegments)
		end

		for d in drivers
			timespaceviz_online_bydriver(string(vizfoldername, "/DriverMaps/", vizfilename, "driver", d, "_", currtime, ".png"), d, wklydelta, viznodesLookup, viznodes, viztimeperiods, currentdriversegments, homearcsegments)
		end
	end

end

#---------------------------------------------------------------------------------------#  

function onlineviz_bns(currtime)

	wklydelta = mod(Dates.value(Dates.Hour(currentdatetime - weekstart)), 168)

	#====================================================#

	#Create new lookups relative to original time horizon
	viztimeperiods = timeperiods + wklydelta/tstep
	viznodesLookup, viznodes = Dict(), Dict()
	index = 1
	for t in 0:tstep:horizon+wklydelta, l in 1:numlocs
		viznodes[l,t] = index
		viznodesLookup[index] = (l,t)
		index += 1
	end

	#====================================================#
	
	homearcsegments = []
	#Add driver arc segments
	for d in drivers, a in setdiff(homeArcSet[d], union(dummyarc, [a for a in numarcs+1:extendednumarcs]))
		#Add new current segment = (d, starttime, endtime, startloc, endloc)
		push!(homearcsegments, (d, nodesLookup[arcLookup[a][1]][2] + wklydelta, nodesLookup[arcLookup[a][2]][2] + wklydelta, nodesLookup[arcLookup[a][1]][1], nodesLookup[arcLookup[a][2]][1] ) )
	end

	#====================================================#

	#Visualize
	if maketimespacevizfiles == 1
		timespaceviz_online_heuristic(string(vizfoldername, "/TimeSpaceNetworkMaps/", vizfilename, currtime, ".png"), showdrivers_flag, wklydelta, viznodesLookup, viznodes, viztimeperiods, pastordersegments, pastdriversegments, pastdriversegments_space, pastemptysegments, pasttaxisegments)
	end

	if makeadvancedvizfiles == 1
		for i in 1:highestorderindex
			timespaceviz_online_byorder(string(vizfoldername, "/OrderMaps/", vizfilename, "order", i, "_", currtime, ".png"), i, wklydelta, viznodesLookup, viznodes, viztimeperiods, pastordersegments, [])
		end

		for d in drivers
			timespaceviz_online_bydriver(string(vizfoldername, "/DriverMaps/", vizfilename, "driver", d, "_", currtime, ".png"), d, wklydelta, viznodesLookup, viznodes, viztimeperiods, pastdriversegments, homearcsegments)
		end
	end

end
