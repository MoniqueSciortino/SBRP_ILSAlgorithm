#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <set>
#include <vector>
#include<numeric>
#include <time.h> 
#include <math.h> 
#include <string> 
#include <valarray>
#include <random>
#include <functional>
#include <cstdlib>
#include "stdafx.h"					// ALGLIB library header file	
#include <stdlib.h>
#include <stdio.h>
#include "dataanalysis.h"			// ALGLIB library header file
#include <limits>
#include <tuple>
#include <unordered_map>

using namespace std;
using namespace alglib;
int students = 0, all_stops;
vector<int> cap;
double MRT, max_walking_distance, maxno = __DBL_MAX__, alpha, beta_par, lb_correlation, sigma, mu, gam, val1, val2, val3, drive_time, drive_time2;
vector<vector<double> > drive, dist, coordinates;
vector<double> new_Route_values_temp, best_new_Route_values_temp, new_Route_values2_noinv, new_Route_values1_inv, new_Route_values2_inv;
double new_per_temp, new_mean_temp, new_per1_inv, new_mean1_inv, new_per2_noinv, new_mean2_noinv, new_per2_inv, new_mean2_inv; 
double cost_improvement, cost_improvement_temp, best_cost_improvement;
ofstream file_results;
int correlated_case;
string desired_path;

// ------------------------------------------------------------------------ STRUCTURES ----------------------------------------------------------------------------------------------

// arc: (i) starting vertex; (ii) ending vertex; (iii) boarding at starting vertex; 
struct arc
{
	int start;
	int end;
	int boarding;
};

// stop: (i) number; (ii) boarding; (iii) compulsory (T/F)
struct stop_board_compulsory_info
{
	int num;
	int boarding;
	bool compulsory;
};

// stop_walk_info: (i) num; (ii) walking_distance
struct stop_walk_info
{
	int num;
	double walking_distance;
};

// route_position_board_load: (i) route index; (ii) position in route; (iii) boarding; (iv) load
struct route_position_board_load
{
	int route_index;
	int route_position;
	int boarding;
	int load;
};

// address: (i) name; (ii) siblings
struct address_info
{
	string name;
	int siblings;
};

vector<arc> add1, remove1, add2, remove2, remove1_inv, remove2_inv, arc_add, arc_remove, route_temp;

// ------------------------------------------------------------------- ADDRESS/DRIVE/WALK DATA -------------------------------------------------------------------------------------

// Fills address vector from the .bus file
void fillData(int all_stops, int addresses, vector<address_info> &vec_addresses, vector<vector<double> > &walk, ifstream &input)
{
	string myString, line, tempString;
	int line_no = 0;

	// Coordinates
	while (line_no < (all_stops + 1) && getline(input, line)) {
		stringstream ss(line);
		getline(ss, tempString, ',');
		getline(ss, myString, ',');
		coordinates[line_no][0] = stod(myString);
		getline(ss, myString, ',');
		coordinates[line_no][1] = stod(myString);
		line_no++;
	}

	// Addresses
	line_no = 0; 
	while (line_no < addresses && getline(input, line))
	{
		stringstream ss(line);
		getline(ss, tempString, ',');
		getline(ss, tempString, ',');
		getline(ss, tempString, ',');
		getline(ss, myString, ',');
		vec_addresses[line_no].siblings = stoi(myString);
		getline(ss, myString, ',');
		vec_addresses[line_no].name = myString;
		line_no++;
	}

	// Driving Times (s) & Distances (km)
	int stop1, stop2;
	line_no = 0;
	while (line_no < (all_stops + 1) * (all_stops + 1) && getline(input, line))
	{
		stringstream ss(line);
		getline(ss, tempString, ',');
		getline(ss, myString, ',');
		stop1 = stoi(myString);
		getline(ss, myString, ',');
		stop2 = stoi(myString);
		getline(ss, myString, ',');
		dist[stop1][stop2] = stod(myString);
		getline(ss, myString, ',');
		drive[stop1][stop2] = stod(myString);
		line_no++;
	}

	// Walking Distances (km)
	int address_no, stop_no;
	while (getline(input, line))
	{
		stringstream ss(line);
		getline(ss, tempString, ',');
		getline(ss, myString, ',');
		address_no = stoi(myString);
		getline(ss, myString, ',');
		stop_no = stoi(myString) - 1;
		getline(ss, myString, ',');
		walk[address_no][stop_no] = stod(myString);
	}
}

// --------------------------------------------------------------------- RANDOM FOREST  ---------------------------------------------------------------------------------------------

real_1d_array arr = "[0,0,0,0,0,0,0,0,0,0]";
decisionforest model;		 
mt19937 corrgenerator(100);		   								
unordered_map<long long int,double> correlations;               
unordered_map<long long int,double>::const_iterator finder;
double min_distance, corr1, corr2, corr;
long long int arc_index;

set<int> pick(int N, int k) {
	uniform_int_distribution<int> distribution(1, N);
    set<int> elems;
    while (elems.size() < k) elems.insert(distribution(corrgenerator));
    return elems;
}

void randomForest() {
	vector<string> locations = {"Mgarr","Mellieha","Porthcawl","Qrendi","Suffolk","Senglea","Victoria","Pembroke","Canberra","Handaq"};
	vector<int> stops = {60,86,153,158,174,186,316,322,331,393};	// including school
	double test_frac = 0.2;
    int no_locations = locations.size(), lines_to_read = 100000, test_lines = test_frac*lines_to_read;  	
    
    real_2d_array train_data, test_data;   
    ae_int_t test_size = lines_to_read*no_locations*test_frac;
    ae_int_t train_size = lines_to_read*no_locations*(1-test_frac);
    train_data.setlength(train_size,11);
    test_data.setlength(test_size,11);	

	string myString, line;
    int test_counter = 0, train_counter = 0, line_no;   
    set<int> rand_set;
        
    // reading data on common arcs
    clock_t rf_start = clock();
    for (int j = 0; j < no_locations; j++) 
    {   
        line_no = 1;
        
        desired_path = locations[j] + "CommonArcs500000.txt";			
        ifstream inFile(desired_path);

        // identifying line numbers to feature in test set for locations[j]
        rand_set = pick(lines_to_read,test_lines);
        vector<int> rand_vect(rand_set.begin(), rand_set.end());

        while (line_no <= lines_to_read)
        {
            getline(inFile,line);
            stringstream ss(line);

            if (binary_search(rand_vect.begin(), rand_vect.end(), line_no)) 
            {
                for (int i = 0; i < 14; i++) {
                    getline(ss,myString,',');
                    if (i != 8 and i != 9 and i != 10)  {
                        if (i < 8)     test_data(test_counter,i) = stod(myString);
                        else           test_data(test_counter,i-3) = stod(myString);
                    }
                }
                test_counter++;
            }
            else 
            {
                for (int i = 0; i < 14; i++) {
                    getline(ss,myString,',');
                    if (i != 8 and i != 9 and i != 10)  {
                        if (i < 8)     train_data(train_counter,i) = stod(myString);
                        else           train_data(train_counter,i-3) = stod(myString);
                    }
                }
                train_counter++;
            }

            line_no++;
        }

        cout << "Data read for " << locations[j] << " - total lines = " << line_no-1 << endl;
    }
    clock_t rf_finish = clock();
    cout << "Time Taken to Load Data = " << (double(rf_finish - rf_start)/CLOCKS_PER_SEC) << "s" << endl;

    double fraction = 0.1;			
    ae_int_t mtry = 6;				
    ae_int_t ntrees = 100;
       
    rf_start = clock();
    decisionforestbuilder builder;
    ae_int_t nvars = 10;
    ae_int_t nclasses = 1;
    ae_int_t npoints = train_size;

    dfbuildercreate(builder);
    dfbuildersetdataset(builder, train_data, npoints, nvars, nclasses);

    // Extremely randomized tree (ExtraTree)
    dfbuildersetrdfsplitstrength(builder,0);        // use random splits 
    dfbuildersetrndvars(builder,mtry);              // use sqrt(nvars) randomly chosen variables

    // set seed 
    dfbuildersetseed(builder, 1);

    // train random forest using (fraction*100)% of sample (decrease to reduce overfitting)
    dfbuildersetsubsampleratio(builder, fraction);

    // train random forest with ntrees trees (recommended: 50-500 trees)
    dfreport rep;
    dfbuilderbuildrandomforest(builder, ntrees, model, rep);

    file_results << "-------------------------------------------------------------------------------------------------------";
    file_results << endl << "FRACTION, MTRY = " << fraction << "," << mtry;
    file_results << endl << "RMSE ON TRAINING SET: " << double(rep.rmserror) << endl;  
    file_results << "OUT-OF-BAG ESTIMATE OF RMSE: " << double(rep.oobrmserror) << endl; 
    file_results << "RMSE ON TEST SET: " << dfrmserror(model,test_data,test_size) << endl << endl; 
    file_results << "AVG ERROR ON TRAINING SET: " << double(rep.avgerror) << endl;  
    file_results << "OUT-OF-BAG ESTIMATE OF AVG ERROR: " << double(rep.oobavgerror) << endl;
    file_results << "AVG ERROR ON TEST SET: " << dfavgerror(model,test_data,test_size) << endl << endl; 

    file_results << "AVG RELATIVE ERROR ON TRAINING SET: " << double(rep.avgrelerror) << endl; 
    file_results << "OUT-OF-BAG ESTIMATE OF AVG RELATIVE ERROR: " << double(rep.oobavgrelerror) << endl;  
    file_results << "AVG RELATIVE ERROR ON TEST SET: " << dfavgrelerror(model,test_data,test_size) << endl << endl; 

    file_results << "VARIABLE IMPORTANCE: ";
	integer_1d_array top_vars = rep.topvars;
    for (int i = 0; i < nvars; i++) {
        if (top_vars[i] == 0)      file_results << "Arc1Lat1 ";
        else if (top_vars[i] == 1) file_results << "Arc1Long1 ";
        else if (top_vars[i] == 2) file_results << "Arc1Lat2 ";
        else if (top_vars[i] == 3) file_results << "Arc1Long2 ";
        else if (top_vars[i] == 4) file_results << "Arc2Lat1 ";
        else if (top_vars[i] == 5) file_results << "Arc2Long1 ";
        else if (top_vars[i] == 6) file_results << "Arc2Lat2 ";
        else if (top_vars[i] == 7) file_results << "Arc2Long2 ";
        else if (top_vars[i] == 8) file_results << "Arc1Distance ";
        else file_results << "Arc2Distance ";
    }
    file_results << endl << endl;
                    
    rf_finish = clock();
    file_results << "Time Taken to Fit Random Forest = " << (double(rf_finish - rf_start)/CLOCKS_PER_SEC) << "s" << endl;
    file_results << "-------------------------------------------------------------------------------------------------------";
}

bool comp(double no1, double no2){
    return (no1 < no2);    
}

int bigger(int endpt1, int endpt2) {
	if (endpt1 > endpt2) 	return 1;
	else 					return 0;
}

double correlationCalc(int arc1_start, int arc1_end, int arc2_start, int arc2_end) {
	if (correlated_case == 0)	return 0;
	else {
		if (arc1_start == arc2_end) {
			arc_index = pow(all_stops+1-2,2)*(arc2_start-1) + 
			(all_stops+1-2)*(arc2_end-1-bigger(arc2_end,arc2_start)) +
			(arc1_end-bigger(arc1_end,arc2_start)-bigger(arc1_end,arc2_end)) + 1;
			if (arc_index % 100 == 0)  		return max(-0.7,lb_correlation);
			else if (arc_index % 2 == 0)	return 0.7;
			else							return 0; 
		}
		else if (arc2_start == arc1_end) {
			arc_index = pow(all_stops+1-2,2)*(arc1_start-1) + 
			(all_stops+1-2)*(arc1_end-1-bigger(arc1_end,arc1_start)) +
			(arc2_end-bigger(arc2_end,arc1_start)-bigger(arc2_end,arc1_end)) + 1;
			if (arc_index % 100 == 0)  		return max(-0.7,lb_correlation);
			else if (arc_index % 2 == 0)	return 0.7;
			else							return 0; 
		}
		else {
			if (arc2_start < arc1_start) {
				arc_index = pow(all_stops+1-2,2)*(all_stops+1-3)*(arc2_start-1) + 
				(all_stops+1-2)*(all_stops+1-3)*(arc1_start-1-bigger(arc1_start,arc2_start)) +
				(all_stops+1-3)*(arc2_end-bigger(arc2_end,arc2_start)-bigger(arc2_end,arc1_start)) +
				(arc1_end-bigger(arc1_end,arc2_start)-bigger(arc1_end,arc1_start)-bigger(arc1_end,arc2_end)) + 1;
			}
			else {
				arc_index = pow(all_stops+1-2,2)*(all_stops+1-3)*(arc1_start-1) + 
				(all_stops+1-2)*(all_stops+1-3)*(arc2_start-1-bigger(arc2_start,arc1_start)) +
				(all_stops+1-3)*(arc1_end-bigger(arc1_end,arc1_start)-bigger(arc1_end,arc2_start)) +
				(arc2_end-bigger(arc2_end,arc1_start)-bigger(arc2_end,arc2_start)-bigger(arc2_end,arc1_end)) + 1;
			}
			min_distance = min({dist[arc1_start][arc2_start],dist[arc1_start][arc2_end],
			dist[arc1_end][arc2_start],dist[arc1_end][arc2_end],dist[arc2_start][arc1_start],
			dist[arc2_start][arc1_end],dist[arc2_end][arc1_start],dist[arc2_end][arc1_end]}, comp);
			if (min_distance <= 0.5) {
				if (arc_index % 50 == 0)  		return max(-0.5,lb_correlation);
				else if (arc_index % 2 == 0)	return 0.5;
				else							return 0; 
			}
			else if (min_distance <= 1) {
				if (arc_index % 50 == 0)  		return max(-0.3,lb_correlation);
				else if (arc_index % 2 == 0)	return 0.3;
				else							return 0; 
			}	
			else {	
				finder = correlations.find(arc_index);
				if (finder != correlations.end())	return finder-> second;
				else {
					arr[0] = coordinates[arc1_start][0];
					arr[1] = coordinates[arc1_start][1];
					arr[2] = coordinates[arc1_end][0];
					arr[3] = coordinates[arc1_end][1];
					arr[4] = coordinates[arc2_start][0];
					arr[5] = coordinates[arc2_start][1];
					arr[6] = coordinates[arc2_end][0];
					arr[7] = coordinates[arc2_end][1];
					arr[8] = dist[arc1_start][arc1_end];
					arr[9] = dist[arc2_start][arc2_end];
					corr1 = dfprocess0(model, arr)/(sqrt(arr[8]*arr[9]));
					arr[0] = coordinates[arc2_start][0];
					arr[1] = coordinates[arc2_start][1];
					arr[2] = coordinates[arc2_end][0];
					arr[3] = coordinates[arc2_end][1];
					arr[4] = coordinates[arc1_start][0];
					arr[5] = coordinates[arc1_start][1];
					arr[6] = coordinates[arc1_end][0];
					arr[7] = coordinates[arc1_end][1];
					arr[8] = dist[arc2_start][arc2_end];
					arr[9] = dist[arc1_start][arc1_end];	
					corr2 = dfprocess0(model, arr)/(sqrt(arr[8]*arr[9]));
					if (arc_index % 50 == 0)	corr = max(-(corr1+corr2)/2,lb_correlation); 
					else						corr = (corr1+corr2)/2;
					correlations.insert({arc_index,corr});			
					return corr;
				}
			}
		}
	}
}

// --------------------------------------------------------------------- ROUTES FUNCTIONS--------------------------------------------------------------------------------------------

// Returns index of arc in "route" that starts with "stop" (if found) or -1 otherwise
int findStopInRoute(vector<arc> route, int stop)
{
	for (int i = 0; i < route.size(); i++)
	{
		if (route[i].start == stop)   	return i;
	}
	return -1;
}

// Returns position of stop in vec (if found) or -1 otherwise
int findStopInVec(vector<stop_board_compulsory_info> vec, int stop)
{
	for (int i = 0; i < vec.size(); i++)
		if (vec[i].num == stop)		return i;
	return -1;
}

// Returns the load of route
int routeLoad(vector<stop_board_compulsory_info> route)
{
	int load = 0;
	for (int z = 1; z < route.size() - 1; z++)	load += route[z].boarding;
	return load;
}

// Calculates the total travelling time, constant D_r and total stopping time of each route r in Routes
void getRouteValues(vector<vector<stop_board_compulsory_info> > Routes, vector<vector<double> > &Routes_values)	
{
	Routes_values.clear();
	Routes_values.resize(Routes.size(),vector<double>(3));
	for (int j = 0; j < Routes.size(); j++)
	{
		Routes_values[j][0] = 0;
		Routes_values[j][1] = 0;
		Routes_values[j][2] = 0;
		for (int k = 1; k < Routes[j].size() - 1; k++)
		{
			Routes_values[j][2] += 15 + 5 * Routes[j][k].boarding;
			drive_time = drive[Routes[j][k].num][Routes[j][k + 1].num];
			if (drive_time > 0) {
				Routes_values[j][0] += drive_time;
				Routes_values[j][1] += pow(drive_time,2);
				for (int l = k + 1; l < Routes[j].size() - 1; l++) {
					drive_time2 = drive[Routes[j][l].num][Routes[j][l + 1].num];
					if (drive_time2 > 0)
						Routes_values[j][1] += 2*correlationCalc(Routes[j][k].num,Routes[j][k + 1].num,
						Routes[j][l].num,Routes[j][l + 1].num)*drive_time*drive_time2;
				}
			}
		}
	}
}

// Calculates the 95th percentiles and means of all routes
void allRoutes95Percentiles_Means(vector<vector<double> > Routes_values, vector<double> &Routes_percentiles, vector<double> &Routes_means) 
{
	Routes_percentiles.clear();
	Routes_percentiles.resize(Routes_values.size());
	Routes_means.clear();
	Routes_means.resize(Routes_values.size());
	for (int j = 0; j < Routes_values.size(); j++)
	{
		sigma = sqrt(log((pow(beta_par,2)*Routes_values[j][1]) / (pow(alpha*Routes_values[j][0],2)) + 1));
		mu = log(alpha * Routes_values[j][0]) - 0.5*pow(sigma,2);
		gam = (1 - alpha) * Routes_values[j][0];
		Routes_percentiles[j] = Routes_values[j][2] + (gam + exp(mu + 1.64485363*sigma)); 
		Routes_means[j] = Routes_values[j][2] + (gam + exp(mu + 0.5*pow(sigma,2)));  
	} 
}

// Displays each route's stops, number of students boarding at each stop, total distance, deterministic journey time and 95th percentile
void displayRoutes(vector<vector<stop_board_compulsory_info> > Routes, vector<vector<double> > &Routes_values, vector<double> Routes_percentiles, vector<double> Routes_means)
{
	double sum_distance; 
	int load;
	for (int j = 0; j < Routes.size(); j++)	
	{
		sum_distance = 0;
		file_results << endl << "Route " << j + 1 << " stops: ";
		for (int l = 0; l < Routes[j].size(); l++)	file_results << Routes[j][l].num << " ";
		for (int k = 1; k < Routes[j].size() - 1; k++)	sum_distance += dist[Routes[j][k].num][Routes[j][k + 1].num];
		load = routeLoad(Routes[j]);
		file_results << "with DIST,TT,ST,MJT,95P = " << sum_distance << "," << Routes_values[j][0] << "," << Routes_values[j][2] << "," << Routes_means[j] << "," << Routes_percentiles[j] << endl;
		file_results << "Route " << j + 1 << " board: ";
		for (int l = 0; l < Routes[j].size(); l++)	file_results << Routes[j][l].boarding << " ";
		file_results << "with load " << load;
	}
}

// ---------------------------------------------------------------- STOP/ADDRESS MATRICES -------------------------------------------------------------------------------------------

// Used to order the stops within max_walking_distance of an address in non-decreasing order of walking distance - Returns True if a.walking_distance < b.walking_distance
bool compareWalkingDistance(const stop_walk_info a, const stop_walk_info b)
{
	return a.walking_distance < b.walking_distance;
}

// Returns True if row1.size() > row2.size()
bool compareRowSize(const vector<int> row1, const vector<int> row2)
{
	return row1.size() > row2.size();
}

// Fills the available_stops matrix whose rows give the available (sorted) stops in subset_stops for each address in subset_addresses
void identifyAvailableStops(vector<vector<double> > walk, vector<int> subset_addresses, vector<stop_board_compulsory_info> subset_stops, vector<vector<stop_walk_info> > &available_stops)
{
	available_stops.clear();
	available_stops.resize(subset_addresses.size());
	for (int s = 0; s < subset_addresses.size(); s++)
	{
		for (int i = 0; i < subset_stops.size(); i++)
		{
			if (walk[subset_addresses[s] - 1][subset_stops[i].num - 1] <= max_walking_distance)
			{
				stop_walk_info stop = {subset_stops[i].num, walk[subset_addresses[s] - 1][subset_stops[i].num - 1]};
				available_stops[s].push_back(stop);
			}
		}
		sort(available_stops[s].begin(), available_stops[s].end(), compareWalkingDistance);
	}
}

// Fills the available_addresses matrix whose rows give the available addresses in subset_addresses for each stop in subset_stops
void identifyAvailableAddresses(vector<vector<double> > walk, vector<stop_board_compulsory_info> subset_stops, vector<int> subset_addresses, vector<vector<int> > &available_addresses)
{
	available_addresses.clear();
	available_addresses.resize(subset_stops.size());
	for (int i = 0; i < subset_stops.size(); i++)
	{
		available_addresses[i].push_back(subset_stops[i].num); // the first element of each row is the stop number
		for (int s = 0; s < subset_addresses.size(); s++)
			if (walk[subset_addresses[s] - 1][subset_stops[i].num - 1] <= max_walking_distance)    available_addresses[i].push_back(subset_addresses[s]);
	}
	sort(available_addresses.begin(), available_addresses.end(), compareRowSize);
}

// ------------------------------------------------------------------- BUS STOP SELECTION -------------------------------------------------------------------------------------------

// Used to order the subset of used stops in non-decreasing order of the stop number - returns True if a.num < b.num
bool orderStops(const stop_board_compulsory_info a, const stop_board_compulsory_info b)
{
	return a.num < b.num;
}

// Return the vector of all stops without those in vec1 and vec2
vector<stop_board_compulsory_info> allStopsWithoutTwoVectorsOfStops(int all_stops, vector<stop_board_compulsory_info> vec1, vector<stop_board_compulsory_info> vec2)
{
	vector<stop_board_compulsory_info> remaining_stops;
	for (int i = 1; i <= all_stops; i++)
	{
		if (findStopInVec(vec1, i) == -1 && findStopInVec(vec2, i) == -1)
		{
			stop_board_compulsory_info stop = {i, 0, false};
			remaining_stops.push_back(stop);
		}
	}
	return remaining_stops;
}

// Covering all addresses in subset_addresses at least once by adding stops in subset_stops to used_stops
void coverAddresses(vector<int> &subset_addresses, vector<stop_board_compulsory_info> &used_stops, vector<vector<double> > walk, vector<stop_board_compulsory_info> &subset_stops, vector<vector<int> > &available_addresses, mt19937 &generator)
{
	int no_max_rows;  // number of rows in available_addresses having maximum size
	int random_index; // index of random stop that will be added in each iteration

	while (subset_addresses.size() > 0)
	{
		no_max_rows = 1;
		identifyAvailableAddresses(walk, subset_stops, subset_addresses, available_addresses);
		for (int k = 1; k < available_addresses.size(); k++)
		{
			if (available_addresses[k].size() == available_addresses[0].size())		no_max_rows++;
			else	break;
		}

		// Generating a uniformly distributed random number between 0 and (no_max_rows-1)
		uniform_int_distribution<int> distribution(0, no_max_rows - 1);
		random_index = distribution(generator);

		// Adding the randomly chosen stop to used_stops
		stop_board_compulsory_info stop = {available_addresses[random_index][0], 0, false};
		used_stops.push_back(stop);

		// Removing addresses from subset_addresses covered by the chosen stop
		for (int j = 1; j < available_addresses[random_index].size(); j++)
			subset_addresses.erase(remove(subset_addresses.begin(), subset_addresses.end(), available_addresses[random_index][j]), subset_addresses.end());
		for (int i = 0; i < subset_stops.size(); i++)
		{
			if (subset_stops[i].num == available_addresses[random_index][0])
			{
				subset_stops.erase(subset_stops.begin() + i);
				break;
			}
		}
	}
}

// Update boarding numbers of each stop in used_stops by choosing the closest stop to each address
void updateDemands(int addresses, vector<vector<double> > walk, vector<vector<stop_walk_info> > &available_stops, vector<stop_board_compulsory_info> &used_stops, vector<int> &w, vector<address_info> vec_addresses)
{
	int pos, j = 0;
	vector<int> all_addresses;
	for (int s = 1; s <= addresses; s++)
		all_addresses.push_back(s);
	for (int i = 0; i < used_stops.size(); i++)
		used_stops[i].boarding = 0;

	identifyAvailableStops(walk, all_addresses, used_stops, available_stops);
	for (int s = 0; s < addresses; s++)
	{
		w[s] = available_stops[s][0].num;
		pos = findStopInVec(used_stops, available_stops[s][0].num);
		used_stops[pos].boarding += vec_addresses[s].siblings;
	}

	// If any stop in used_stops has demand 0, then it should be removed
	while (j < used_stops.size())
	{
		if (used_stops[j].boarding == 0)	used_stops.erase(used_stops.begin() + j);
		else	j++;
	}
}

// Chooses a random subset of stops that covers each address at least once
void subsetStopsSelection(int addresses, int all_stops, vector<vector<double> > walk, vector<vector<stop_walk_info> > &available_stops, vector<vector<int> > &available_addresses, vector<stop_board_compulsory_info> &used_stops, mt19937 &generator, vector<int> &w, vector<address_info> vec_addresses)
{
	used_stops.clear();

	// Initially setting subset_stops as the set of all stops and subset_addresses as the set of all addresses
	vector<stop_board_compulsory_info> subset_stops, extra_vector;
	for (int i = 1; i <= all_stops; i++)
	{
		stop_board_compulsory_info stop = {i, 0, false};
		subset_stops.push_back(stop);
	}
	vector<int> subset_addresses;
	for (int s = 1; s <= addresses; s++)	subset_addresses.push_back(s);

	// Adding compulsory stops to used_stops
	identifyAvailableStops(walk, subset_addresses, subset_stops, available_stops);
	for (int s = 0; s < addresses; s++)
	{
		if (available_stops[s].size() == 1)
		{
			stop_board_compulsory_info stop = {available_stops[s][0].num, 0, true};
			used_stops.push_back(stop);
		}
	}

	// Removing addresses who are covered by compulsory stops from subset_addresses
	identifyAvailableAddresses(walk, used_stops, subset_addresses, available_addresses);
	for (int k = 0; k < available_addresses.size(); k++)
	{
		for (int j = 1; j < available_addresses[k].size(); j++)
			subset_addresses.erase(remove(subset_addresses.begin(), subset_addresses.end(), available_addresses[k][j]), subset_addresses.end());
	}
	subset_stops = allStopsWithoutTwoVectorsOfStops(all_stops, used_stops, extra_vector);

	// Looping until all addresses are covered at least once
	coverAddresses(subset_addresses, used_stops, walk, subset_stops, available_addresses, generator);
	sort(used_stops.begin(), used_stops.end(), orderStops);
	updateDemands(addresses, walk, available_stops, used_stops, w, vec_addresses);
}

// ------------------------------------------------------------------ NEAREAST NEIGHBOUR --------------------------------------------------------------------------------------------

// Parallel Nearest Neighbour constructive heuristic (with "no_vehicles" routes)
void initialRoutesNN(vector<stop_board_compulsory_info> used_stops, vector<vector<stop_board_compulsory_info> > &Routes, int no_vehicles)
{
	Routes.clear();
	Routes.resize(no_vehicles);
	stop_board_compulsory_info zero_stop = {0, 0};
	vector<int> index_closest_stop(no_vehicles);
	vector<int> remaining_cap = cap;
	vector<stop_board_compulsory_info> last_stop(no_vehicles, zero_stop);
	for (int i = 0; i < Routes.size(); i++)
		Routes[i].push_back(zero_stop);		// initially all routes have just 0

	while (!used_stops.empty())
	{
		for (int j = 0; j < Routes.size(); j++)
		{
			index_closest_stop[j] = 0;
			if (remaining_cap[j] > 0 && !used_stops.empty())
			{
				// Nearest neighbour is the one with shortest time away including loading time
				// We keep on loading students until bus is full
				for (int i = 0; i < used_stops.size(); i++)
				{
					if (15 + 5 * min(remaining_cap[j], used_stops[i].boarding) + drive[used_stops[i].num][last_stop[j].num] <
						15 + 5 * min(remaining_cap[j], used_stops[index_closest_stop[j]].boarding) + drive[used_stops[index_closest_stop[j]].num][last_stop[j].num])
						index_closest_stop[j] = i;
				}
				last_stop[j] = (stop_board_compulsory_info){used_stops[index_closest_stop[j]].num, min(remaining_cap[j], used_stops[index_closest_stop[j]].boarding)};
				Routes[j].push_back(last_stop[j]);
				remaining_cap[j] -= last_stop[j].boarding;
				used_stops[index_closest_stop[j]].boarding -= last_stop[j].boarding;
				if (used_stops[index_closest_stop[j]].boarding == 0)
					used_stops.erase(used_stops.begin() + index_closest_stop[j]);
			}				
		}
	}

	for (int i = 0; i < Routes.size(); i++)
	{
		Routes[i].push_back(zero_stop);
		reverse(Routes[i].begin(), Routes[i].end()); // the routes are constructed backwards so we reverse them
	}
}

// ------------------------------------------------------------ LOCAL SEARCH MOVE EVALUATIONS ---------------------------------------------------------------------------------------

double routeBalancing(vector<double> Routes_percentiles, int m, double newPer1, int n, double newPer2)
{
	Routes_percentiles[m] = newPer1;
	if (n != -1) 	Routes_percentiles[n] =  newPer2;
	return (*max_element(Routes_percentiles.begin(), Routes_percentiles.end()) - *min_element(Routes_percentiles.begin(), Routes_percentiles.end()));
}

// Returns the change in total cost when percentile old_p1 changes to new_p1, mean old_m1 changes to new_m1
// same for those of route 2
double costImprovement(double old_p1, double new_p1, double old_m1, double new_m1, double old_p2, double new_p2, double old_m2, double new_m2)
{
	double cost_difference = 0.0;

	if (old_p1 <= MRT)		cost_difference += old_p1;
	else 					cost_difference += MRT * (2 + old_p1 - MRT);
	if (new_p1 <= MRT)		cost_difference -= new_p1;
	else 					cost_difference -= MRT * (2 + new_p1 - MRT);

	if (old_p2 != 0) {
		if (old_p2 <= MRT)	cost_difference += old_p2;
		else 				cost_difference += MRT * (2 + old_p2 - MRT);
		if (new_p2 <= MRT)	cost_difference -= new_p2;
		else 				cost_difference -= MRT * (2 + new_p2 - MRT);
	}

	return cost_difference;
}

// Updates route parameters and percentile/mean when arcs in "remove" are removed and arcs in "add" are added to "route"
void updateRouteParameters(vector<arc> route, vector<arc> remove, vector<arc> add, vector<double> &new_Route_values, double &new_per, double &new_mean) {
	int pos = -1;
	for (int i = 0; i < remove.size(); i++) {
		if (remove[i].start != 0) 
		{
			for (int j = 1; j < route.size(); j++) {
				if (route[j].start == remove[i].start && route[j].end == remove[i].end) 	
				{
					pos = j;
					break;
				}
			}
			drive_time = drive[remove[i].start][remove[i].end];
			if (drive_time > 0) {
				new_Route_values[0] -= drive_time;
				new_Route_values[1] -= pow(drive_time,2);
				for (int j = 1; j < route.size(); j++) {		
					if (j != pos) {
						drive_time2 = drive[route[j].start][route[j].end];
						if (drive_time2 > 0) 
							new_Route_values[1] -= 2*correlationCalc(remove[i].start,remove[i].end,route[j].start,route[j].end)*
												drive_time*drive_time2;
					}
				}
			}
			route.erase(route.begin() + pos);
		}
	}

	for (int i = 0; i < add.size(); i++) {
		if (add[i].start != 0) 
		{
			drive_time = drive[add[i].start][add[i].end];
			if (drive_time > 0) {
				new_Route_values[0] += drive_time;
				new_Route_values[1] += pow(drive_time,2);

				for (int j = i + 1; j < add.size(); j++) {
					if (add[j].start != 0) {
						drive_time2 = drive[add[j].start][add[j].end];
						if (drive_time2 > 0) 
							new_Route_values[1] += 2*correlationCalc(add[i].start,add[i].end,add[j].start,add[j].end)*
												drive_time*drive_time2;
					}
				}

				for (int j = 1; j < route.size(); j++) {
					drive_time2 = drive[route[j].start][route[j].end];
					if (drive_time2 > 0) 
						new_Route_values[1] += 2*correlationCalc(add[i].start,add[i].end,route[j].start,route[j].end)*
											drive_time*drive_time2;
				}
			}
		}
	}

	sigma = sqrt(log((pow(beta_par,2)*new_Route_values[1]) / (pow(alpha*new_Route_values[0],2)) + 1));
	mu = log(alpha * new_Route_values[0]) - 0.5*pow(sigma,2);
	gam = (1 - alpha) * new_Route_values[0];
	new_per = new_Route_values[2] + (gam + exp(mu + 1.64485363*sigma));
	new_mean = new_Route_values[2] + (gam + exp(mu + 0.5*pow(sigma,2)));    
}

// Returns cost improvement when swapping stops in positions l and k in route
double checkSwapIntraRoute(vector<arc> route, int l, int k, double old_per, double &new_per, double old_mean, double &new_mean, vector<double> &new_Route_values)
{
	add1.clear();
	remove1.clear();
	if (k > l + 1) {
		remove1.push_back(route[l - 1]);
		remove1.push_back(route[l]);
		remove1.push_back(route[k - 1]);
		remove1.push_back(route[k]);
		add1.push_back({route[l - 1].start,route[k].start,0});
		add1.push_back({route[k].start,route[l].end,0});
		add1.push_back({route[k - 1].start,route[l].start,0});
		add1.push_back({route[l].start,route[k].end,0});
	}
	else {
		remove1.push_back(route[l - 1]);
		remove1.push_back(route[l]);
		remove1.push_back(route[k]);
		add1.push_back({route[l - 1].start,route[k].start,0});
		add1.push_back({route[k].start,route[l].start,0});
		add1.push_back({route[l].start,route[k].end,0});	    
	}
	updateRouteParameters(route, remove1, add1, new_Route_values, new_per, new_mean);	
	return costImprovement(old_per, new_per, old_mean, new_mean, 0, 0, 0, 0); 
}

// Returns cost improvement when inverting subroute (2-opt) in positions l ... k in route
double check2OptIntraRoute(vector<arc> route, int l, int k, double old_per, double &new_per, double old_mean, double &new_mean, vector<double> &new_Route_values)
{
	add1.clear();
	remove1.clear();
	remove1.push_back(route[l - 1]);
	remove1.push_back(route[k]);
	add1.push_back({route[l - 1].start,route[k].start,0});
	add1.push_back({route[l].start,route[k].end,0});
	for (int count = 0; count < (k - l); count++) {
		remove1.push_back(route[l + count]);
		add1.push_back({route[k - 1 - count].end,route[k - 1 - count].start,0});
	}
	updateRouteParameters(route, remove1, add1, new_Route_values, new_per, new_mean);		
	return costImprovement(old_per, new_per, old_mean, new_mean, 0, 0, 0, 0); 
}

// Returns cost improvement when moving, and possibly inverting, subroute (extended-or-opt) in positions l ... k in route before position j 
double checkExtendedOrOptIntraRoute(vector<arc> route, int j, int l, int k, bool &invert, double old_per, double &new_per, double old_mean, double &new_mean, vector<double> &new_Route_values)
{
	new_Route_values1_inv = new_Route_values;

	// no inversion
	add1.clear();
	remove1.clear();
	remove1.push_back(route[j - 1]);
	remove1.push_back(route[l - 1]);
	remove1.push_back(route[k]);
	add1.push_back({route[j - 1].start,route[l].start,0});
	add1.push_back({route[k].start,route[j - 1].end,0});
	add1.push_back({route[l - 1].start,route[k].end,0});
	updateRouteParameters(route, remove1, add1, new_Route_values, new_per, new_mean);	
	cost_improvement = costImprovement(old_per, new_per, old_mean, new_mean, 0, 0, 0, 0); 

	// with inversion
	add1.clear();
	add1.push_back({route[j - 1].start,route[k].start,0});
	add1.push_back({route[l].start,route[j - 1].end,0});
	add1.push_back({route[l - 1].start,route[k].end,0});
	for (int count = 0; count < (k - l); count++) {
		remove1.push_back(route[l + count]);
		add1.push_back({route[k - 1 - count].end,route[k - 1 - count].start,0});
	}
	updateRouteParameters(route, remove1, add1, new_Route_values1_inv, new_per1_inv, new_mean1_inv);	
	cost_improvement_temp = costImprovement(old_per, new_per1_inv, old_mean, new_mean1_inv, 0, 0, 0, 0);

	if (cost_improvement < cost_improvement_temp)
	{
		invert = true;
		new_per = new_per1_inv;
		new_mean = new_mean1_inv;
		new_Route_values = new_Route_values1_inv;
		return cost_improvement_temp;
	}
	else
	{
		invert = false;
		return cost_improvement;
	}
}

// Removes pairs of arcs which are the same in to_add and to_remove vectors
void removeCancellingArcs(vector<arc> &to_add, vector<arc> &to_remove) {
	int pos = -1, counter = 0;
	while (counter < to_add.size()) {
		pos = -1;
		for (int j = 0; j < to_remove.size(); j++) {
			if (to_remove[j].start == to_add[counter].start && to_remove[j].end == to_add[counter].end) 
			{
				pos = j; 
				break;
			}
		}
		if (pos != -1) {
			to_remove.erase(to_remove.begin() + pos);
			to_add.erase(to_add.begin() + counter);
		}
		else counter++;
	}
}

// Amends add2 to remove duplicate stops from route2 when 
// relocating subroute in positions l ... k from route1 before position j in route2 (or-exchange duplicates)
void checkRemoveDuplicatesOrExchange(vector<arc> route1, vector<arc> route2, int j, int l, int k, bool invert, vector<arc> &new_add2, vector<arc> &new_remove2, vector<double> &new_Route_values2)
{
	int last_count_not_removed = -1; 		// to keep record of which stop is next to the current one being removed
	int position;

	for (int count = 0; count <= (k - l); count++) {

		position = findStopInRoute(route2, route1[l + count].start);

		if (position == -1)		last_count_not_removed = count;
		else 
		{	// we need to remove the duplicate stop
			new_Route_values2[2] -= 15;

			if (k - l == 0) { // one element relocated (in position l) and it is already in route2
				new_add2.clear();
				new_remove2.clear();
			}

			else if (count == 0) { // first element (in position l) relocated is already in route2
				if (!invert) {
					new_remove2.push_back(route1[l]);
					new_remove2.push_back({route2[j - 1].start,route1[l].start,0});
					new_add2.push_back({route2[j - 1].start,route1[l].end,0});	
				}
				else {
					new_remove2.push_back({route1[l].end,route1[l].start,0});
					new_remove2.push_back({route1[l].start,route2[j - 1].end,0});
					new_add2.push_back({route1[l].end,route2[j - 1].end,0});	
				}
			}

			else if (count == (k - l)) {  // last element (in position k) relocated is already in route2
				if (last_count_not_removed == -1) {	// all elements up to (and including) stop at position k are duplicates
					new_add2.clear();
					new_remove2.clear();
				}
				else
				{
					if (!invert) {
						new_remove2.push_back({route1[l + last_count_not_removed].start,route1[k].start,0});
						new_remove2.push_back({route1[k].start,route2[j - 1].end,0});
						new_add2.push_back({route1[l + last_count_not_removed].start,route2[j - 1].end,0});
					}
					else {
						new_remove2.push_back({route2[j - 1].start,route1[k].start,0});
						new_remove2.push_back({route1[k].start,route1[l + last_count_not_removed].start,0});
						new_add2.push_back({route2[j - 1].start,route1[l + last_count_not_removed].start,0});
					}
				}
			}

			else {	// element in position (l+count) where 0 < count < (k-l) relocated is already in route2
				if (last_count_not_removed == -1) {  // all elements up to (and including) stop at position (l+count) are duplicates
					if (!invert) {
						new_remove2.push_back({route2[j - 1].start,route1[l + count].start,0});
						new_remove2.push_back(route1[l + count]);
						new_add2.push_back({route2[j - 1].start,route1[l + count].end,0});
					}
					else {
						new_remove2.push_back({route1[l + count].end,route1[l + count].start,0});
						new_remove2.push_back({route1[l + count].start,route2[j - 1].end,0});
						new_add2.push_back({route1[l + count].end,route2[j - 1].end,0});	
					}
				}
				else {
					if (!invert) {
						new_remove2.push_back({route1[l + last_count_not_removed].start,route1[l + count].start,0});
						new_remove2.push_back(route1[l + count]);
						new_add2.push_back({route1[l + last_count_not_removed].start,route1[l + count].end,0});
					}
					else {
						new_remove2.push_back({route1[l + count].start,route1[l + last_count_not_removed].start,0});
						new_remove2.push_back({route1[l + count].end,route1[l + count].start,0});
						new_add2.push_back({route1[l + count].end,route1[l + last_count_not_removed].start,0});
					}
				}
			}  
		}
	}

	removeCancellingArcs(new_add2, new_remove2);
}

// Returns cost improvement when relocating subroute in positions l ... k from route1 before position j in route 2 (or-exchange)
double checkOrExchangeInterRoute(vector<arc> route1, vector<arc> route2, int j, int l, int k, bool &invert, double old_per1, double &new_per1, double old_per2, double &new_per2, double old_mean1, double &new_mean1, double old_mean2, double &new_mean2, vector<double> &new_Route_values1, vector<double> &new_Route_values2)
{
	// no inversion
	add1.clear();
	remove1.clear();
	remove1.push_back(route1[l - 1]);
	remove1.push_back(route1[k]);
	add1.push_back({route1[l - 1].start,route1[k].end,0});
	add2.clear();
	remove2.clear();
	remove2.push_back(route2[j - 1]);
	add2.push_back({route2[j - 1].start,route1[l].start,0});
	add2.push_back({route1[k].start,route2[j - 1].end,0});
	for (int count = 0; count <= (k - l); count++)
	{
		new_Route_values1[2] -= 15 + 5 * route1[l + count].boarding;
		new_Route_values2[2] += 15 + 5 * route1[l + count].boarding;
		if (count < (k - l)) {
			remove1.push_back({route1[l + count]});
			add2.push_back({route1[l + count]});
		}
	}
	updateRouteParameters(route1, remove1, add1, new_Route_values1, new_per1, new_mean1);	
	new_Route_values2_inv = new_Route_values2;
	checkRemoveDuplicatesOrExchange(route1, route2, j, l, k, false, add2, remove2, new_Route_values2);
	updateRouteParameters(route2, remove2, add2, new_Route_values2, new_per2, new_mean2);	
	cost_improvement = costImprovement(old_per1, new_per1, old_mean1, new_mean1, old_per2, new_per2, old_mean2, new_mean2);
	
	// with inversion
	add2.clear();
	add2.push_back({route2[j - 1].start,route1[k].start,0});
	add2.push_back({route1[l].start,route2[j - 1].end,0});
	for (int count = 0; count < (k - l); count++) 
		add2.push_back({route1[k - count - 1].end,route1[k - count - 1].start,0}); 
	checkRemoveDuplicatesOrExchange(route1, route2, j, l, k, true, add2, remove2, new_Route_values2_inv);
	updateRouteParameters(route2, remove2, add2, new_Route_values2_inv, new_per2_inv, new_mean2_inv);	
	cost_improvement_temp = costImprovement(old_per1, new_per1, old_mean1, new_mean1, old_per2, new_per2_inv, old_mean2, new_mean2_inv);

	if (cost_improvement < cost_improvement_temp) {
		invert = true;
		new_per2 = new_per2_inv;
		new_mean2 = new_mean2_inv;
		new_Route_values2 = new_Route_values2_inv;
		return cost_improvement_temp;
	}
	else {
		invert = false;
		return cost_improvement;
	}
}

// Amends add2 to remove duplicate stops from route2 when swapping subroute in positions l ... k
// from route1 with subroute in positions j ... i in route 2 (cross-exchange duplicates)
void checkRemoveDuplicatesCrossExchange(vector<arc> route1, vector<arc> route2, int l, int k, int j, int i, bool invert, vector<arc> &new_add2, vector<arc> &new_remove2, vector<double> &new_Route_values2)
{
	int last_count_not_removed = -1; 		// to keep record of which stop is next to the current one being removed
	int position;

	for (int count = 0; count <= (k - l); count++) {

		position = findStopInRoute(route2, route1[l + count].start);

		if (position == -1 || (position >= j && position <= i))		last_count_not_removed = count;	
		else 
		{	// we need to remove the duplicate stop
			new_Route_values2[2] -= 15;

			if (k - l == 0) { // one element relocated (in position l) and it is already in route2
				new_add2.push_back({route2[j - 1].start,route2[i].end,0});
				new_remove2.push_back({route2[j - 1].start,route1[l].start,0});
				new_remove2.push_back({route1[l].start,route2[i].end,0});
			}

			else if (count == 0) { // first element (in position l) relocated is already in route2
				if (!invert) {
					new_remove2.push_back(route1[l]);
					new_remove2.push_back({route2[j - 1].start,route1[l].start,0});
					new_add2.push_back({route2[j - 1].start,route1[l].end,0});	
				}
				else {
					new_remove2.push_back({route1[l].end,route1[l].start,0});
					new_remove2.push_back({route1[l].start,route2[i].end,0});
					new_add2.push_back({route1[l].end,route2[i].end,0});	
				}
			}

			else if (count == (k - l)) {  // last element (in position k) relocated is already in route2
				if (last_count_not_removed == -1) {	// all elements up to (and including) stop at position k are duplicates
					new_remove2.push_back({route2[j - 1].start,route1[k].start,0});
					new_remove2.push_back({route1[k].start,route2[i].end,0});
					new_add2.push_back({route2[j - 1].start,route2[i].end,0});	
				}
				else
				{
					if (!invert) {
						new_remove2.push_back({route1[l + last_count_not_removed].start,route1[k].start,0});
						new_remove2.push_back({route1[k].start,route2[i].end,0});
						new_add2.push_back({route1[l + last_count_not_removed].start,route2[i].end,0});
					}
					else {
						new_remove2.push_back({route2[j - 1].start,route1[k].start,0});
						new_remove2.push_back({route1[k].start,route1[l + last_count_not_removed].start,0});
						new_add2.push_back({route2[j - 1].start,route1[l + last_count_not_removed].start,0});
					}
				}
			}

			else {	// element in position (l+count) where 0 < count < (k-l) relocated is already in route2
				if (last_count_not_removed == -1) {  // all elements up to (and including) stop at position (l+count) are duplicates
					if (!invert) {
						new_remove2.push_back({route2[j - 1].start,route1[l + count].start,0});
						new_remove2.push_back(route1[l + count]);
						new_add2.push_back({route2[j - 1].start,route1[l + count].end,0});
					}
					else {
						new_remove2.push_back({route1[l + count].end,route1[l + count].start,0});
						new_remove2.push_back({route1[l + count].start,route2[i].end,0});
						new_add2.push_back({route1[l + count].end,route2[i].end,0});	
					}
				}
				else {
					if (!invert) {
						new_remove2.push_back({route1[l + last_count_not_removed].start,route1[l + count].start,0});
						new_remove2.push_back(route1[l + count]);
						new_add2.push_back({route1[l + last_count_not_removed].start,route1[l + count].end,0});
					}
					else {
						new_remove2.push_back({route1[l + count].start,route1[l + last_count_not_removed].start,0});
						new_remove2.push_back({route1[l + count].end,route1[l + count].start,0});
						new_add2.push_back({route1[l + count].end,route1[l + last_count_not_removed].start,0});
					}
				}
			}  
		}
	}

	removeCancellingArcs(new_add2, new_remove2);
}

// Returns cost improvement when swapping subroute in positions l ... k from route1 with subroute in positions j ... i in route 2 (cross-exchange)
double checkCrossExchangeInterRoute(vector<arc> route1, vector<arc> route2, int l, int k, int j, int i, bool &invert1, bool &invert2, double old_per1, double &new_per1, double old_per2, double &new_per2, double old_mean1, double &new_mean1, double old_mean2, double &new_mean2, vector<double> &new_Route_values1, vector<double> &new_Route_values2)
{
	// no inversion in both routes
	add1.clear();
	remove1.clear();
	remove1.push_back(route1[l - 1]);
	remove1.push_back(route1[k]);
	add1.push_back({route1[l - 1].start,route2[j].start,0});
	add1.push_back({route2[i].start,route1[k].end,0});
	add2.clear();
	remove2.clear();
	remove2.push_back(route2[j - 1]);
	remove2.push_back(route2[i]);
	add2.push_back({route2[j - 1].start,route1[l].start,0});
	add2.push_back({route1[k].start,route2[i].end,0});
	for (int count1 = 0; count1 <= (k - l); count1++)
	{
		new_Route_values1[2] -= 15 + 5 * route1[l + count1].boarding;
		new_Route_values2[2] += 15 + 5 * route1[l + count1].boarding;
		if (count1 < (k - l)) {
			remove1.push_back(route1[l + count1]);
			add2.push_back(route1[l + count1]);
		}
	}
	for (int count2 = 0; count2 <= (i - j); count2++)
	{
		new_Route_values1[2] += 15 + 5 * route2[j + count2].boarding;
		new_Route_values2[2] -= 15 + 5 * route2[j + count2].boarding;
		if (count2 < (i - j))
		{
			remove2.push_back(route2[j + count2]);
			add1.push_back(route2[j + count2]);		
		}
	}
	new_Route_values1_inv = new_Route_values1;
	new_Route_values2_inv = new_Route_values2;
	remove1_inv = remove1;
	remove2_inv = remove2;
	// removing duplicates (if any) in route 1 (invert = false)
	checkRemoveDuplicatesCrossExchange(route2, route1, j, i, l, k, false, add1, remove1, new_Route_values1);
	updateRouteParameters(route1, remove1, add1, new_Route_values1, new_per1, new_mean1);	
	// removing duplicates (if any) in route 2 (invert = false)
	checkRemoveDuplicatesCrossExchange(route1, route2, l, k, j, i, false, add2, remove2, new_Route_values2);
	updateRouteParameters(route2, remove2, add2, new_Route_values2, new_per2, new_mean2);	
	new_per2_noinv = new_per2;							// creating a copy of percentile of route 2 (no inversion)
	new_mean2_noinv = new_mean2;						// creating a copy of mean of route 2 (no inversion)
	new_Route_values2_noinv = new_Route_values2;    	// creating a copy of new_Route_values of route 2 (no inversion)
	cost_improvement = costImprovement(old_per1, new_per1, old_mean1, new_mean1, old_per2, new_per2, old_mean2, new_mean2);
	invert1 = false;
	invert2 = false;
	
	// with inversion in route 2 only 
	add2.clear();
	add2.push_back({route2[j - 1].start,route1[k].start,0});
	add2.push_back({route1[l].start,route2[i].end,0});
	for (int count1 = 0; count1 < (k - l); count1++) 
		add2.push_back({route1[k - count1 - 1].end,route1[k - count1 - 1].start,0});
	// removing duplicates (if any) in route 2 (invert = true)
	checkRemoveDuplicatesCrossExchange(route1, route2, l, k, j, i, true, add2, remove2_inv, new_Route_values2_inv); 
	updateRouteParameters(route2, remove2_inv, add2, new_Route_values2_inv, new_per2_inv, new_mean2_inv);	
	cost_improvement_temp = costImprovement(old_per1, new_per1, old_mean1, new_mean1, old_per2, new_per2_inv, old_mean2, new_mean2_inv);
	if (cost_improvement < cost_improvement_temp) {
		cost_improvement = cost_improvement_temp;
		invert2 = true;
		new_per2 = new_per2_inv;
		new_mean2 = new_mean2_inv;
		new_Route_values2 = new_Route_values2_inv;
	}

	// with inversion in route 1 only
	add1.clear();
	add1.push_back({route1[l - 1].start,route2[i].start,0});
	add1.push_back({route2[j].start,route1[k].end,0});
	for (int count2 = 0; count2 < (i - j); count2++) 
		add1.push_back({route2[i - count2 - 1].end,route2[i - count2 - 1].start,0});
	// removing duplicates (if any) in route 1 (invert = true)
	checkRemoveDuplicatesCrossExchange(route2, route1, j, i, l, k, true, add1, remove1_inv, new_Route_values1_inv); 
	updateRouteParameters(route1, remove1_inv, add1, new_Route_values1_inv, new_per1_inv, new_mean1_inv);	
	cost_improvement_temp = costImprovement(old_per1, new_per1_inv, old_mean1, new_mean1_inv, old_per2, new_per2_noinv, old_mean2, new_mean2_noinv);
	if (cost_improvement < cost_improvement_temp) {
		cost_improvement = cost_improvement_temp;
		invert1 = true;
		invert2 = false;
		new_per1 = new_per1_inv;
		new_per2 = new_per2_noinv;
		new_mean1 = new_mean1_inv;
		new_mean2 = new_mean2_noinv;
		new_Route_values1 = new_Route_values1_inv;
		new_Route_values2 = new_Route_values2_noinv;
	}

	// with inversion in both routes 1 and 2
	cost_improvement_temp = costImprovement(old_per1, new_per1_inv, old_mean1, new_mean1_inv, old_per2, new_per2_inv, old_mean2, new_mean2_inv);
	if (cost_improvement < cost_improvement_temp) {
		cost_improvement = cost_improvement_temp;
		invert1 = true;
		invert2 = true;
		new_per1 = new_per1_inv;
		new_per2 = new_per2_inv;
		new_mean1 = new_mean1_inv;
		new_mean2 = new_mean2_inv;
		new_Route_values1 = new_Route_values1_inv;
		new_Route_values2 = new_Route_values2_inv;
	} 

	return cost_improvement;
}

// Returns cost improvement when transferring students from stop in position x in route1 (with percentile exceeding MRT) 
// in front of "position" in route2 (if stop is not already in route2)
double checkCreateMultistop(int cap2, vector<arc> route1, vector<arc> route2, int route_load2, int x, bool &present, int &position, int &transfer, double old_per1, double &new_per1, double old_per2, double &new_per2, double old_mean1, double &new_mean1, double old_mean2, double &new_mean2, vector<double> &new_Route_values1, vector<double> &new_Route_values2)
{
	transfer = min(route1[x].boarding - 1, cap2 - route_load2);
	new_Route_values1[2] -= 5*transfer;
	new_per1 = old_per1 - 5*transfer;
	new_mean1 = old_mean1 - 5*transfer;
	new_Route_values2[2] += 5*transfer;
	
	position = findStopInRoute(route2, route1[x].start);
	if (position != -1) // i.e., stop already in route2
	{
		present = true; 
		new_per2 = old_per2 + 5*transfer;
		new_mean2 = old_mean2 + 5*transfer;
		return costImprovement(old_per1, new_per1, old_mean1, new_mean1,old_per2, new_per2, old_mean2, new_mean2);
	} 
	else	// find the best insertion point in route2
	{
		present = false;
		new_Route_values2[2] += 15;
		best_cost_improvement = maxno;
		for (int z = 1; z <= route2.size(); z++) {
			new_Route_values_temp = new_Route_values2;
			add2.clear();
			remove2.clear();
			remove2.push_back(route2[z - 1]);
			add2.push_back({route2[z - 1].start,route1[x].start,0});
			add2.push_back({route1[x].start,route2[z - 1].end,0});
			updateRouteParameters(route2, remove2, add2, new_Route_values_temp, new_per_temp, new_mean_temp);	
			cost_improvement = costImprovement(old_per1, new_per1, old_mean1, new_mean1, old_per2, new_per_temp, old_mean2, new_mean_temp);
			if (cost_improvement < best_cost_improvement) {
				best_cost_improvement = cost_improvement;
				position = z;
				new_per2 = new_per_temp;
				new_mean2 = new_mean_temp;
				best_new_Route_values_temp = new_Route_values_temp;
			}
		}
		new_Route_values2 = best_new_Route_values_temp;
		return best_cost_improvement;
	}
}

// ------------------------------------------------------------ LOCAL SEARCH MOVE EXECUTIONS ---------------------------------------------------------------------------------------

// Removes duplicate stops from subroute (which are already in route) before 
// relocating subroute before position j in route (or-exchange duplicates)
void removeDuplicatesOrExchange(vector<stop_board_compulsory_info> &route, vector<stop_board_compulsory_info> &subroute)
{
	int position, i = 0;
	while (i < subroute.size())
	{
		position = findStopInVec(route, subroute[i].num);
		if (position != -1)
		{
			route[position].boarding += subroute[i].boarding;
			subroute.erase(subroute.begin() + i);
		}
		else
			i++;
	}
}

// Removes duplicate stops from sub_route (which are already in route) before 
// swapping subroute in position pos1 ... pos2 in route with sub_route (cross-exchange duplicates)
void removeDuplicatesCrossExchange(vector<stop_board_compulsory_info> &route, vector<stop_board_compulsory_info> &sub_route, int pos1, int pos2)
{
	int position, i = 0;
	while (i < sub_route.size())
	{
		position = findStopInVec(route, sub_route[i].num);
		if ((position != -1) && (position < pos1 || position > pos2))
		{
			route[position].boarding += sub_route[i].boarding;
			sub_route.erase(sub_route.begin() + i);
		}
		else
			i++;
	}
}

// Converts route from stop_board_compulsory_info format to arc format
vector<arc> convertRouteToArcFormat(vector<stop_board_compulsory_info> route) {
	vector<arc> converted_route;
	for (int i = 0; i < route.size() - 1; i++) 	converted_route.push_back({route[i].num,route[i + 1].num,route[i].boarding});
	return converted_route;
}

void localSearch(vector<vector<stop_board_compulsory_info> > &Routes, vector<vector<double> > &Routes_values, vector<double> &Routes_percentiles, vector<double> &Routes_means)
{
	double bestCostImprovement = 1, newCostImprovement = 0, bestDiscrepancy;
	double new_per1, new_per2, new_mean1, new_mean2, best_new_per1, best_new_per2, best_new_mean1, best_new_mean2;
	bool invert_subroute_goinginto1, invert_subroute_goinginto2, best_invert_subroute_goinginto1, best_invert_subroute_goinginto2, present, best_present;
	int route_load1, route_load2, demand_inc1, demand_inc2, best_index_route1, best_index_route2, best_l, best_k, best_j, best_i, best_x, position, best_position, transfer, best_transfer;
	int best_move;				// 1 = swap; 2 = 2-opt; 3 = extended-or-opt; 4 = or-exchange; 5 = cross-exchange; 6 = multistop
	vector<vector<stop_board_compulsory_info> > Routes_copy = Routes;
	vector<stop_board_compulsory_info> subroute1, subroute2;
	vector<double> new_route_values1, new_route_values2, best_new_route_values1, best_new_route_values2;
	vector<arc> route1_arc, route2_arc; 

	// removing empty routes before LS------------------------------------------------------------------------------
	int route_index = 0; 
	while (route_index < Routes.size()) {
		if (Routes[route_index][0].num == 0 and Routes[route_index][1].num == 0) {
				Routes.erase(Routes.begin() + route_index);
				Routes_values.erase(Routes_values.begin() + route_index);
				Routes_percentiles.erase(Routes_percentiles.begin() + route_index);
				Routes_means.erase(Routes_means.begin() + route_index);
		}
		else route_index++;
	} 
	//--------------------------------------------------------------------------------------------------------------

	while (bestCostImprovement > 0)
	{
		bestCostImprovement = 0;
		bestDiscrepancy = *max_element(Routes_percentiles.begin(), Routes_percentiles.end()) - *min_element(Routes_percentiles.begin(), Routes_percentiles.end());
		
		// INTRA-ROUTE OPERATORS
		for (int m = 0; m < Routes.size(); m++)
		{
			route_load1 = routeLoad(Routes[m]);	
			route1_arc = convertRouteToArcFormat(Routes[m]);
			for (int l = 1; l < Routes[m].size() - 1; l++)
			{
				for (int k = l; k < Routes[m].size() - 1; k++)
				{
					
					// swap
					if (k >= (l + 1) && l < (Routes[m].size() - 2))
					{
						new_route_values1 = Routes_values[m];
						newCostImprovement = checkSwapIntraRoute(route1_arc, l, k, Routes_percentiles[m], new_per1, Routes_means[m], new_mean1, new_route_values1);
						if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && 
						(routeBalancing(Routes_percentiles, m, new_per1, -1, 0) < bestDiscrepancy)))
						{
							bestCostImprovement = newCostImprovement;
							bestDiscrepancy = routeBalancing(Routes_percentiles, m, new_per1, -1, 0); 
							best_new_per1 = new_per1;
							best_new_mean1 = new_mean1;
							best_new_route_values1 = new_route_values1;
							best_index_route1 = m;
							best_l = l;
							best_k = k;
							best_move = 1;
						}
					}
					
					// 2-opt
					if (k >= (l + 3) && l < (Routes[m].size() - 2))
					{
						new_route_values1 = Routes_values[m];
						newCostImprovement = check2OptIntraRoute(route1_arc, l, k, Routes_percentiles[m], new_per1, Routes_means[m], new_mean1, new_route_values1);
						if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && 
						(routeBalancing(Routes_percentiles, m, new_per1, -1, 0) < bestDiscrepancy)))
						{
							bestCostImprovement = newCostImprovement;
							bestDiscrepancy = routeBalancing(Routes_percentiles, m, new_per1, -1, 0); 
							best_new_per1 = new_per1;
							best_new_mean1 = new_mean1;
							best_new_route_values1 = new_route_values1;
							best_index_route1 = m;
							best_l = l;
							best_k = k;
							best_move = 2;
						}
					}
					
					// extended-or-opt
					for (int j = 1; j < Routes[m].size(); j++)
					{
						if (j < l || j > k + 1)
						{
							new_route_values1 = Routes_values[m];
							newCostImprovement = checkExtendedOrOptIntraRoute(route1_arc, j, l, k, invert_subroute_goinginto1, Routes_percentiles[m], new_per1, Routes_means[m], new_mean1, new_route_values1);
							if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && 
							(routeBalancing(Routes_percentiles, m, new_per1, -1, 0) < bestDiscrepancy)))
							{
								bestCostImprovement = newCostImprovement;
								bestDiscrepancy = routeBalancing(Routes_percentiles, m, new_per1, -1, 0); 
								best_new_per1 = new_per1;
								best_new_mean1 = new_mean1;	
								best_new_route_values1 = new_route_values1;				
								best_invert_subroute_goinginto1 = invert_subroute_goinginto1;
								best_index_route1 = m;
								best_l = l;
								best_k = k;
								best_j = j;
								best_move = 3;
							}
						}
					}  
				}
			}
		} 
		
		
		// INTER-ROUTE OPERATORS
		for (int m = 0; m < Routes.size(); m++)
		{
			route_load1 = routeLoad(Routes[m]); 
			route1_arc = convertRouteToArcFormat(Routes[m]);
			
			for (int n = 0; n < Routes.size(); n++)
			{
				if (n != m)
				{
					route_load2 = routeLoad(Routes[n]);
					route2_arc = convertRouteToArcFormat(Routes[n]);
					for (int l = 1; l < Routes[m].size() - 1; l++)
					{
						for (int k = l; k < Routes[m].size() - 1; k++)
						{	
							demand_inc2 = 0;
							for (int count = 0; count <= (k - l); count++)	demand_inc2 += Routes[m][l + count].boarding;
							
							// or-exchange
							if (route_load2 + demand_inc2 <= cap[n])
							{
								for (int j = 1; j < Routes[n].size(); j++)
								{
									new_route_values1 = Routes_values[m];
									new_route_values2 = Routes_values[n];
									newCostImprovement = checkOrExchangeInterRoute(route1_arc, route2_arc, j, l, k, invert_subroute_goinginto2, Routes_percentiles[m], new_per1, Routes_percentiles[n], new_per2, Routes_means[m], new_mean1, Routes_means[n], new_mean2, new_route_values1, new_route_values2);
									if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && 
									(routeBalancing(Routes_percentiles, m, new_per1, n, new_per2) < bestDiscrepancy)))
									{
										bestCostImprovement = newCostImprovement;
										bestDiscrepancy = routeBalancing(Routes_percentiles, m, new_per1, n, new_per2); 
										best_new_per1 = new_per1;
										best_new_mean1 = new_mean1;	
										best_new_per2 = new_per2;
										best_new_mean2 = new_mean2;
										best_new_route_values1 = new_route_values1;
										best_new_route_values2 = new_route_values2;
										best_invert_subroute_goinginto2 = invert_subroute_goinginto2;
										best_index_route1 = m;
										best_index_route2 = n;
										best_l = l;
										best_k = k;
										best_j = j;
										best_move = 4;
									}
								}
							}
							
							if (n > m)
							{	
								// cross-exchange
								for (int j = 1; j < Routes[n].size() - 1; j++)
								{
									for (int i = j; i < Routes[n].size() - 1; i++)
									{
										demand_inc1 = 0;
										for (int count = 0; count <= (i - j); count++)	demand_inc1 += Routes[n][j + count].boarding;
										if ((route_load1 - demand_inc2 + demand_inc1 <= cap[m]) & (route_load2 - demand_inc1 + demand_inc2 <= cap[n]) & !((l == 1) & (j == 1) & (k == Routes[m].size() - 2) & (i == Routes[n].size() - 2)))
										{	
											new_route_values1 = Routes_values[m];
											new_route_values2 = Routes_values[n];
											newCostImprovement = checkCrossExchangeInterRoute(route1_arc, route2_arc, l, k, j, i, invert_subroute_goinginto1, invert_subroute_goinginto2, Routes_percentiles[m], new_per1, Routes_percentiles[n], new_per2, Routes_means[m], new_mean1, Routes_means[n], new_mean2, new_route_values1, new_route_values2);
											if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && 
											(routeBalancing(Routes_percentiles, m, new_per1, n, new_per2) < bestDiscrepancy)))
											{
												bestCostImprovement = newCostImprovement;
												bestDiscrepancy = routeBalancing(Routes_percentiles, m, new_per1, n, new_per2); 
												best_new_per1 = new_per1;
												best_new_mean1 = new_mean1;	
												best_new_per2 = new_per2;
												best_new_mean2 = new_mean2;
												best_new_route_values1 = new_route_values1;
												best_new_route_values2 = new_route_values2;
												best_invert_subroute_goinginto1 = invert_subroute_goinginto1;
												best_invert_subroute_goinginto2 = invert_subroute_goinginto2;
												best_index_route1 = m;
												best_index_route2 = n;
												best_l = l;
												best_k = k;
												best_j = j;
												best_i = i;
												best_move = 5;
											}  
										} 
									}
								}		 
							} 
						}
					}  
				}
			} 
			
			// multi-stop
			if (Routes_percentiles[m] > MRT)			// trying to create multistop if 95th percentile > MRT
			{ 
				for (int x = 1; x < Routes[m].size() - 1; x++)
				{
					if (Routes[m][x].boarding >= 2)
					{
						for (int n = 0; n < Routes.size(); n++)
						{
							route_load2 = routeLoad(Routes[n]);
							route2_arc = convertRouteToArcFormat(Routes[n]);
							if (n != m && route_load2 < cap[n])
							{
								new_route_values1 = Routes_values[m];
								new_route_values2 = Routes_values[n];
								newCostImprovement = checkCreateMultistop(cap[n], route1_arc, route2_arc, route_load2, x, present, position, transfer, Routes_percentiles[m], new_per1, Routes_percentiles[n], new_per2, Routes_means[m], new_mean1, Routes_means[n], new_mean2, new_route_values1, new_route_values2); 
								if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && 
								(routeBalancing(Routes_percentiles, m, new_per1, n, new_per2) < bestDiscrepancy)))
								{
									bestCostImprovement = newCostImprovement;
									bestDiscrepancy = routeBalancing(Routes_percentiles, m, new_per1, n, new_per2); 
									best_new_per1 = new_per1;
									best_new_mean1 = new_mean1;	
									best_new_per2 = new_per2;
									best_new_mean2 = new_mean2;
									best_new_route_values1 = new_route_values1;
									best_new_route_values2 = new_route_values2;
									best_index_route1 = m;
									best_index_route2 = n;
									best_x = x;
									best_position = position;
									best_transfer = transfer;
									best_present = present;
									best_move = 6;
								}
							}
						}
					}
				}
			} 
		}
		
		if (bestCostImprovement != 0)				
		{
			if (best_move == 1) {
				swap(Routes[best_index_route1][best_l], Routes[best_index_route1][best_k]);
			}
			
			else if (best_move == 2)	{
				reverse(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + best_k + 1);
			}
			
			else if (best_move == 3)	{
				subroute1.clear();
				subroute1.assign(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + (best_k + 1));
				if (best_invert_subroute_goinginto1 == true)	reverse(subroute1.begin(), subroute1.end());
				if (best_j < best_l)
				{
					Routes[best_index_route1].erase(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + (best_k + 1));
					Routes[best_index_route1].insert(Routes[best_index_route1].begin() + best_j, subroute1.begin(), subroute1.end());
				}
				else
				{
					Routes[best_index_route1].insert(Routes[best_index_route1].begin() + best_j, subroute1.begin(), subroute1.end());
					Routes[best_index_route1].erase(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + (best_k + 1));
				}
			}
			
			else if (best_move == 4)
			{
				subroute1.clear();
				subroute1.assign(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + (best_k + 1));
				subroute2 = subroute1;
				removeDuplicatesOrExchange(Routes[best_index_route2], subroute1); 	// removing duplicate vertices from subroute1 (which are already in route2)
				if (best_invert_subroute_goinginto2 == true)	reverse(subroute1.begin(), subroute1.end());
				Routes[best_index_route1].erase(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + (best_k + 1));
				Routes[best_index_route2].insert(Routes[best_index_route2].begin() + best_j, subroute1.begin(), subroute1.end());
			}	
			
			else if (best_move == 5)
			{
				subroute1.clear();
				subroute1.assign(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + (best_k + 1));
				subroute2.clear();
				subroute2.assign(Routes[best_index_route2].begin() + best_j, Routes[best_index_route2].begin() + (best_i + 1));
				removeDuplicatesCrossExchange(Routes[best_index_route1], subroute2, best_l, best_k); // removing duplicate vertices from subroute2 (which are already in route1)
				removeDuplicatesCrossExchange(Routes[best_index_route2], subroute1, best_j, best_i); // removing duplicate vertices from subroute1 (which are already in route2)
				if (best_invert_subroute_goinginto2 == true)	reverse(subroute1.begin(), subroute1.end());
				if (best_invert_subroute_goinginto1 == true)	reverse(subroute2.begin(), subroute2.end());
				Routes[best_index_route1].erase(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + (best_k + 1));
				Routes[best_index_route1].insert(Routes[best_index_route1].begin() + best_l, subroute2.begin(), subroute2.end());
				Routes[best_index_route2].erase(Routes[best_index_route2].begin() + best_j, Routes[best_index_route2].begin() + (best_i + 1));
				Routes[best_index_route2].insert(Routes[best_index_route2].begin() + best_j, subroute1.begin(), subroute1.end());	
			}
			
			else if (best_move == 6)
			{
				Routes[best_index_route1][best_x].boarding -= best_transfer;
				if (best_present == false)		// stop is not already in route2
					Routes[best_index_route2].insert(Routes[best_index_route2].begin() + best_position, {Routes[best_index_route1][best_x].num, best_transfer, Routes[best_index_route1][best_x].compulsory});
				else	Routes[best_index_route2][best_position].boarding += best_transfer;
			}
			
			Routes_percentiles[best_index_route1] = best_new_per1;
			Routes_means[best_index_route1] = best_new_mean1;
			Routes_values[best_index_route1] = best_new_route_values1;
			if (best_move >= 4) {
				Routes_percentiles[best_index_route2] = best_new_per2;
				Routes_means[best_index_route2] = best_new_mean2;
				Routes_values[best_index_route2] = best_new_route_values2;
			}	
		}
	}
}

// ---------------------------------------------------------------- SOLUTION ALTERATION ---------------------------------------------------------------------------------------------

// Alters a vector of used stops by removing some of its stops and adding others (if required)
void alterUsedStops(vector<stop_board_compulsory_info> &used_stops, mt19937 &generator, vector<vector<double> > walk, int all_stops, int addresses, vector<vector<stop_walk_info> > &available_stops, vector<vector<int> > &available_addresses, vector<int> &w, vector<address_info> vec_addresses)
{
	// Creating a vector of non-compulsory stops
	vector<stop_board_compulsory_info> non_compulsory_stops;
	for (int i = 0; i < used_stops.size(); i++)
		if (used_stops[i].compulsory == 0)	non_compulsory_stops.push_back(used_stops[i]);

	// Identifying a Binomial random number of non-compulsory stops to remove with n = non_compulsory_stops.size() and p = 3/non_compulsory_stops.size()
	binomial_distribution<int> bin_distribution(non_compulsory_stops.size(), 3.0 / non_compulsory_stops.size());
	int no_to_remove = 0;
	while (no_to_remove == 0)	no_to_remove = bin_distribution(generator);		// making sure random number is non-zero

	// Choosing no_to_remove random non-compulsory stops to remove
	int random_index, current_no_removed_stops = 0;
	vector<stop_board_compulsory_info> removed_stops;
	while (current_no_removed_stops < no_to_remove)
	{
		uniform_int_distribution<int> uni_distribution(0, non_compulsory_stops.size() - 1);
		random_index = uni_distribution(generator);
		for (int i = 0; i < used_stops.size(); i++)
		{
			if (used_stops[i].num == non_compulsory_stops[random_index].num)
			{
				removed_stops.push_back(used_stops[i]);
				used_stops.erase(used_stops.begin() + i);
				non_compulsory_stops.erase(non_compulsory_stops.begin() + random_index);
				current_no_removed_stops++;
				break;
			}
		}
	}

	// Creating a vector of stops that are neither in removed_stops nor in used_stops
	vector<stop_board_compulsory_info> remaining_stops = allStopsWithoutTwoVectorsOfStops(all_stops, removed_stops, used_stops);

	// Checking whether any address does not have an available stop within maximum walking distance
	vector<int> addresses_uncovered_used_stops, addresses_uncovered_remaining_stops, subset_addresses;
	for (int s = 1; s <= addresses; s++)	subset_addresses.push_back(s);
	identifyAvailableStops(walk, subset_addresses, used_stops, available_stops);

	for (int s = 0; s < addresses; s++)
		if (available_stops[s].size() == 0)		addresses_uncovered_used_stops.push_back(s + 1);

	if (addresses_uncovered_used_stops.size() != 0)
	{
		// Checking whether addresses in addresses_uncovered_used_stops have an alternative stop in remaining_stops
		identifyAvailableStops(walk, addresses_uncovered_used_stops, remaining_stops, available_stops);

		int s = 0;
		while (s < addresses_uncovered_used_stops.size())
		{
			if (available_stops[s].size() == 0)
			{
				addresses_uncovered_remaining_stops.push_back(addresses_uncovered_used_stops[s]);
				addresses_uncovered_used_stops.erase(addresses_uncovered_used_stops.begin() + s);
			}
			else	s++;
		}

		// If some addresses in addresses_uncovered_used_stops do not have an alternative stop, then at least one stop from removed_stops must be added back
		if (addresses_uncovered_remaining_stops.size() > 0)		coverAddresses(addresses_uncovered_remaining_stops, used_stops, walk, removed_stops, available_addresses, generator);

		// Adding new stops to cover the addresses still left in addresses_uncovered_used_stops (i.e. those that have an alternative stop in remaining_stops)
		coverAddresses(addresses_uncovered_used_stops, used_stops, walk, remaining_stops, available_addresses, generator);
	}

	// Sorting used_stops and updating demands
	sort(used_stops.begin(), used_stops.end(), orderStops);
	updateDemands(addresses, walk, available_stops, used_stops, w, vec_addresses);
}

// Used to order the occurrences of a stop in Routes in non-increasing order of the number of boarding students - Returns True if a.boarding > b.boarding
bool compareBoarding(const route_position_board_load a, const route_position_board_load b)
{
	return a.boarding > b.boarding;
}

// Used to order the occurrences of a stop in Routes in non-decreasing order of the vehicle loads - Returns True if a.load < b.load
bool compareLoad(const route_position_board_load a, const route_position_board_load b)
{
	return a.load < b.load;
}

// Compares two vector of used stops to determine which stops should be removed/added and any changes in the demands
void compareTwoUsedStops(vector<stop_board_compulsory_info> used_stops_old, vector<stop_board_compulsory_info> used_stops_new, vector<stop_board_compulsory_info> &to_add, vector<stop_board_compulsory_info> &to_remove)
{
	to_add.clear();
	to_remove.clear();
	int pos;

	// Inserting stops in used_stops_old but not in used_stops_new to vector "to_remove"
	for (int i = 0; i < used_stops_old.size(); i++)
	{
		pos = findStopInVec(used_stops_new, used_stops_old[i].num);
		if (pos == -1)
			to_remove.push_back(used_stops_old[i]);
	}

	for (int i = 0; i < used_stops_new.size(); i++)
	{
		pos = findStopInVec(used_stops_old, used_stops_new[i].num);

		// Inserting stops in used_stops_new but not in used_stops_old to vector "to_add"
		if (pos == -1)
			to_add.push_back(used_stops_new[i]);

		// Dealing with stops in both used_stops_old and used_stops_new whose pair of demands are different
		else
		{
			if (used_stops_new[i].boarding != used_stops_old[pos].boarding)
			{
				stop_board_compulsory_info changing_stop = {used_stops_new[i].num, (used_stops_new[i].boarding - used_stops_old[pos].boarding), used_stops_new[i].compulsory};
				if (changing_stop.boarding < 0)
					to_remove.push_back(changing_stop);
				// an element in to_remove whose demand is negative implies that its demand must be decreased
				else
				{
					changing_stop.boarding *= -1;
					to_add.push_back(changing_stop);
				}
				// an element in to_add whose demand is negative implies that its demand must be increased
			}
		}
	}
}

// Lists down all occurrences of a stop in Routes - returns a vector of four tuples (route index, route position, boarding, route load)
void occurrencesOfStopInRoutes(vector<vector<stop_board_compulsory_info> > Routes, stop_board_compulsory_info stop, vector<route_position_board_load> &occurrences)
{
	for (int i = 0; i < Routes.size(); i++)
	{
		for (int j = 1; j < Routes[i].size() - 1; j++)
		{
			if (Routes[i][j].num == stop.num)
			{
				route_position_board_load occurrence = {i, j, Routes[i][j].boarding, routeLoad(Routes[i])};
				occurrences.push_back(occurrence);
				break;
			}
		}
	}
}

// Removes all occurrences of a stop from Routes - used for elements in to_remove having positive demands
void removeStopFromRoutes(vector<vector<stop_board_compulsory_info> > &Routes, stop_board_compulsory_info stop, vector<vector<double> > &Routes_values, vector<double> &Routes_percentiles, vector<double> &Routes_means)
{
	for (int i = 0; i < Routes.size(); i++)
	{
		int j = 1;
		while (j < Routes[i].size() - 1)
		{
			if (Routes[i][j].num == stop.num)
			{
				route_temp = convertRouteToArcFormat(Routes[i]);
				arc_add.clear();
				arc_remove.clear();
				arc_remove.push_back(route_temp[j - 1]);
				arc_remove.push_back(route_temp[j]);
				arc_add.push_back({route_temp[j - 1].start,route_temp[j].end,0});
				Routes_values[i][2] -= 15 + 5*Routes[i][j].boarding;  	
				updateRouteParameters(route_temp, arc_remove, arc_add, Routes_values[i], Routes_percentiles[i], Routes_means[i]);
				Routes[i].erase(Routes[i].begin() + j);
				break;
			}
			else	j++;
		}
	}
}

// Removes some demand of a stop from Routes - used for elements in to_remove having negative demands
void decreaseDemandOfStopInRoutes(vector<vector<stop_board_compulsory_info> > &Routes, stop_board_compulsory_info stop, vector<vector<double> > &Routes_values, vector<double> &Routes_percentiles, vector<double> &Routes_means)
{	
	int decrease = -stop.boarding; // by how much demand of stop must decrease
	
	vector<route_position_board_load> occurrences;
	occurrencesOfStopInRoutes(Routes, stop, occurrences);
	sort(occurrences.begin(), occurrences.end(), compareBoarding);
	int route_ind;
	while (decrease > 0)
	{	
		route_ind = occurrences[0].route_index;
		if (occurrences[0].boarding > decrease)
		{
			Routes_values[route_ind][2] -= 5*decrease;		
			Routes_percentiles[route_ind] -= 5*decrease;	
			Routes_means[route_ind] -= 5*decrease;						
			Routes[route_ind][occurrences[0].route_position].boarding -= decrease;
			break;
		}
		else
		{	
			route_temp = convertRouteToArcFormat(Routes[route_ind]);
			arc_add.clear();
			arc_remove.clear();
			arc_remove.push_back(route_temp[occurrences[0].route_position - 1]);
			arc_remove.push_back(route_temp[occurrences[0].route_position]);
			arc_add.push_back({route_temp[occurrences[0].route_position - 1].start,route_temp[occurrences[0].route_position].end,0});
			Routes_values[route_ind][2] -= 15 + 5*occurrences[0].boarding; 			
			updateRouteParameters(route_temp, arc_remove, arc_add, Routes_values[route_ind], Routes_percentiles[route_ind], Routes_means[route_ind]);
			Routes[route_ind].erase(Routes[route_ind].begin() + occurrences[0].route_position);
			decrease -= occurrences[0].boarding;
			occurrences.erase(occurrences.begin()); 
		}
	}
}

// Identifies the best (before which) insertion point of stop in route
int bestInsertionPointOfStopInRoute(vector<stop_board_compulsory_info> route_to_insert, int cap, int load, stop_board_compulsory_info stop, vector<double> &route_values, double &new_per, double &new_mean)
{
	double best_mean, best_per = maxno; 	
	int best_insertion_point;
	route_temp = convertRouteToArcFormat(route_to_insert);

	if (load + stop.boarding <= cap)		route_values[2] += 15 + 5*stop.boarding;
	else									route_values[2] += 15 + 5*(cap - load);

	for (int z = 1; z < route_to_insert.size(); z++) {
		new_Route_values_temp = route_values;
		arc_add.clear();
		arc_remove.clear();
		arc_remove.push_back(route_temp[z - 1]);
		arc_add.push_back({route_temp[z - 1].start,stop.num,0});
		arc_add.push_back({stop.num,route_temp[z - 1].end,0});
		updateRouteParameters(route_temp, arc_remove, arc_add, new_Route_values_temp, new_per_temp, new_mean_temp);
		if (new_per_temp < best_per) {
			best_mean = new_mean_temp;
			best_per = new_per_temp;
			best_new_Route_values_temp = new_Route_values_temp;
			best_insertion_point = z;
		}
	}

	route_values = best_new_Route_values_temp;
	new_per = best_per;
	new_mean = best_mean;
	return best_insertion_point;
}

// Adds a stop (at best position in the route having lowest load) to Routes - used for elements in to_add having positive demands
void addStopToRoutes(vector<vector<stop_board_compulsory_info> > &Routes, stop_board_compulsory_info stop, vector<vector<double> > &Routes_values, vector<double> &Routes_percentiles, vector<double> &Routes_means)
{	
	vector<route_position_board_load> inclusions;
	int route_load, route_ind, insertion_pt;
	
	for (int i = 0; i < Routes.size(); i++)
	{
		route_load = routeLoad(Routes[i]);
		if (route_load < cap[i])	inclusions.push_back({i, 0, 0, route_load});
	}
	sort(inclusions.begin(), inclusions.end(), compareLoad);
	
	while (stop.boarding > 0)
	{
		route_ind = inclusions[0].route_index;
		insertion_pt = bestInsertionPointOfStopInRoute(Routes[route_ind], cap[route_ind], inclusions[0].load, stop, Routes_values[route_ind], Routes_percentiles[route_ind],Routes_means[route_ind]);	
		if (inclusions[0].load + stop.boarding <= cap[route_ind])
		{
			Routes[route_ind].insert(Routes[route_ind].begin() + insertion_pt, stop);	   				  	
			break;
		}
		else
		{ 	
			Routes[route_ind].insert(Routes[route_ind].begin() + insertion_pt, {stop.num, cap[route_ind] - inclusions[0].load, stop.compulsory});			
			stop.boarding -= (cap[route_ind] - inclusions[0].load);
			inclusions.erase(inclusions.begin());
		}
	}
}

// Adds some demand of a stop in Routes - used for elements in to_add having negative demands
void increaseDemandOfStopInRoutes(vector<vector<stop_board_compulsory_info> > &Routes, stop_board_compulsory_info stop, vector<vector<double> > &Routes_values, vector<double> &Routes_percentiles, vector<double> &Routes_means)
{
	int increase = -stop.boarding;		// by how much demand of stop must increase

	vector<route_position_board_load> occurrences;
	occurrencesOfStopInRoutes(Routes, stop, occurrences);
	sort(occurrences.begin(), occurrences.end(), compareLoad);
	int route_ind;

	while (increase > 0 && occurrences.size() > 0)
	{
		route_ind = occurrences[0].route_index;
		if (occurrences[0].load + increase <= cap[route_ind])
		{
			Routes_values[route_ind][2] += 5*increase;	
			Routes_percentiles[route_ind] += 5*increase;	
			Routes_means[route_ind] += 5*increase;		
			Routes[route_ind][occurrences[0].route_position].boarding += increase;
			increase = 0;
			break;
		}
		else
		{
			if (occurrences[0].load < cap[route_ind])
			{
				Routes_values[route_ind][2] += 5*(cap[route_ind] - occurrences[0].load);		
				Routes_percentiles[route_ind] += 5*(cap[route_ind] - occurrences[0].load);
				Routes_means[route_ind] += 5*(cap[route_ind] - occurrences[0].load);
				Routes[route_ind][occurrences[0].route_position].boarding += (cap[route_ind] - occurrences[0].load);
				increase -= (cap[route_ind] - occurrences[0].load);
				occurrences.erase(occurrences.begin());
			}
			else	break;
		}
	}

	if (increase > 0)
	{
		stop.boarding = increase;
		addStopToRoutes(Routes, stop, Routes_values, Routes_percentiles, Routes_means);
	}
}

// --------------------------------------------------------------------- MAIN FUNCTION ---------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{	
	// Setting parameters (file_name, MRT, max_walking_distance, available_capacities) and opening file
	string file = argv[1], file_name = file + ".bus", prefix;
	int it_limit = atoi(argv[2]);
	MRT = atoi(argv[3]);
	max_walking_distance = atof(argv[4]);
	alpha = atof(argv[5]), beta_par = atof(argv[6]);  
	lb_correlation = -pow(alpha,2)/(pow(alpha,2)+pow(beta_par,2));
	correlated_case = atoi(argv[7]); 
	// Creating vector of available capacities
	vector<int> available_capacities;
	for (int i = 0; i < (argc - 9) / 2; i++) 
		for (int j = 0; j < atoi(argv[10 + 2*i]); j++)   available_capacities.push_back(atoi(argv[9+2*i]));
	// Sorting vector of available capacities in non-increasing order
	sort(available_capacities.begin(), available_capacities.end(), greater<int>()); 

	ifstream input(file_name);

	// Getting total number of stops and total number of addresses
	string myString, line;
	int addresses;
	getline(input, line);
	stringstream ss(line);
	getline(ss, myString, ',');
	all_stops = stoi(myString) - 1;
	getline(ss, myString, ',');
	addresses = stoi(myString);

	// Filling coordinates, information about addresses, driving times/distances and walking distances
	coordinates.resize(all_stops + 1, vector<double>(2, 0));
	vector<address_info> vec_addresses(addresses);
	drive.resize(all_stops + 1, vector<double>(all_stops + 1, 0));
	dist.resize(all_stops + 1, vector<double>(all_stops + 1, 0));
	vector<vector<double> > walk(addresses, vector<double>(all_stops, maxno));
	fillData(all_stops, addresses, vec_addresses, walk, input);
	input.close();
	
	// Calculating total number of students and minimal number of vehicles required
	int min_vehicles = 0, total = 0, no_vehicles, feasible_no_vehicles;
	vector<int> initial_cap;
	for (int i = 0; i < addresses; i++)		students += vec_addresses[i].siblings;
	for (int i = 0; i < available_capacities.size(); i++) 
	{
		total += available_capacities[i];
		min_vehicles++;
		if (total >= students) break;
	}
	no_vehicles = min_vehicles; 
	feasible_no_vehicles = available_capacities.size();

	// Filling initial vector of capacities of used vehicles
	for (int i = 0; i < no_vehicles; i++)	initial_cap.push_back(available_capacities[i]);
	cap = initial_cap;

	// Data structures needed in loop
	vector<vector<int> > available_addresses(all_stops);
	vector<vector<stop_walk_info> > available_stops(addresses);
	vector<int> w(addresses, 0), w_best(addresses, 0);											// stop where students living in each address will walk to
	vector<stop_board_compulsory_info> used_stops_old, used_stops_new, used_stops_best; 		// stops that are actually used
	vector<vector<stop_board_compulsory_info> > Routes, best_Routes;
	vector<vector<double> > Routes_values, best_Routes_values;								    // required for route individual parameters
	vector<double> Routes_percentiles, best_Routes_percentiles, Routes_means, best_Routes_means;
	vector<stop_board_compulsory_info> to_add, to_remove; 										// required for solution alteration
	uniform_real_distribution<double> distribution(0.0, 1.0);
	double random_probability;

	// Timer and solution feasibility variables
	int counter, no_feasible_solutions, go, finish, route_load;  
	bool solution_infeasible;
	double route_percentile, total_per_time, best_total_per_time, overall_best_total_per_time = maxno, average_walking_distance, total_mean_time, best_total_mean_time;
	vector<vector<string> > best_solutions;
	vector<string> temp_string;
	ofstream myfile;
	
	double subset_alteration;
	// (i) -1 = Variant I = independent runs 
	// (ii) 1 = Variant II = random walk (altering previous subset)
	// (iii) 0 = Variant III = steepest descent (altering best subset found so far)
	// (iv) 0.5 = Variant IV = tradeoff between previous and best found so far
	if (correlated_case == 0)	file_results.open(file + "Ind_A=" + to_string(alpha).substr(0,4) + "_B=" + to_string(beta_par).substr(0,4) + " Results Summary.txt");
	else						file_results.open(file + "Corr_A=" + to_string(alpha).substr(0,4) + "_B=" + to_string(beta_par).substr(0,4) + " Results Summary.txt");
	
	// Fitting random forest
	if (correlated_case == 1)	randomForest(); 
	
	for (int seed = 0; seed <= 24; seed++)   
	{	 
			for (int variant = 3; variant <= 4; variant++)    
			{  	 
				no_vehicles = min_vehicles;   
				cap.clear();
				cap = initial_cap;   

				if (correlated_case == 0)	prefix = file + "Ind.Seed" + to_string(seed) + ".Variant" + to_string(variant) + "_A=" + to_string(alpha).substr(0,4) + "_B=" + to_string(beta_par).substr(0,4);
				else 						prefix = file + "Corr.Seed" + to_string(seed) + ".Variant" + to_string(variant) + "_A=" + to_string(alpha).substr(0,4) + "_B=" + to_string(beta_par).substr(0,4);
				file_results << endl << file << "\tSeed " << seed << " \tVariant " << variant; 
				if (correlated_case == 0)	cout << endl << "Ind " << file << "\tSeed " << seed << " \tVariant " << variant << " alpha " << alpha << " beta " << beta_par;     
				else						cout << endl << "Corr " << file << "\tSeed " << seed << " \tVariant " << variant << " alpha " << alpha << " beta " << beta_par; 

				if (variant == 1)		subset_alteration = -1;
				else if (variant == 2)	subset_alteration = 1;
				else if (variant == 3)	subset_alteration = 0;		
				else 					subset_alteration = 0.5;
				myfile.open(prefix + " - All Iterations.txt");			
				counter = 0;
				no_feasible_solutions = 0;
				best_total_per_time = maxno;
				used_stops_best.clear();
				best_Routes.clear();
				best_Routes_percentiles.clear();
				best_Routes_means.clear();
				best_Routes_values.clear();
				w_best.clear();

				while (no_feasible_solutions == 0)
				{
					mt19937 generator(seed); 
					correlations.clear();			// clearing unordered map 

					// Choosing first subset of stops
					subsetStopsSelection(addresses, all_stops, walk, available_stops, available_addresses, used_stops_new, generator, w, vec_addresses);
					
					// Nearest Neighbour constructive heuristic
					initialRoutesNN(used_stops_new, Routes, no_vehicles);
					getRouteValues(Routes, Routes_values);
					allRoutes95Percentiles_Means(Routes_values, Routes_percentiles, Routes_means);
		
					// Starting iterations of LS and Solution Alteration
					go = clock();
					while (counter < it_limit) 
					{
						counter++; 
						solution_infeasible = false;

						if (counter > 1)
						{
							if (subset_alteration != -1) {
		
								// Altering the subset of stops
								used_stops_old = used_stops_new;
								if (subset_alteration == 1 || used_stops_best.size() == 0)
									alterUsedStops(used_stops_new, generator, walk, all_stops, addresses, available_stops, available_addresses, w, vec_addresses);
								else if (subset_alteration == 0) {
									used_stops_new = used_stops_best;
									alterUsedStops(used_stops_new, generator, walk, all_stops, addresses, available_stops, available_addresses, w, vec_addresses);
								}
								else {
									// Generating a random probability
									random_probability = distribution(generator);
									if (random_probability <= subset_alteration)
										alterUsedStops(used_stops_new, generator, walk, all_stops, addresses, available_stops, available_addresses, w, vec_addresses);
									else {
										used_stops_new = used_stops_best;
										alterUsedStops(used_stops_new, generator, walk, all_stops, addresses, available_stops, available_addresses, w, vec_addresses);
									}
								}
								
								// Comparing the two subsets of stops and filling in to_add and to_remove
								compareTwoUsedStops(used_stops_old, used_stops_new, to_add, to_remove);
								
								// Removing stops in to_remove
								for (int i = 0; i < to_remove.size(); i++) 
									if (to_remove[i].boarding > 0) 
										removeStopFromRoutes(Routes, to_remove[i], Routes_values, Routes_percentiles, Routes_means);	
								
								// Decreasing demand of stops in to_remove
								for (int i = 0; i < to_remove.size(); i++) 
									if (to_remove[i].boarding < 0) 
										decreaseDemandOfStopInRoutes(Routes, to_remove[i], Routes_values, Routes_percentiles, Routes_means);
								
								// Increasing demand of stops in to_add
								for (int i = 0; i < to_add.size(); i++)
									if (to_add[i].boarding < 0)	
										increaseDemandOfStopInRoutes(Routes, to_add[i], Routes_values, Routes_percentiles, Routes_means);
								
								// Adding new stops in to_add
								for (int i = 0; i < to_add.size(); i++) 
									if (to_add[i].boarding > 0) 
										addStopToRoutes(Routes, to_add[i], Routes_values, Routes_percentiles, Routes_means);			
							}	
							else {
								// Creating a new subset of stops
								subsetStopsSelection(addresses, all_stops, walk, available_stops, available_addresses, used_stops_new, generator, w, vec_addresses);
								initialRoutesNN(used_stops_new, Routes, no_vehicles);
								getRouteValues(Routes, Routes_values);
								allRoutes95Percentiles_Means(Routes_values, Routes_percentiles, Routes_means);				
							}
						}
				
						// Local Search improvement heuristics		
						localSearch(Routes, Routes_values, Routes_percentiles, Routes_means);
											
						// Checking whether any route is considered as infeasible 
						total_per_time = 0;
						total_mean_time = 0;
						for (int j = 0; j < Routes.size(); j++) {
							if (Routes_percentiles[j] > MRT)
							{
								solution_infeasible = true;
								break;
							}
							total_per_time += Routes_percentiles[j]; 
							total_mean_time += Routes_means[j];
						} 
						
						if (solution_infeasible == false)	{
							no_feasible_solutions++;
			
							// Updating best total_per_time found so far
							if (total_per_time <= best_total_per_time)	{
								best_total_per_time = total_per_time;
								best_total_mean_time = total_mean_time;
								used_stops_best = used_stops_new;
								best_Routes = Routes;
								best_Routes_percentiles = Routes_percentiles;
								best_Routes_means = Routes_means;
								best_Routes_values = Routes_values;
								w_best = w;
							}

							// Calculating average walking distance
							average_walking_distance = 0;
							for (int s = 0; s < addresses; s++)
								average_walking_distance += (walk[s][w[s] - 1] * vec_addresses[s].siblings);
							average_walking_distance /= students;

							// Writing solution results in text file for plotting Pareto front
							myfile << total_per_time << "," << average_walking_distance << endl;
						}
					}  
					finish = clock();
					file_results << "\t" << (finish - go)/double(CLOCKS_PER_SEC) << "s LSTime";
					cout << "\t" << (finish - go)/double(CLOCKS_PER_SEC) << "s LSTime";
					
					if (no_feasible_solutions == 0)	{
						no_vehicles++;
						cap.push_back(available_capacities[no_vehicles-1]);
						counter = 0;
					}
					else {
						no_vehicles = best_Routes.size();  // needed due to LS possibly removing routes
						if (no_vehicles < feasible_no_vehicles) {
							feasible_no_vehicles = no_vehicles;
							overall_best_total_per_time = maxno;
						}
						myfile.close();
						file_results << "\t" << counter << " iterations\t" << no_vehicles << " vehicle(s)\t" << no_feasible_solutions << " feasible solns\t" << best_total_per_time/60 << " TPT\t" << best_total_mean_time/60 << " TMT\t";
						displayRoutes(best_Routes, best_Routes_values, best_Routes_percentiles, best_Routes_means);

						// Creating visualizer file for best solution
						myfile.open(prefix + " - Best Solution Visualizer.txt");
						myfile << "AvBXvyz_SchLwmGq1flVrsKu9_k9SHzLMTvVDovpaUMNhD-67VzoYP26CnrVhJUg\n" << best_Routes.size() << " 5.0 15.0\n";
						for (int i = 0; i < best_Routes.size(); i++) {
							myfile << best_Routes[i].size() - 2 << " ";
							for (int j = 1; j < best_Routes[i].size() - 1; j++)
									myfile << best_Routes[i][j].num << " " << best_Routes[i][j].boarding << " ";
							myfile << "\n";
						}
						for (int s = 0; s < addresses; s++)
							myfile << w_best[s] << "\n";
						myfile.close();
					} 
				}
				
				// Creating an array of best solutions 
				// Each row corresponds to a best solution with the first element being the seed and variant number
				// and the other elements being the loads of the routes 
				if ((no_vehicles == feasible_no_vehicles) && (best_total_per_time <= overall_best_total_per_time))
				{
					if (best_total_per_time < overall_best_total_per_time) {
						best_solutions.clear();
						overall_best_total_per_time = best_total_per_time;
					}
					temp_string.clear(); 
					temp_string.push_back(prefix);	
					for(int i = 0; i < best_Routes.size(); i++) 
					{
						route_load = routeLoad(best_Routes[i]);
						temp_string.push_back(to_string(route_load));
					}
					best_solutions.push_back(temp_string);
				}
			}		
	}
		
	file_results.close();
	return 0; 
}

