#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <set>
#include <vector>
#include <time.h>
#include <string>
#include <valarray>
#include <random>
#include <functional>
#include <cstdlib>

using namespace std;
int students = 0;
vector<int> cap;
double MRT, max_walking_distance, maxno = __DBL_MAX__;
vector<vector<double> > drive, dist;
ofstream file_results;

// ------------------------------------------------------------------------ STRUCTURES ----------------------------------------------------------------------------------------------

// stop_board_compulsory_info: (i) num; (ii) boarding; (iii) compulsory (true/false)
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

// route_position_load: (i) route index; (ii) position in route; (iii) load
struct route_position_load
{
	int route_index;
	int route_position;
	int load;
};

// address: (i) name; (ii) siblings
struct address_info
{
	string name;
	int siblings;
};

// ------------------------------------------------------------------- ADDRESS/DRIVE/WALK DATA -------------------------------------------------------------------------------------

// Fills address vector from the .bus file
void fillAddress(int all_stops, int addresses, vector<address_info> &vec_addresses, ifstream &input)
{
	string myString, line, tempString;
	int address_no, siblings_no;
	string address_name;
	for (int j = 0; j < 2 + all_stops; j++)
		input.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // ignoring lines before first a line in .bus file
	int line_no = 0;
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
}

// Fills the drive (in seconds) matrix and the distance (in km) matrix from the .bus file
void fillDriveDistance(int addresses, int all_stops, ifstream &input)
{
	string myString, line, tempString;
	int stop1, stop2;
	for (int j = 0; j < 2 + all_stops + addresses + (all_stops + 1); j++)
		input.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // ignoring lines before line of d,1,0 in .bus file
	int line_no = 0;
	while (line_no < ((all_stops + 1) * (all_stops + 1) - (all_stops + 1)) && getline(input, line))
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
}

// Fills the walk (in km) matrix from the .bus file
void fillWalk(vector<vector<double> > &walk, int addresses, int all_stops, ifstream &input)
{
	string myString, line, tempString;
	int address_no, stop_no;
	for (int i = 0; i < 2 + all_stops + addresses + (all_stops + 1) * (all_stops + 1); i++)
		input.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // ignoring lines before first w line in .bus file
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

// --------------------------------------------------------------------- ROUTES FUNCTIONS--------------------------------------------------------------------------------------------

// Returns position of stop in vec (if found) or -1 otherwise
int findStopInVec(vector<stop_board_compulsory_info> vec, int stop)
{
	for (int i = 0; i < vec.size(); i++)
	{
		if (vec[i].num == stop)
			return i;
	}
	return -1;
}

// Returns the load of route
int routeLoad(vector<stop_board_compulsory_info> route)
{
	int load = 0;
	for (int z = 1; z < route.size() - 1; z++)
	{
		load += route[z].boarding;
	}
	return load;
}

// Calculates the journey times of the routes present in the rows of Routes
void journeyTime(vector<double> &journey_time, vector<vector<stop_board_compulsory_info> > Routes)
{
	journey_time.clear();
	journey_time.resize(Routes.size(), 0);
	for (int j = 0; j < journey_time.size(); j++)
	{
		for (int k = 1; k < Routes[j].size() - 1; k++)
		{
			journey_time[j] += drive[Routes[j][k].num][Routes[j][k + 1].num] + (15 + 5 * Routes[j][k].boarding);
		}
	}
}

// Displays each route's stops, number of students boarding at each stop, total distance
void displayRoutes(vector<vector<stop_board_compulsory_info> > Routes) 
{
	double sum_TT, sum_ST, sum_distance; 
	int load;
	for (int j = 0; j < Routes.size(); j++)
	{
		sum_TT = 0; 
		sum_ST = 0;
		sum_distance = 0;
		file_results << endl << "Route " << j + 1 << " stops: ";
		for (int l = 0; l < Routes[j].size(); l++)	file_results << Routes[j][l].num << " ";
		for (int k = 1; k < Routes[j].size() - 1; k++)
		{
			sum_TT += drive[Routes[j][k].num][Routes[j][k + 1].num];
			sum_ST += (15 + 5 * Routes[j][k].boarding);
			sum_distance += dist[Routes[j][k].num][Routes[j][k + 1].num];
		}
		load = routeLoad(Routes[j]);
		file_results << "with DIST,TT,ST,MJT,95P_Ind = " << sum_distance << "," << sum_TT << "," << sum_ST << "," << sum_TT+sum_ST << endl;
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
		{
			if (walk[subset_addresses[s] - 1][subset_stops[i].num - 1] <= max_walking_distance)
			{
				available_addresses[i].push_back(subset_addresses[s]);
			}
		}
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
void coverAddresses(vector<int> &subset_addresses, vector<stop_board_compulsory_info> &used_stops, vector<vector<double> > walk, vector<stop_board_compulsory_info> &subset_stops, vector<vector<int> > &available_addresses, mt19937 &generator, bool print)
{
	int no_max_rows;  // number of rows in available_addresses having maximum size
	int random_index; // index of random stop that will be added in each iteration

	while (subset_addresses.size() > 0)
	{
		no_max_rows = 1;
		identifyAvailableAddresses(walk, subset_stops, subset_addresses, available_addresses);
		for (int k = 1; k < available_addresses.size(); k++)
		{
			if (available_addresses[k].size() == available_addresses[0].size())
			{
				no_max_rows++;
			}
			else
				break;
		}

		// Generating a uniformly distributed random number between 0 and (no_max_rows-1)
		uniform_int_distribution<int> distribution(0, no_max_rows - 1);
		random_index = distribution(generator);

		// Adding the randomly chosen stop to used_stops
		stop_board_compulsory_info stop = {available_addresses[random_index][0], 0, false};
		used_stops.push_back(stop);

		// Removing addresses from subset_addresses covered by the chosen stop
		for (int j = 1; j < available_addresses[random_index].size(); j++)
		{
			subset_addresses.erase(remove(subset_addresses.begin(), subset_addresses.end(), available_addresses[random_index][j]), subset_addresses.end());
		}
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
		if (used_stops[j].boarding == 0)
			used_stops.erase(used_stops.begin() + j);
		else
			j++;
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
	for (int s = 1; s <= addresses; s++)
	{
		subset_addresses.push_back(s);
	}

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
		{
			subset_addresses.erase(remove(subset_addresses.begin(), subset_addresses.end(), available_addresses[k][j]), subset_addresses.end());
		}
	}
	subset_stops = allStopsWithoutTwoVectorsOfStops(all_stops, used_stops, extra_vector);

	// Looping until all addresses are covered at least once
	coverAddresses(subset_addresses, used_stops, walk, subset_stops, available_addresses, generator, false);
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
	{
		Routes[i].push_back(zero_stop);
	} // initially all routes have just 0

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

// Returns the change in total cost when journey_time1 is decreased by dec1, journey_time2 is decreased by dec2
double costImprovement(double journey_time1, double dec1, double journey_time2, double dec2)
{
	double cost_difference = 0.0;

	if (journey_time1 <= MRT)		cost_difference += journey_time1;
	else							cost_difference += (MRT * (2 + journey_time1 - MRT));
	journey_time1 -= dec1;
	if (journey_time1 <= MRT)		cost_difference -= journey_time1;
	else							cost_difference -= (MRT * (2 + journey_time1 - MRT));

	if (journey_time2 != 0) {
		if (journey_time2 <= MRT)		cost_difference += journey_time2;
		else							cost_difference += (MRT * (2 + journey_time2 - MRT));
		journey_time2 -= dec2;
		if (journey_time2 <= MRT)		cost_difference -= journey_time2;
		else							cost_difference -= (MRT * (2 + journey_time2 - MRT));
	}
	return cost_difference;
}

// Returns discrepancy between longest and shortest routes in journey_time
double routeBalancing(vector<double> journey_time, int m, double improv1, int n, double improv2)
{
	journey_time[m] -= improv1;
	journey_time[n] -= improv2;
	return (*max_element(journey_time.begin(), journey_time.end()) - *min_element(journey_time.begin(), journey_time.end()));
}

// Returns cost improvement (can be negative) when swapping stops in positions l and k in route
double checkSwapIntraRoute(vector<stop_board_compulsory_info> route, double route_time, int l, int k, double &improv1)
{
	double swap_time_improvement;

	if (k > l + 1)
		swap_time_improvement = drive[route[l - 1].num][route[l].num] + drive[route[l].num][route[l + 1].num] + drive[route[k - 1].num][route[k].num] +
								drive[route[k].num][route[k + 1].num] - (drive[route[l - 1].num][route[k].num] + drive[route[k].num][route[l + 1].num] + drive[route[k - 1].num][route[l].num] + drive[route[l].num][route[k + 1].num]);
	else
		swap_time_improvement = drive[route[l - 1].num][route[l].num] + drive[route[l].num][route[k].num] + drive[route[k].num][route[k + 1].num] - (drive[route[l - 1].num][route[k].num] + drive[route[k].num][route[l].num] + drive[route[l].num][route[k + 1].num]);
	improv1 = swap_time_improvement;

	return costImprovement(route_time, swap_time_improvement, 0, 0);
}

// Returns cost improvement (can be negative) when inverting subroute in positions l ... k in route (2-opt)
double check2OptIntraRoute(vector<stop_board_compulsory_info> route, double route_time, int l, int k, double &improv1)
{
	double opt_time_improvement = drive[route[l - 1].num][route[l].num] + drive[route[k].num][route[k + 1].num] - (drive[route[l - 1].num][route[k].num] + drive[route[l].num][route[k + 1].num]);

	for (int count = 0; count < (k - l); count++)
	{
		opt_time_improvement += drive[route[l + count].num][route[l + count + 1].num];
		opt_time_improvement -= drive[route[k - count].num][route[k - count - 1].num];
	}
	improv1 = opt_time_improvement;

	return costImprovement(route_time, opt_time_improvement, 0, 0);
}

// Returns cost improvement (can be negative) when moving, and possibly inverting, subroute in positions l ... k in route before position j (extended-or-opt)
double checkExtendedOrOptIntraRoute(vector<stop_board_compulsory_info> route, double route_time, int j, int l, int k, bool &invert, double &improv1)
{
	double extendednoinv_time_improvement, extendedinv_time_improvement, extendednoinv_cost_improvement, extendedinv_cost_improvement;

	extendednoinv_time_improvement = drive[route[l - 1].num][route[l].num] + drive[route[k].num][route[k + 1].num] + drive[route[j - 1].num][route[j].num] - drive[route[j - 1].num][route[l].num] - drive[route[l - 1].num][route[k + 1].num] - drive[route[k].num][route[j].num];
	extendedinv_time_improvement = drive[route[l - 1].num][route[l].num] + drive[route[k].num][route[k + 1].num] + drive[route[j - 1].num][route[j].num] - drive[route[j - 1].num][route[k].num] - drive[route[l - 1].num][route[k + 1].num] - drive[route[l].num][route[j].num];
	for (int count = 0; count < (k - l); count++)
	{
		extendedinv_time_improvement += drive[route[l + count].num][route[l + count + 1].num];
		extendedinv_time_improvement -= drive[route[k - count].num][route[k - count - 1].num];
	}

	extendednoinv_cost_improvement = costImprovement(route_time, extendednoinv_time_improvement, 0, 0);
	extendedinv_cost_improvement = costImprovement(route_time, extendedinv_time_improvement, 0, 0);
	if (extendednoinv_cost_improvement < extendedinv_cost_improvement)
	{
		invert = true;
		improv1 = extendedinv_time_improvement;
		return extendedinv_cost_improvement;
	}
	else
	{
		invert = false;
		improv1 = extendednoinv_time_improvement;
		return extendednoinv_cost_improvement;
	}
}

// Returns decrease when duplicate stops are removed from route2 when relocating subroute in positions l ... k from route1 before position j in route 2 (or-exchange duplicates)
double checkRemoveDuplicatesOrExchange(vector<stop_board_compulsory_info> route1, vector<stop_board_compulsory_info> route2, int j, int l, int k, bool invert)
{
	double decrease = 0;
	int last_count_not_removed = -1; // needed to keep record of this to see which stop is next to the current one being removed
	int position;
	for (int count = 0; count <= (k - l); count++)
	{
		position = findStopInVec(route2, route1[l + count].num);
		if (position == -1)
			last_count_not_removed = count;
		else
		{ // we need to remove the duplicate stop
			decrease += 15;

			if (k - l == 0)
				// one element relocated (in position l) and it is already in route2
				decrease += drive[route2[j - 1].num][route1[l].num] + drive[route1[l].num][route2[j].num] - drive[route2[j - 1].num][route2[j].num];

			else if (count == 0)
			{
				// first element (in position l) relocated is already in route2
				if (!invert)
					decrease += drive[route2[j - 1].num][route1[l].num] + drive[route1[l].num][route1[l + 1].num] - drive[route2[j - 1].num][route1[l + 1].num];
				else
					decrease += drive[route1[l + 1].num][route1[l].num] + drive[route1[l].num][route2[j].num] - drive[route1[l + 1].num][route2[j].num];
			}

			else if (count == (k - l))
			{
				// last element (in position k) relocated is already in route2
				if (last_count_not_removed == -1) // all elements up to (and including) stop at position k are duplicates
					decrease += drive[route2[j - 1].num][route1[k].num] + drive[route1[k].num][route2[j].num] - drive[route2[j - 1].num][route2[j].num];
				else
				{
					if (!invert)
						decrease += drive[route1[l + last_count_not_removed].num][route1[k].num] + drive[route1[k].num][route2[j].num] - drive[route1[l + last_count_not_removed].num][route2[j].num];
					else
						decrease += drive[route2[j - 1].num][route1[k].num] + drive[route1[k].num][route1[l + last_count_not_removed].num] - drive[route2[j - 1].num][route1[l + last_count_not_removed].num];
				}
			}

			else
			{
				// element in position (l+count) where 0 < count < (k-l) relocated is already in route2
				if (last_count_not_removed == -1) // all elements up to (and including) stop at position (l+count) are duplicates
				{
					if (!invert)
						decrease += drive[route2[j - 1].num][route1[l + count].num] + drive[route1[l + count].num][route1[l + count + 1].num] - drive[route2[j - 1].num][route1[l + count + 1].num];
					else
						decrease += drive[route1[l + count + 1].num][route1[l + count].num] + drive[route1[l + count].num][route2[j].num] - drive[route1[l + count + 1].num][route2[j].num];
				}
				else
				{
					if (!invert)
						decrease += drive[route1[l + last_count_not_removed].num][route1[l + count].num] + drive[route1[l + count].num][route1[l + count + 1].num] - drive[route1[l + last_count_not_removed].num][route1[l + count + 1].num];
					else
						decrease += drive[route1[l + count + 1].num][route1[l + count].num] + drive[route1[l + count].num][route1[l + last_count_not_removed].num] - drive[route1[l + count + 1].num][route1[l + last_count_not_removed].num];
				}
			}
		}
	}
	return decrease;
}

// Returns decrease when duplicate stops are removed from route2 when swapping subroute in positions l ... k from route1 with subroute in positions j ... i in route 2 (cross-exchange duplicates)
double checkRemoveDuplicatesCrossExchange(vector<stop_board_compulsory_info> route1, vector<stop_board_compulsory_info> route2, int l, int k, int j, int i, bool invert)
{
	double decrease = 0;
	int last_count_not_removed = -1; // needed to keep record of this to see which stop is next to the current one being removed
	int position;
	for (int count = 0; count <= (k - l); count++)
	{
		position = findStopInVec(route2, route1[l + count].num);
		if (position == -1 || (position >= j && position <= i))
			last_count_not_removed = count;
		else
		{ // we need to remove the duplicate stop
			decrease += 15;

			if (k - l == 0)
				// one element relocated (in position l) and it is already in route2
				decrease += drive[route2[j - 1].num][route1[l].num] + drive[route1[l].num][route2[i + 1].num] - drive[route2[j - 1].num][route2[i + 1].num];

			else if (count == 0)
			{
				// first element (in position l) relocated is already in route2
				if (!invert)
					decrease += drive[route2[j - 1].num][route1[l].num] + drive[route1[l].num][route1[l + 1].num] - drive[route2[j - 1].num][route1[l + 1].num];
				else
					decrease += drive[route1[l + 1].num][route1[l].num] + drive[route1[l].num][route2[i + 1].num] - drive[route1[l + 1].num][route2[i + 1].num];
			}

			else if (count == (k - l))
			{
				// last element (in position k) relocated is already in route2
				if (last_count_not_removed == -1) // all elements up to (and including) stop at position k are duplicates
					decrease += drive[route2[j - 1].num][route1[k].num] + drive[route1[k].num][route2[i + 1].num] - drive[route2[j - 1].num][route2[i + 1].num];
				else
				{
					if (!invert)
						decrease += drive[route1[l + last_count_not_removed].num][route1[k].num] + drive[route1[k].num][route2[i + 1].num] - drive[route1[l + last_count_not_removed].num][route2[i + 1].num];
					else
						decrease += drive[route2[j - 1].num][route1[k].num] + drive[route1[k].num][route1[l + last_count_not_removed].num] - drive[route2[j - 1].num][route1[l + last_count_not_removed].num];
				}
			}

			else
			{
				// element in position (l+count) where 0 < count < (k-l) relocated is already in route2
				if (last_count_not_removed == -1) // all elements up to (and including) stop at position (l+count) are duplicates
				{
					if (!invert)
						decrease += drive[route2[j - 1].num][route1[l + count].num] + drive[route1[l + count].num][route1[l + count + 1].num] - drive[route2[j - 1].num][route1[l + count + 1].num];
					else
						decrease += drive[route1[l + count + 1].num][route1[l + count].num] + drive[route1[l + count].num][route2[i + 1].num] - drive[route1[l + count + 1].num][route2[i + 1].num];
				}
				else
				{
					if (!invert)
						decrease += drive[route1[l + last_count_not_removed].num][route1[l + count].num] + drive[route1[l + count].num][route1[l + count + 1].num] - drive[route1[l + last_count_not_removed].num][route1[l + count + 1].num];
					else
						decrease += drive[route1[l + count + 1].num][route1[l + count].num] + drive[route1[l + count].num][route1[l + last_count_not_removed].num] - drive[route1[l + count + 1].num][route1[l + last_count_not_removed].num];
				}
			}
		}
	}
	return decrease;
}

// Returns cost improvement (can be negative) when relocating subroute in positions l ... k from route1 before position j in route 2 (or-exchange)
double checkOrExchangeInterRoute(vector<stop_board_compulsory_info> route1, double route_time1, vector<stop_board_compulsory_info> route2, double route_time2, int j, int l, int k, bool &invert, double &improv1, double &improv2)
{
	double orexchange_time_dec, orexchangenoinv_time_inc, orexchangeinv_time_inc, orexchangenoinv_cost_improvement, orexchangeinv_cost_improvement;

	orexchange_time_dec = drive[route1[l - 1].num][route1[l].num] + drive[route1[k].num][route1[k + 1].num] - drive[route1[l - 1].num][route1[k + 1].num];
	orexchangenoinv_time_inc = drive[route2[j - 1].num][route1[l].num] + drive[route1[k].num][route2[j].num] - drive[route2[j - 1].num][route2[j].num];
	orexchangeinv_time_inc = drive[route2[j - 1].num][route1[k].num] + drive[route1[l].num][route2[j].num] - drive[route2[j - 1].num][route2[j].num];
	for (int count = 0; count <= (k - l); count++)
	{
		orexchange_time_dec += 15 + 5 * route1[l + count].boarding;
		orexchangenoinv_time_inc += 15 + 5 * route1[l + count].boarding;
		orexchangeinv_time_inc += 15 + 5 * route1[l + count].boarding;
		if (count < (k - l))
		{
			orexchange_time_dec += drive[route1[l + count].num][route1[l + count + 1].num];
			orexchangenoinv_time_inc += drive[route1[l + count].num][route1[l + count + 1].num];
			orexchangeinv_time_inc += drive[route1[k - count].num][route1[k - count - 1].num];
		}
	}

	// removing duplicates (if any) in route 2
	orexchangenoinv_time_inc -= checkRemoveDuplicatesOrExchange(route1, route2, j, l, k, false);
	orexchangeinv_time_inc -= checkRemoveDuplicatesOrExchange(route1, route2, j, l, k, true);

	orexchangenoinv_cost_improvement = costImprovement(route_time1, orexchange_time_dec, route_time2, -orexchangenoinv_time_inc);
	orexchangeinv_cost_improvement = costImprovement(route_time1, orexchange_time_dec, route_time2, -orexchangeinv_time_inc);

	if (orexchangenoinv_cost_improvement < orexchangeinv_cost_improvement)
	{
		invert = true;
		improv1 = orexchange_time_dec;
		improv2 = -orexchangeinv_time_inc;
		return orexchangeinv_cost_improvement;
	}
	else
	{
		invert = false;
		improv1 = orexchange_time_dec;
		improv2 = -orexchangenoinv_time_inc;
		return orexchangenoinv_cost_improvement;
	}
}

// Returns cost improvement (can be negative) when swapping subroute in positions l ... k from route1 with subroute in positions j ... i in route 2 (cross-exchange)
double checkCrossExchangeInterRoute(vector<stop_board_compulsory_info> route1, double route_time1, vector<stop_board_compulsory_info> route2, double route_time2, int l, int k, int j, int i, bool &invert1, bool &invert2, double &improv1, double &improv2)
{
	double crossexchangenoinv_time_dec1, crossexchangenoinv_time_dec2, crossexchangeinv_time_dec1, crossexchangeinv_time_dec2, crossexchangenoinv1noinv2_cost_improvement,
		crossexchangeinv1noinv2_cost_improvement, crossexchangenoinv1inv2_cost_improvement, crossexchangeinv1inv2_cost_improvement, crossexchange_best_cost_improvement;

	crossexchangenoinv_time_dec1 = drive[route1[l - 1].num][route1[l].num] + drive[route1[k].num][route1[k + 1].num] - drive[route1[l - 1].num][route2[j].num] - drive[route2[i].num][route1[k + 1].num];
	crossexchangenoinv_time_dec2 = drive[route2[j - 1].num][route2[j].num] + drive[route2[i].num][route2[i + 1].num] - drive[route2[j - 1].num][route1[l].num] - drive[route1[k].num][route2[i + 1].num];
	crossexchangeinv_time_dec1 = drive[route1[l - 1].num][route1[l].num] + drive[route1[k].num][route1[k + 1].num] - drive[route1[l - 1].num][route2[i].num] - drive[route2[j].num][route1[k + 1].num];
	crossexchangeinv_time_dec2 = drive[route2[j - 1].num][route2[j].num] + drive[route2[i].num][route2[i + 1].num] - drive[route2[j - 1].num][route1[k].num] - drive[route1[l].num][route2[i + 1].num];
	for (int count1 = 0; count1 <= (k - l); count1++)
	{
		crossexchangenoinv_time_dec1 += 15 + 5 * route1[l + count1].boarding;
		crossexchangenoinv_time_dec2 -= 15 + 5 * route1[l + count1].boarding;
		crossexchangeinv_time_dec1 += 15 + 5 * route1[l + count1].boarding;
		crossexchangeinv_time_dec2 -= 15 + 5 * route1[l + count1].boarding;
		if (count1 < (k - l))
		{
			crossexchangenoinv_time_dec1 += drive[route1[l + count1].num][route1[l + count1 + 1].num];
			crossexchangenoinv_time_dec2 -= drive[route1[l + count1].num][route1[l + count1 + 1].num];
			crossexchangeinv_time_dec1 += drive[route1[l + count1].num][route1[l + count1 + 1].num];
			crossexchangeinv_time_dec2 -= drive[route1[k - count1].num][route1[k - count1 - 1].num];
		}
	}
	for (int count2 = 0; count2 <= (i - j); count2++)
	{
		crossexchangenoinv_time_dec2 += 15 + 5 * route2[j + count2].boarding;
		crossexchangenoinv_time_dec1 -= 15 + 5 * route2[j + count2].boarding;
		crossexchangeinv_time_dec2 += 15 + 5 * route2[j + count2].boarding;
		crossexchangeinv_time_dec1 -= 15 + 5 * route2[j + count2].boarding;
		if (count2 < (i - j))
		{
			crossexchangenoinv_time_dec2 += drive[route2[j + count2].num][route2[j + count2 + 1].num];
			crossexchangenoinv_time_dec1 -= drive[route2[j + count2].num][route2[j + count2 + 1].num];
			crossexchangeinv_time_dec2 += drive[route2[j + count2].num][route2[j + count2 + 1].num];
			crossexchangeinv_time_dec1 -= drive[route2[i - count2].num][route2[i - count2 - 1].num];
		}
	}

	// removing duplicates (if any) in route 2
	crossexchangenoinv_time_dec2 += checkRemoveDuplicatesCrossExchange(route1, route2, l, k, j, i, false);
	crossexchangeinv_time_dec2 += checkRemoveDuplicatesCrossExchange(route1, route2, l, k, j, i, true);
	// removing duplicates (if any) in route 1
	crossexchangenoinv_time_dec1 += checkRemoveDuplicatesCrossExchange(route2, route1, j, i, l, k, false);
	crossexchangeinv_time_dec1 += checkRemoveDuplicatesCrossExchange(route2, route1, j, i, l, k, true);

	crossexchangenoinv1noinv2_cost_improvement = costImprovement(route_time1, crossexchangenoinv_time_dec1, route_time2, crossexchangenoinv_time_dec2);
	crossexchangeinv1noinv2_cost_improvement = costImprovement(route_time1, crossexchangeinv_time_dec1, route_time2, crossexchangenoinv_time_dec2);
	crossexchangenoinv1inv2_cost_improvement = costImprovement(route_time1, crossexchangenoinv_time_dec1, route_time2, crossexchangeinv_time_dec2);
	crossexchangeinv1inv2_cost_improvement = costImprovement(route_time1, crossexchangeinv_time_dec1, route_time2, crossexchangeinv_time_dec2);
	crossexchange_best_cost_improvement = max(max(crossexchangenoinv1noinv2_cost_improvement, crossexchangeinv1noinv2_cost_improvement),
											  max(crossexchangenoinv1inv2_cost_improvement, crossexchangeinv1inv2_cost_improvement));
	if (crossexchange_best_cost_improvement == crossexchangenoinv1noinv2_cost_improvement)
	{
		invert1 = false;
		invert2 = false;
		improv1 = crossexchangenoinv_time_dec1;
		improv2 = crossexchangenoinv_time_dec2;
	}
	else if (crossexchange_best_cost_improvement == crossexchangeinv1noinv2_cost_improvement)
	{
		invert1 = true;
		invert2 = false;
		improv1 = crossexchangeinv_time_dec1;
		improv2 = crossexchangenoinv_time_dec2;
	}
	else if (crossexchange_best_cost_improvement == crossexchangenoinv1inv2_cost_improvement)
	{
		invert1 = false;
		invert2 = true;
		improv1 = crossexchangenoinv_time_dec1;
		improv2 = crossexchangeinv_time_dec2;
	}
	else
	{
		invert1 = true;
		invert2 = true;
		improv1 = crossexchangeinv_time_dec1;
		improv2 = crossexchangeinv_time_dec2;
	}
	return crossexchange_best_cost_improvement;
}

// Returns cost improvement (can be negative) when transferring students from stop in position x in route1 (with journey time exceeding MRT) in front of "position" in route2 (if stop is not already in route2)
double checkCreateMultistop(int cap2, vector<stop_board_compulsory_info> route1, double route_time1, vector<stop_board_compulsory_info> route2, double route_time2, int route_load2, int x, bool &present, int &position, int &transfer, double &improv1, double &improv2)
{
	int multistop_best_index = 0, pos = findStopInVec(route2, route1[x].num);
	transfer = min(route1[x].boarding - 1, cap2 - route_load2);
	double multistop_time_best_inc2 = maxno, multistop_time_inc2, multistop_time_dec1 = 5 * transfer;

	if (pos == -1)
	{
		// stop is not already in route2 so identify the best insertion point (before stop at "multistop_best_index")
		present = false;
		for (int z = 1; z < route2.size(); z++)
		{
			multistop_time_inc2 = drive[route2[z - 1].num][route1[x].num] + drive[route1[x].num][route2[z].num] - drive[route2[z - 1].num][route2[z].num];
			if (multistop_time_inc2 < multistop_time_best_inc2)
			{
				multistop_time_best_inc2 = multistop_time_inc2;
				multistop_best_index = z;
			}
		}
		multistop_time_best_inc2 += (15 + 5 * transfer);
	}
	else
	{
		present = true;
		multistop_time_best_inc2 = 5 * transfer;
		multistop_best_index = pos;
	}

	position = multistop_best_index;
	improv1 = multistop_time_dec1;
	improv2 = -multistop_time_best_inc2;
	return costImprovement(route_time1, multistop_time_dec1, route_time2, -multistop_time_best_inc2);
}

// ------------------------------------------------------------ LOCAL SEARCH MOVE EXECUTIONS ---------------------------------------------------------------------------------------

// Removes duplicate stops from subroute (which are already in route) before relocating subroute before position j in route (or-exchange duplicates)
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

// Removes duplicate stops from sub_route (which are already in route) before swapping subroute in position pos1 ... pos2 in route with sub_route (cross-exchange duplicates)
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

void localSearch(vector<double> &journey_time, vector<vector<stop_board_compulsory_info> > &Routes)
{
	double route_time1, route_time2, bestCostImprovement = 1, newCostImprovement = 0;
	bool invert_subroute_goinginto1, invert_subroute_goinginto2, best_invert_subroute_goinginto1, best_invert_subroute_goinginto2, present, best_present;
	int route_load1, route_load2, demand_inc1, demand_inc2, best_index_route1, best_index_route2, best_l, best_k, best_j, best_i, best_x, position, best_position, transfer, best_transfer;
	int best_move;							  // 1 = swap; 2 = 2-opt; 3 = extended-or-opt; 4 = or-exchange; 5 = cross-exchange; 6 = multistop
	double improv1, improv2, bestDiscrepancy; // required for secondary objective (route balancing / total time balancing)
	vector<vector<stop_board_compulsory_info> > Routes_copy = Routes;
	vector<stop_board_compulsory_info> subroute1, subroute2;

	while (bestCostImprovement > 0)
	{
		bestCostImprovement = 0;
		bestDiscrepancy = *max_element(journey_time.begin(), journey_time.end()) - *min_element(journey_time.begin(), journey_time.end());

		// INTRA-ROUTE OPERATORS
		for (int m = 0; m < Routes.size(); m++)
		{
			route_time1 = journey_time[m];
			route_load1 = routeLoad(Routes[m]);
			for (int l = 1; l < Routes[m].size() - 1; l++)
			{
				for (int k = l; k < Routes[m].size() - 1; k++)
				{
					// swap
					if (k >= (l + 1) && l < (Routes[m].size() - 2))
					{
						newCostImprovement = checkSwapIntraRoute(Routes[m], route_time1, l, k, improv1);
						if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && (routeBalancing(journey_time, m, improv1, 0, 0) < bestDiscrepancy)))
						{
							bestDiscrepancy = routeBalancing(journey_time, m, improv1, 0, 0);
							bestCostImprovement = newCostImprovement;
							best_index_route1 = m;
							best_l = l;
							best_k = k;
							best_move = 1;
						}
					}

					// 2-opt
					if (k >= (l + 3) && l < (Routes[m].size() - 2))
					{
						newCostImprovement = check2OptIntraRoute(Routes[m], route_time1, l, k, improv1);
						if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && (routeBalancing(journey_time, m, improv1, 0, 0) < bestDiscrepancy)))
						{
							bestDiscrepancy = routeBalancing(journey_time, m, improv1, 0, 0);
							bestCostImprovement = newCostImprovement;
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
							newCostImprovement = checkExtendedOrOptIntraRoute(Routes[m], route_time1, j, l, k, invert_subroute_goinginto1, improv1);
							if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && (routeBalancing(journey_time, m, improv1, 0, 0) < bestDiscrepancy)))
							{
								bestDiscrepancy = routeBalancing(journey_time, m, improv1, 0, 0);
								bestCostImprovement = newCostImprovement;
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
			route_time1 = journey_time[m];
			route_load1 = routeLoad(Routes[m]);
			for (int n = 0; n < Routes.size(); n++)
			{
				if (n != m)
				{
					route_time2 = journey_time[n];
					route_load2 = routeLoad(Routes[n]);

					for (int l = 1; l < Routes[m].size() - 1; l++)
					{
						for (int k = l; k < Routes[m].size() - 1; k++)
						{
							demand_inc2 = 0;
							for (int count = 0; count <= (k - l); count++)
							{
								demand_inc2 += Routes[m][l + count].boarding;
							}

							// or-exchange
							if (route_load2 + demand_inc2 <= cap[n])
							{
								for (int j = 1; j < Routes[n].size(); j++)
								{
									newCostImprovement = checkOrExchangeInterRoute(Routes[m], route_time1, Routes[n], route_time2, j, l, k, invert_subroute_goinginto2, improv1, improv2);
									if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && (routeBalancing(journey_time, m, improv1, n, improv2) < bestDiscrepancy)))
									{
										bestDiscrepancy = routeBalancing(journey_time, m, improv1, n, improv2);
										bestCostImprovement = newCostImprovement;
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
										for (int count = 0; count <= (i - j); count++)
										{
											demand_inc1 += Routes[n][j + count].boarding;
										}

										if ((route_load1 - demand_inc2 + demand_inc1 <= cap[m]) & (route_load2 - demand_inc1 + demand_inc2 <= cap[n]) & !((l == 1) & (j == 1) & (k == Routes[m].size() - 2) & (i == Routes[n].size() - 2)))
										{
											newCostImprovement = checkCrossExchangeInterRoute(Routes[m], route_time1, Routes[n], route_time2, l, k, j, i, invert_subroute_goinginto1, invert_subroute_goinginto2, improv1, improv2);
											if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && (routeBalancing(journey_time, m, improv1, n, improv2) < bestDiscrepancy)))
											{
												bestDiscrepancy = routeBalancing(journey_time, m, improv1, n, improv2);
												bestCostImprovement = newCostImprovement;
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
			if (journey_time[m] > MRT)
			{ // moving only if Route m exceeds MRT
				for (int x = 1; x < Routes[m].size() - 1; x++)
				{
					if (Routes[m][x].boarding >= 2)
					{
						for (int n = 0; n < Routes.size(); n++)
						{
							route_load2 = routeLoad(Routes[n]);
							if (n != m && route_load2 < cap[n])
							{
								route_time2 = journey_time[n];
								newCostImprovement = checkCreateMultistop(cap[n], Routes[m], route_time1, Routes[n], route_time2, route_load2, x, present, position, transfer, improv1, improv2);
								if ((newCostImprovement > bestCostImprovement) || ((newCostImprovement == bestCostImprovement) && (routeBalancing(journey_time, m, improv1, n, improv2) < bestDiscrepancy)))
								{
									bestDiscrepancy = routeBalancing(journey_time, m, improv1, n, improv2);
									bestCostImprovement = newCostImprovement;
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
			if (best_move == 1)
				swap(Routes[best_index_route1][best_l], Routes[best_index_route1][best_k]);

			else if (best_move == 2)
				reverse(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + best_k + 1);

			else if (best_move == 3)
			{
				subroute1.clear();
				subroute1.assign(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + (best_k + 1));
				if (best_invert_subroute_goinginto1 == true)
					reverse(subroute1.begin(), subroute1.end());
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
				removeDuplicatesOrExchange(Routes[best_index_route2], subroute1); // removing duplicate vertices from subroute1 (which are already in route2)
				if (best_invert_subroute_goinginto2 == true)
					reverse(subroute1.begin(), subroute1.end());
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
				if (best_invert_subroute_goinginto2 == true)
					reverse(subroute1.begin(), subroute1.end());
				if (best_invert_subroute_goinginto1 == true)
					reverse(subroute2.begin(), subroute2.end());
				Routes[best_index_route1].erase(Routes[best_index_route1].begin() + best_l, Routes[best_index_route1].begin() + (best_k + 1));
				Routes[best_index_route1].insert(Routes[best_index_route1].begin() + best_l, subroute2.begin(), subroute2.end());
				Routes[best_index_route2].erase(Routes[best_index_route2].begin() + best_j, Routes[best_index_route2].begin() + (best_i + 1));
				Routes[best_index_route2].insert(Routes[best_index_route2].begin() + best_j, subroute1.begin(), subroute1.end());
			}

			else if (best_move == 6)
			{
				Routes[best_index_route1][best_x].boarding -= best_transfer;
				if (best_present == false)
				{ // stop is not already in route2
					stop_board_compulsory_info multistop = {Routes[best_index_route1][best_x].num, best_transfer, Routes[best_index_route1][best_x].compulsory};
					Routes[best_index_route2].insert(Routes[best_index_route2].begin() + best_position, multistop);
				}
				else
					Routes[best_index_route2][best_position].boarding += best_transfer;
			}

			journeyTime(journey_time, Routes); // recalculating journey times from scratch	
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
	{
		if (used_stops[i].compulsory == 0)
			non_compulsory_stops.push_back(used_stops[i]);
	}

	// Identifying a Binomial random number of non-compulsory stops to remove with n = non_compulsory_stops.size() and p = 3/non_compulsory_stops.size()
	binomial_distribution<int> bin_distribution(non_compulsory_stops.size(), 3.0 / non_compulsory_stops.size());
	int no_to_remove = 0;
	// Making sure random number is non-zero
	while (no_to_remove == 0)
	{
		no_to_remove = bin_distribution(generator);
	}

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
	for (int s = 1; s <= addresses; s++)
	{
		subset_addresses.push_back(s);
	}
	identifyAvailableStops(walk, subset_addresses, used_stops, available_stops);
	//cout << endl << "Addresses not covered by new used_stops: ";
	for (int s = 0; s < addresses; s++)
	{
		if (available_stops[s].size() == 0)
		{
			addresses_uncovered_used_stops.push_back(s + 1);
		}
	}

	if (addresses_uncovered_used_stops.size() != 0)
	{
		identifyAvailableStops(walk, addresses_uncovered_used_stops, remaining_stops, available_stops);

		int s = 0;
		while (s < addresses_uncovered_used_stops.size())
		{
			if (available_stops[s].size() == 0)
			{
				addresses_uncovered_remaining_stops.push_back(addresses_uncovered_used_stops[s]);
				addresses_uncovered_used_stops.erase(addresses_uncovered_used_stops.begin() + s);
			}
			else
				s++;
		}

		// If some addresses in addresses_uncovered_used_stops do not have an alternative stop, then at least one stop from removed_stops must be added back
		if (addresses_uncovered_remaining_stops.size() > 0)
		{
			coverAddresses(addresses_uncovered_remaining_stops, used_stops, walk, removed_stops, available_addresses, generator, true);
		}

		// Adding new stops to cover the addresses still left in addresses_uncovered_used_stops (i.e. those that have an alternative stop in remaining_stops)
		coverAddresses(addresses_uncovered_used_stops, used_stops, walk, remaining_stops, available_addresses, generator, true);
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
bool compareLoad1(const route_position_board_load a, const route_position_board_load b)
{
	return a.load < b.load;
}

// Used to order the potential inclusions of a stop in Routes in non-decreasing order of the vehicle loads - Returns True if a.load < b.load
bool compareLoad2(const route_position_load a, const route_position_load b)
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

// Removes some demand of a stop from Routes - used for elements in to_remove having negative demands
void decreaseDemandOfStopInRoutes(vector<vector<stop_board_compulsory_info> > &Routes, stop_board_compulsory_info stop)
{
	int decrease = -stop.boarding; // by how much demand of stop must decrease

	vector<route_position_board_load> occurrences;
	occurrencesOfStopInRoutes(Routes, stop, occurrences);
	sort(occurrences.begin(), occurrences.end(), compareBoarding);

	while (decrease > 0)
	{
		if (occurrences[0].boarding > decrease)
		{
			Routes[occurrences[0].route_index][occurrences[0].route_position].boarding -= decrease;
			break;
		}
		else
		{
			Routes[occurrences[0].route_index].erase(Routes[occurrences[0].route_index].begin() + occurrences[0].route_position);
			decrease -= occurrences[0].boarding;
			occurrences.erase(occurrences.begin());
		}
	}
}

// Removes all occurrences of a stop from Routes - used for elements in to_remove having positive demands
void removeStopFromRoutes(vector<vector<stop_board_compulsory_info> > &Routes, stop_board_compulsory_info stop)
{
	for (int i = 0; i < Routes.size(); i++)
	{
		int j = 1;
		while (j < Routes[i].size() - 1)
		{
			if (Routes[i][j].num == stop.num)
			{
				Routes[i].erase(Routes[i].begin() + j);
				break;
			}
			else
				j++;
		}
	}
}

// Identifies the best (before which) insertion point of stop in route
int bestInsertionPointOfStopInRoute(vector<stop_board_compulsory_info> route, stop_board_compulsory_info stop)
{
	int best_insertion_point = 1;
	double best_increase = -drive[route[0].num][route[1].num] + drive[route[0].num][stop.num] + drive[stop.num][route[1].num], increase;
	for (int j = 2; j < route.size(); j++)
	{
		increase = -drive[route[j - 1].num][route[j].num] + drive[route[j - 1].num][stop.num] + drive[stop.num][route[j].num];
		if (increase < best_increase)
		{
			best_increase = increase;
			best_insertion_point = j;
		}
	}
	return best_insertion_point;
}

// Adds a stop (at best position in the route having lowest load) to Routes - used for elements in to_add having positive demands
void addStopToRoutes(vector<vector<stop_board_compulsory_info> > &Routes, stop_board_compulsory_info stop)
{
	vector<route_position_load> inclusions;
	int best_insertion_point, route_load;

	for (int i = 0; i < Routes.size(); i++)
	{
		route_load = routeLoad(Routes[i]);
		if (route_load < cap[i])
		{
			best_insertion_point = bestInsertionPointOfStopInRoute(Routes[i], stop);
			route_position_load inclusion = {i, best_insertion_point, route_load};
			inclusions.push_back(inclusion);
		}
	}
	sort(inclusions.begin(), inclusions.end(), compareLoad2);

	while (stop.boarding > 0)
	{
		if (inclusions[0].load + stop.boarding <= cap[inclusions[0].route_index])
		{
			Routes[inclusions[0].route_index].insert(Routes[inclusions[0].route_index].begin() + inclusions[0].route_position, stop);
			stop.boarding = 0;
			break;
		}
		else
		{
			stop_board_compulsory_info partial_stop = {stop.num, cap[inclusions[0].route_index] - inclusions[0].load, stop.compulsory};
			Routes[inclusions[0].route_index].insert(Routes[inclusions[0].route_index].begin() + inclusions[0].route_position, partial_stop);
			stop.boarding -= (cap[inclusions[0].route_index] - inclusions[0].load);
			inclusions.erase(inclusions.begin());
		}
	}
}

// Adds some demand of a stop in Routes - used for elements in to_add having negative demands
void increaseDemandOfStopInRoutes(vector<vector<stop_board_compulsory_info> > &Routes, stop_board_compulsory_info stop)
{
	int increase = -stop.boarding; // by how much demand of stop must increase

	vector<route_position_board_load> occurrences;
	occurrencesOfStopInRoutes(Routes, stop, occurrences);
	sort(occurrences.begin(), occurrences.end(), compareLoad1);

	while (increase > 0 && occurrences.size() > 0)
	{
		if (occurrences[0].load + increase <= cap[occurrences[0].route_index])
		{
			Routes[occurrences[0].route_index][occurrences[0].route_position].boarding += increase;
			increase = 0;
			break;
		}
		else
		{
			if (occurrences[0].load < cap[occurrences[0].route_index])
			{
				Routes[occurrences[0].route_index][occurrences[0].route_position].boarding += (cap[occurrences[0].route_index] - occurrences[0].load);
				increase -= (cap[occurrences[0].route_index] - occurrences[0].load);
				occurrences.erase(occurrences.begin());
			}
			else
				break;
		}
	}

	if (increase > 0)
	{
		stop.boarding = increase;
		addStopToRoutes(Routes, stop);
	}
}

// --------------------------------------------------------------------- MAIN FUNCTION ---------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
	// Setting parameters (file_name, MRT, max_walking_distance, available_capacities) and opening file
	string file = argv[1], file_name = file + ".bus", prefix;
	int it_limit = atoi(argv[2]);
	MRT = atof(argv[3]);
	max_walking_distance = atof(argv[4]);
	// Creating vector of available capacities
	vector<int> available_capacities;
	for (int i = 0; i < (argc - 5) / 2; i++) 
	{
		for (int j = 0; j < atoi(argv[6 + 2*i]); j++)   available_capacities.push_back(atoi(argv[5+2*i]));
	}
	// Sorting vector of available capacities in non-increasing order
	sort(available_capacities.begin(), available_capacities.end(), greater<int>()); 

	ifstream input(file_name);

	// Getting total number of stops and total number of addresses
	string myString, line;
	int addresses, all_stops;
	getline(input, line);
	stringstream ss(line);
	getline(ss, myString, ',');
	all_stops = stoi(myString) - 1;
	getline(ss, myString, ',');
	addresses = stoi(myString);
	input.close();

	// Filling information about addresses
	input.open(file_name);
	vector<address_info> vec_addresses(addresses);
	fillAddress(all_stops, addresses, vec_addresses, input);
	input.close();

	// Calculating total number of students and minimal number of vehicles required
	int min_vehicles = 0, total = 0, no_vehicles, feasible_no_vehicles;
	vector<int> initial_cap;
	for (int i = 0; i < addresses; i++) { students += vec_addresses[i].siblings; }
	for (int i = 0; i < available_capacities.size(); i++) {
		total += available_capacities[i];
		min_vehicles++;
		if (total >= students) break;
	}
	no_vehicles = min_vehicles; 
	feasible_no_vehicles = available_capacities.size();
	// Filling initial vector of capacities of used vehicles
	for (int i = 0; i < no_vehicles; i++) {
		initial_cap.push_back(available_capacities[i]);
	}
	cap = initial_cap;

	// Filling driving times and walking distances
	input.open(file_name);
	drive.resize(all_stops + 1, vector<double>(all_stops + 1, 0));
	dist.resize(all_stops + 1, vector<double>(all_stops + 1, 0));
	fillDriveDistance(addresses, all_stops, input);
	input.close();
	input.open(file_name);
	vector<vector<double> > walk(addresses, vector<double>(all_stops, maxno));
	fillWalk(walk, addresses, all_stops, input);

	// Data structures needed in loop
	vector<vector<int> > available_addresses(all_stops);
	vector<vector<stop_walk_info> > available_stops(addresses);
	vector<int> w(addresses, 0), w_best(addresses, 0);									// stop where students living in each address will walk to
	vector<stop_board_compulsory_info> used_stops_old, used_stops_new, used_stops_best; // stops that are actually used
	vector<vector<stop_board_compulsory_info> > Routes, best_Routes;
	vector<double> journey_time;
	vector<stop_board_compulsory_info> to_add, to_remove; 								// required for solution alteration
	uniform_real_distribution<double> distribution(0.0, 1.0);
	double random_probability;

	// Timer and solution feasibility variables
	int counter, no_feasible_solutions, go, finish, route_load;
	bool solution_infeasible;
	double total_journey_time, best_total_journey_time, overall_best_total_journey_time = maxno, average_walking_distance;
	vector<vector<string> > best_solutions;
	vector<string> temp_string;
	ofstream myfile;

	double subset_alteration;
	// (i) -1 = Variant I = independent runs 
	// (ii) 1 = Variant II = random walk (altering previous subset)
	// (iii) 0 = Variant III = steepest descent (altering best subset found so far)
	// (iv) 0.5 = Variant IV = tradeoff between previous and best found so far
	
	file_results.open(file + " DetIt Results Summary.txt");
	for (int seed = 0; seed <= 24; seed++) {	   
		for (int variant = 3; variant <= 4; variant++)    
		{ 
			no_vehicles = min_vehicles; 
			cap.clear();
			cap = initial_cap;
			prefix = file + ".DetIt.Seed" + to_string(seed) + ".Variant" + to_string(variant);

			file_results << endl << file << "\tSeed " << seed << " \tVariant " << variant; 
			cout << endl << "DetIt " << file << "\tSeed " << seed << " \tVariant " << variant;    

			if (variant == 1)		subset_alteration = -1;
			else if (variant == 2)	subset_alteration = 1;
			else if (variant == 3)	subset_alteration = 0;
			else 					subset_alteration = 0.5;

			myfile.open(prefix + " - All Iterations.txt");			
			counter = 0;
			no_feasible_solutions = 0;
			best_total_journey_time = maxno;
			used_stops_best.clear();
			best_Routes.clear();
			w_best.clear();

			while (no_feasible_solutions == 0)
			{
				mt19937 generator(seed);	
				
				// Choosing first subset of stops
				subsetStopsSelection(addresses, all_stops, walk, available_stops, available_addresses, used_stops_new, generator, w, vec_addresses);

				// Nearest Neighbour constructive heuristic
				initialRoutesNN(used_stops_new, Routes, no_vehicles);
				journeyTime(journey_time, Routes);

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
								if (to_remove[i].boarding > 0)	removeStopFromRoutes(Routes, to_remove[i]);
					
							// Decreasing demand of stops in to_remove
							for (int i = 0; i < to_remove.size(); i++)
								if (to_remove[i].boarding < 0)	decreaseDemandOfStopInRoutes(Routes, to_remove[i]);

							// Increasing demand of stops in to_add
							for (int i = 0; i < to_add.size(); i++)
								if (to_add[i].boarding < 0)		increaseDemandOfStopInRoutes(Routes, to_add[i]);
						
							// Adding new stops in to_add
							for (int i = 0; i < to_add.size(); i++)
								if (to_add[i].boarding > 0)		addStopToRoutes(Routes, to_add[i]);
						
							// Recalculating journey times from stratch
							journeyTime(journey_time, Routes);
						}

						else {
							// Creating a new subset of stops
							subsetStopsSelection(addresses, all_stops, walk, available_stops, available_addresses, used_stops_new, generator, w, vec_addresses);

							initialRoutesNN(used_stops_new, Routes, no_vehicles);
							journeyTime(journey_time, Routes);
						}
					}

					// Local Search improvement heuristics
					localSearch(journey_time, Routes);
		
					// Checking whether MRT is violated
					total_journey_time = 0;
					for (int j = 0; j < journey_time.size(); j++) {
						if (journey_time[j] > MRT) {
							solution_infeasible = true;
							break;
						}
						total_journey_time += journey_time[j];
					}

					if (solution_infeasible == false)	{
						no_feasible_solutions++;

						// Updating best total journey time found so far
						if (total_journey_time <= best_total_journey_time)	{
							best_total_journey_time = total_journey_time;
							used_stops_best = used_stops_new;
							best_Routes = Routes;
							w_best = w;
						}

						// Calculating average walking distance
						average_walking_distance = 0;
						for (int s = 0; s < addresses; s++)
							average_walking_distance += (walk[s][w[s] - 1] * vec_addresses[s].siblings);
						average_walking_distance /= students;

						// Writing solution results in text file for plotting Pareto front
						myfile << total_journey_time/60 << "," << average_walking_distance << endl;
					}
				}
				finish = clock();
				file_results << "\t" << (finish - go)/double(CLOCKS_PER_SEC) << "s LSTime";

				if (no_feasible_solutions == 0)	{
					no_vehicles++;
					cap.push_back(available_capacities[no_vehicles-1]);
					counter = 0;
				}
				else {
					if (no_vehicles < feasible_no_vehicles) {
						feasible_no_vehicles = no_vehicles;
						overall_best_total_journey_time = maxno;
					}
					myfile.close();
					file_results << "\t" << counter << " iterations\t" << no_vehicles << " vehicle(s)\t" << no_feasible_solutions << " feasible solns\t" << best_total_journey_time/60 << " MJT\t";
					displayRoutes(best_Routes);
							
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
			if ((no_vehicles == feasible_no_vehicles) && (best_total_journey_time <= overall_best_total_journey_time)) {
				if (best_total_journey_time < overall_best_total_journey_time) {
					best_solutions.clear();
					overall_best_total_journey_time = best_total_journey_time;
				}
				temp_string.clear(); 
				temp_string.push_back(prefix);
				for(int i = 0; i < best_Routes.size(); i++) {
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
// #pragma warning(pop)
