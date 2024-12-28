#include "trojanmap.h"
#include <random>

//-----------------------------------------------------
// TODO: Students should implement the following:
//-----------------------------------------------------
// Letters in lower case
std::string ToLower(const std::string& str) {
    std::string result = str;
    for (auto& c : result) {
        c = std::tolower(c);
    }
    return result;
}
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return
 * -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string &id) { 
   if (data.find(id) != data.end()) {
        return data[id].lat;
    }
    return -1;  // Return -1 if the node ID does not exist
}


/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist,
 * return -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string &id) {
  if (data.find(id) != data.end()) {
        return data[id].lon;
    }
    return -1;  // Return -1 if the node ID does not exist
}



/**
 * GetName: Get the name of a Node given its id. If id does not exist, return
 * "NULL".
 *
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string &id) {
  if (data.find(id) != data.end()) {
        return data[id].name;
    }
    return "NULL";  // Return "NULL" if the node ID does not exist
}


/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return
 * an empty vector.
 *
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string &id) {
  if (data.find(id) != data.end()) {
        return data[id].neighbors;
    }
    return {};  // Return empty vector if the node ID does not exist
}


/**
 * GetID: Given a location name, return the id.
 * If the node does not exist, return an empty string.
 * The location name must be unique, which means there is only one node with the name.
 *
 * @param  {std::string} name          : location name
 * @return {std::string}               : id
 */
std::string TrojanMap::GetID(const std::string &name) {
  std::string name_lower = name;
    std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);

    for (const auto& cur : data) {
        std::string node_name = cur.second.name;
        std::transform(node_name.begin(), node_name.end(), node_name.begin(), ::tolower);
        if (node_name == name_lower) {
            return cur.first;  // Return the node ID
        }
    }
    return "";  // Return empty string if the node name does not exist
}

/**
 * GetPosition: Given a location name, return the position. If id does not
 * exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  // std::map<std::string, Node>::iterator iter;
  for(auto iter = data.begin(); iter != data.end(); iter++){
    if(iter->second.name == name){
      results.first = iter->second.lat;
      results.second = iter->second.lon;
      return results;
    }
  }
  return results;
}

/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * @param  {std::string} a          : first string
 * @param  {std::string} b          : second string
 * @return {int}                    : edit distance between two strings
 */
// The edit distance is the minimum number of operations required to transform one string into another
int TrojanMap::CalculateEditDistance(std::string a, std::string b) {  
  int m = a.size();
  int n = b.size();
  std::vector<std::vector<int>> dp(m + 1, std::vector<int>(n + 1));

  for (int i = 0; i <= m; i++){
    for (int j = 0; j <= n; j++){
      if (i == 0){
        dp[i][j] = j; // copy b on the first row
      }
      else if (j == 0){
        dp[i][j] = i; // delete a on the first column
      }
      else if (a[i - 1] == b[j - 1]){
        dp[i][j] = dp[i - 1][j - 1];
      }
      else{
        dp[i][j] = 1 + std::min({dp[i - 1][j],     // Delete
                                 dp[i][j - 1],     // Insert
                                 dp[i - 1][j - 1]  // Replace
                                });
      }
    }
  }
  return dp[m][n];
}

/**
 * FindClosestName: Given a location name, return the name with the smallest edit
 * distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : the closest name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string tmp = "";
  int minDistance = INT_MAX;
  //INT_MAX: used as a placeholder value to represent infinity or an unreachable value in algorithms 
  std::string LowerName = ToLower(name);

  for (auto& pair : data){
    std::string LocationName = ToLower(pair.second.name);
    // data.first is just an unique id but not the location name
    int distance = CalculateEditDistance(LowerName, LocationName);
    if (distance < minDistance){
      minDistance = distance;
      tmp = pair.second.name;
    }
  }
  return tmp;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {
  std::vector<std::string> results;

  // If the input name is empty, return an empty vector.if (name.empty()) freturn results;
  if (name.empty()){
    return results;
  }

  name = ToLower(name);
  for (auto iter = data.begin(); iter != data.end(); iter++) {
    std::string cur_node = iter->second.name;
    // Create a lowercase
    std::string lower_curNode = ToLower(cur_node);

    // Check if the lowercase (lower_curNode) starts with name
    if (lower_curNode.substr(0, name.size()) == name) {
      results.push_back(cur_node); 
    }
  }
  return results;
}


/**
 * GetAllCategories: Return all the possible unique location categories, i.e.
 * there should be no duplicates in the output.
 *
 * @return {std::vector<std::string>}  : all unique location categories
 */
std::vector<std::string> TrojanMap::GetAllCategories() {
  // set: avoid duplicates here
  std::unordered_set<std::string> categories_set;
  
  for (auto& pair : data){
    categories_set.insert(pair.second.attributes.begin(), pair.second.attributes.end());
  }
  // set -> vector
  std::vector<std::string> Categories(categories_set.begin(), categories_set.end());
  std::sort(Categories.begin(), Categories.end());
  return Categories;
}

/**
 * GetAllLocationsFromCategory: Return all the locations of the input category (i.e.
 * 'attributes' in data.csv). If there is no location of that category, return
 * (-1, -1). The function should be case-insensitive.
 *
 * @param  {std::string} category         : category name (attribute)
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetAllLocationsFromCategory(
    std::string category) {
  std::vector<std::string> res;

  // Convert category to lowercase for case-insensitive comparison
  std::transform(category.begin(), category.end(), category.begin(), ::tolower);

  // // Iterate over each node in the map.
  for (const auto& pair : data) {
    const Node& node = pair.second;

    // Check if any attribute matches the input category.
    for (const auto& attribute : node.attributes) {
      std::string lower_attribute = attribute;
      std::transform(lower_attribute.begin(), lower_attribute.end(), lower_attribute.begin(), ::tolower);
      
      // If there is a match, add the node's id to the result vector.
      if (lower_attribute == category) {
        res.push_back(node.id);
        break;  // No need to check further attributes for this node.
      }
    }
  }
  return res;
}



/**
 * GetLocationRegex: Given the regular expression of a location's name, your
 * program should first check whether the regular expression is valid, and if so
 * it returns all locations that match that regular expression.
 *
 * @param  {std::regex} location name      : the regular expression of location
 * names
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetLocationRegex(std::regex location) {
  std::vector<std::string> result;
  
  try {
        // Iterate over each node in the map to check if the name matches the regex
        for (const auto& pair : data) {
            const Node& node = pair.second;

            // Check if the name matches the given regular expression
            if (std::regex_match(node.name, location)) {
                result.push_back(node.id);
            }
        }
    } catch (const std::regex_error& e) {
        // If the regex is invalid, return an empty vector
        std::cerr << "Invalid regular expression: " << e.what() << std::endl;
        return {};
    }
  return result;
}

/**
 * CalculateDistance: Get the distance between 2 nodes.
 * We have provided the code for you. Please do not need to change this function.
 * You can use this function to calculate the distance between 2 nodes.
 * The distance is in mile.
 * The distance is calculated using the Haversine formula.
 * https://en.wikipedia.org/wiki/Haversine_formula
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id,
                                    const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2), 2.0) + cos(a.lat * M_PI / 180.0) *
                                           cos(b.lat * M_PI / 180.0) *
                                           pow(sin(dlon / 2), 2.0);
  double c = 2 * asin(std::min(1.0, sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations
 * inside the vector.
 * We have provided the code for you. Please do not need to change this function.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0; i < int(path.size()) - 1; i++) {
    sum += CalculateDistance(path[i], path[i + 1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path
 * which is a list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::string start_id = GetID(location1_name);
  std::string end_id = GetID(location2_name);

  if(start_id.empty() || end_id.empty()) return path; //Invalid input

  //Priority queue to store (distance, node_id)
  std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std:: greater<>> pq;

  //Distance map to store the minimum distance to each node
  std::unordered_map<std::string, double> distance;
  //Parent map to reconstruct the path
  std::unordered_map<std::string, std::string> parent;

  //Initialize distances to infinity, except for the starting node
  for(auto &pair : data){
    distance[pair.first] = std::numeric_limits<double>::infinity();
  }
  distance[start_id] = 0;
  pq.push({0, start_id});

  while (!pq.empty()){
    auto [cur_dist, cur_node] = pq.top();
    pq.pop();

    // If we reach the target node, stop
    if (cur_node == end_id) break;
    // Skip nodes that have already been processed with a smaller distance
    if (cur_dist > distance[cur_node]) continue;

    // Relaxation step
    for (auto &neighbor_id : data[cur_node].neighbors){
      double new_dist = cur_dist + CalculateDistance(cur_node, neighbor_id);
      if (new_dist <distance[neighbor_id]){
        distance[neighbor_id] = new_dist;
        parent[neighbor_id] = cur_node;
        pq.push({new_dist, neighbor_id});
      }
    }
  }

  //Reconstruct the path from end to start
  if (distance[end_id] == std::numeric_limits<double>::infinity()) return {};

  for (std::string at = end_id; at != ""; at = parent[at]) {
    path.push_back(at);
  }
  std::reverse(path.begin(), path.end());
  return path;
}


/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest
 * path which is a list of id. Hint: Do the early termination when there is no
 * change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::string start_id = GetID(location1_name);
  std::string end_id = GetID(location2_name);

  if(start_id.empty() || end_id.empty()) return path; //Invalid input

  //Distance map to store the minimum distance to each node
  std::unordered_map<std::string, double> distance;
  //Parent map to reconstruct the path
  std::unordered_map<std::string, std::string> parent;

  //Initialize distances to infinity, except for the starting node
  for(auto &pair : data){
    distance[pair.first] = std::numeric_limits<double>::infinity();
  }
  distance[start_id] = 0;
  
  int n = data.size();

  // Relax all edges up to (n - 1) times
  for (int i = 0; i < n - 1; ++i){
    bool updated = false;
    
    for (auto &pair : data){
      std::string u = pair.first;
      if (distance[u] == std::numeric_limits<double>::infinity()) continue;
      for (auto &v : data[u].neighbors){
        double new_dist = distance[u] + CalculateDistance(u,v);
        
        if (new_dist < distance[v]){
          distance[v] = new_dist;
          parent[v] = u;
          updated = true;
        }
      }
    }

    if (!updated) break; // Early termination if no update in this iteration
  }

  // Check for negative weight cycles (not needed here since we don't have negative weights)

  // Reconstruct the path from end to start
  if(distance[end_id] == std::numeric_limits<double>::infinity()) return {}; 

  for (std::string at = end_id; at != ""; at = parent[at]){
    if (parent.find(at) == parent.end() && at != start_id) return {}; 
    path.push_back(at);
  }
  std::reverse(path.begin(),path.end());
  return path;
}
/**
 * CalculateShortestPath_Bellman_Ford_Optimized: Given 2 locations, return the shortest
 * path which is a list of id. This version adds optimization to terminate early if no change occurs.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford_Optimized(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::string start_id = GetID(location1_name);
  std::string end_id = GetID(location2_name);

  if (start_id.empty() || end_id.empty()) return path; // Invalid input

  // Distance map to store the minimum distance to each node
  std::unordered_map<std::string, double> distance;
  // Parent map to reconstruct the path
  std::unordered_map<std::string, std::string> parent;

  // Initialize distances to infinity, except for the starting node
  for (auto &pair : data) {
    distance[pair.first] = std::numeric_limits<double>::infinity();
  }
  distance[start_id] = 0;

  int n = data.size();

  // Relax all edges up to (n - 1) times
  for (int i = 0; i < n - 1; ++i) {
    bool updated = false;

    for (auto &pair : data) {
      std::string u = pair.first;
      if (distance[u] == std::numeric_limits<double>::infinity()) continue;
      for (auto &v : data[u].neighbors) {
        double new_dist = distance[u] + CalculateDistance(u, v);

        if (new_dist < distance[v]) {
          distance[v] = new_dist;
          parent[v] = u;
          updated = true;
        }
      }
    }

    if (!updated) break; // Early termination if no update in this iteration
  }

  // Check for negative weight cycles (not needed here since we don't have negative weights)

  // Reconstruct the path from end to start
  if (distance[end_id] == std::numeric_limits<double>::infinity()) return {};

  for (std::string at = end_id; at != ""; at = parent[at]) {
    path.push_back(at);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * Traveling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path, 
 *                                                                      for example: {10.3, {{0, 1, 2, 3, 4, 0}, {0, 1, 2, 3, 4, 0}, {0, 4, 3, 2, 1, 0}}},
 *                                                                      where 10.3 is the total distance, 
 *                                                                      and the first vector is the path from 0 and travse all the nodes and back to 0,
 *                                                                      and the second vector is the path shorter than the first one,
 *                                                                      and the last vector is the shortest path.
 */
// Please use brute force to implement this function, ie. find all the permutations.
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  double min_distance = std::numeric_limits<double>::infinity();
  std::vector<std::string> best_path;

  // Input validation
  for (const auto &id : location_ids) {
    if (data.find(id) == data.end()) {
      return {0, {}}; // Invalid node, return empty result
    }
  }
  // Base case for single node
  if (location_ids.size() == 1) {
    records.first = 0.0;
    records.second.push_back(location_ids);
    return records;
  }

  // Generate all permutations of location_ids, starting from the second element
  std::sort(location_ids.begin() + 1, location_ids.end());
  do {
    // Complete the route by adding the start point at the end
    std::vector<std::string> current_path = location_ids;
    current_path.push_back(location_ids[0]);

    // Calculate the distance for the current permutation
    double total_distance = CalculatePathLength(current_path);

    // Store the current path in progress
    records.second.push_back(current_path);

    // Update the shortest path and minimum distance
    if (total_distance < min_distance) {
      min_distance = total_distance;
      best_path = current_path;
    }
  } while (std::next_permutation(location_ids.begin() + 1, location_ids.end()));

  // Ensure the shortest path is stored as the last element
  records.first = min_distance;
  records.second.push_back(best_path);

  return records;
}

// Please use backtracking to implement this function
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;

  double min_distance = std::numeric_limits<double>::infinity();  // To keep track of the minimum distance found
  std::vector<std::vector<std::string>> progress;  // To store the progress
  std::vector<std::string> current_path;  // To keep track of the current path being explored
  std::unordered_set<std::string> visited;  // Set to keep track of visited locations

  // Empty Input
  if (location_ids.empty()) {
    records.first = 0.0;
    records.second = {};
    return records;
  }
  // Base case for single node
  if (location_ids.size() == 1) {
    records.first = 0.0;
    records.second.push_back(location_ids);
    return records;
  }

  // Helper function for recursive backtracking
  std::function<void(double, std::vector<std::string>&)> backtrack;

  backtrack = [&](double current_distance, std::vector<std::string>& current_path) {
    // Base case: if all locations have been visited, calculate the return distance
    if (current_path.size() == location_ids.size()) {
      double return_distance = CalculateDistance(current_path.back(), location_ids[0]);
      current_distance += return_distance;

      // Check if the current path has a shorter distance than the best one found so far
      if (current_distance < min_distance) {
        min_distance = current_distance;
        std::vector<std::string> completed_path = current_path;
        completed_path.push_back(location_ids[0]);  // Add the start location to complete the path
        progress.push_back(completed_path);
      }

      return;  // Backtrack
    }

    // Explore all unvisited locations
    for (auto &loc : location_ids) {
      if (visited.find(loc) == visited.end()) {  // If the location has not been visited
        visited.insert(loc);  // Mark as visited
        current_path.push_back(loc);  // Add to the current path

        double next_distance = current_distance + CalculateDistance(current_path[current_path.size() - 2], loc);

        // Prune paths if the current distance is already worse than the best found so far
        if (next_distance < min_distance) {
          backtrack(next_distance, current_path);  // Recurse to explore the next location
        }

        // Backtrack: undo the visit and the path addition
        visited.erase(loc);
        current_path.pop_back();
      }
    }
  };

  // Initialize backtracking with the first location
  current_path.push_back(location_ids[0]);
  visited.insert(location_ids[0]);
  backtrack(0.0, current_path);

  // Store the best route found
  records.first = min_distance;
  records.second = progress;
  return records;
}

// Hint: https://en.wikipedia.org/wiki/2-opt
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;

  // Input validation
  for (const auto &id : location_ids) {
    if (data.find(id) == data.end()) {
      return {0, {}}; // Invalid node, return empty result
    }
  }
  // Base case for single node
  if (location_ids.size() == 1) {
    records.first = 0.0;
    records.second.push_back(location_ids);
    return records;
  }

  // Create an initial path using a greedy approach
  std::vector<std::string> best_path;
  std::unordered_set<std::string> visited; // Return to the start point

  std::string current = location_ids[0];
  best_path.push_back(current);
  visited.insert(current);

  while (best_path.size() < location_ids.size()) {
    double min_distance = std::numeric_limits<double>::infinity();
    std::string next_node;

    for (const auto &loc : location_ids) {
      if (visited.find(loc) == visited.end()) {
        double dist = CalculateDistance(current, loc);
        if (dist < min_distance) {
          min_distance = dist;
          next_node = loc;
        }
      }
    }

    best_path.push_back(next_node);
    visited.insert(next_node);
    current = next_node;
  }
  // Complete the path by returning to the start
  best_path.push_back(location_ids[0]);
  double best_distance = CalculatePathLength(best_path);
  records.second.push_back(best_path);

  // Apply the 2-opt algorithm to refine the path
  bool improvement = true;
  while (improvement) {
    improvement = false;

    for (int i = 1; i < best_path.size() - 2; ++i) {
      for (int j = i + 1; j < best_path.size() - 1; ++j) {
        // Reverse the segment between i and j
        std::vector<std::string> new_path = best_path;
        std::reverse(new_path.begin() + i, new_path.begin() + j + 1);

        // Calculate the new path length
        double new_distance = CalculatePathLength(new_path);

        // If the new path is better, update the best path
        if (new_distance < best_distance) {
          best_distance = new_distance;
          best_path = new_path;
          improvement = true;
          records.second.push_back(best_path);  // Record progress
        }
      }
    }
  }

  // Ensure the shortest path is stored as the last element
  records.first = best_distance;
  records.second.push_back(best_path);
  return records;
}

// This is optional
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_3opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;

  // Empty Input
  if (location_ids.empty()) {
    records.first = 0.0;
    records.second = {};
    return records;
  }
  // Base case for single node
  if (location_ids.size() == 1) {
    records.first = 0.0;
    records.second.push_back(location_ids);
    return records;
  }

  // Initialize the path as a closed loop
  std::vector<std::string> best_path = location_ids;
  best_path.push_back(location_ids[0]);  // Return to the start point
  double best_distance = CalculatePathLength(best_path);
  records.second.push_back(best_path);  // Record the initial path

  // Apply 3-opt optimization
  bool improvement = true;
  int max_iterations = 1000;  // Define a limit for the number of iterations
  int iteration_count = 0;

  while (improvement && iteration_count < max_iterations) {
    improvement = false;
    iteration_count++;

    for (int i = 1; i < best_path.size() - 2; i++) {
      for (int j = i + 1; j < best_path.size() - 1; j++) {
        for (int k = j + 1; k < best_path.size(); k++) {
          // Generate possible new paths from 3-opt
          std::vector<std::vector<std::string>> new_paths(4, best_path);

          std::reverse(new_paths[1].begin() + i, new_paths[1].begin() + j + 1);
          std::reverse(new_paths[2].begin() + j, new_paths[2].begin() + k + 1);
          std::reverse(new_paths[3].begin() + i, new_paths[3].begin() + k + 1);

          for (auto &new_path : new_paths) {
            // Ensure the path returns to the start node
            if (new_path.back() != location_ids[0]) {
              new_path.push_back(location_ids[0]);
            }

            // Calculate the distance of the new path
            double new_distance = CalculatePathLength(new_path);

            // If the new path is better, update the best path and distance
            if (new_distance < best_distance) {
              best_distance = new_distance;
              best_path = new_path;
              improvement = true;
              records.second.push_back(best_path);  // Record progress
            }
          }
        }
      }
    }
  }

  // Store the final best path and distance
  records.first = best_distance;
  records.second.push_back(best_path);
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_locations.csv"
 *   File content:
 *    Name
 *    Ralphs
 *    KFC
 *    Chick-fil-A
 *   Output: ['Ralphs', 'KFC', 'Chick-fil-A']
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(
    std::string locations_filename) {
  std::vector<std::string> location_names_from_csv;
  std::fstream fin(locations_filename, std::ios::in);

  if (!fin.is_open()) {
    std::cerr << "Error: Could not open file " << locations_filename << "\n";
    return location_names_from_csv;
  }

  std::string line;
  bool is_header = true;  // Flag to skip the first line (header)

  while (getline(fin, line)) {
    if (is_header) {
      is_header = false;  // Skip the header
      continue;
    }
    std::stringstream ss(line);
    std::string location;

    if (getline(ss, location)) {
      location_names_from_csv.push_back(location);
    }
  }

  fin.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_dependencies.csv"
 *   File content:
 *     Source,Destination
 *     Ralphs,Chick-fil-A
 *     Ralphs,KFC
 *     Chick-fil-A,KFC
 *   Output: [['Ralphs', 'Chick-fil-A'], ['Ralphs', 'KFC'], ['Chick-fil-A', 'KFC']]
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(
    std::string dependencies_filename) {
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream fin(dependencies_filename, std::ios::in);

  if (!fin.is_open()) {
    std::cerr << "Error: Could not open file " << dependencies_filename << "\n";
    return dependencies_from_csv;
  }

  std::string line;
  bool is_header = true;  // Flag to skip the first line (header)

  while (getline(fin, line)) {
    if (is_header) {
      is_header = false;  // Skip the header
      continue;
    }

    std::stringstream ss(line);
    std::string source, destination;

    if (getline(ss, source, ',') && getline(ss, destination)) {
      dependencies_from_csv.push_back({source, destination});
    }
  }

  fin.close();
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a
 * sorting of nodes that satisfies the given dependencies. If there is no way to
 * do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(
    std::vector<std::string> &locations,
    std::vector<std::vector<std::string>> &dependencies) {
  std::vector<std::string> result;
  std::unordered_map<std::string, std::unordered_set<std::string>> graph;
  std::unordered_map<std::string, int> in_degree;

  // Initialize graph and in-degree map
  for (const auto &location : locations) {
    graph[location] = {};
    in_degree[location] = 0;
  }

  // Build graph and in-degree map based on dependencies
  for (const auto &dependency : dependencies) {
    if (dependency.size() != 2) continue;
    const std::string &from = dependency[0];
    const std::string &to = dependency[1];

    if (graph[from].find(to) == graph[from].end()) {
      graph[from].insert(to);
      in_degree[to]++;
    }
  }

  // Perform topological sort using Kahn's algorithm
  std::queue<std::string> zero_in_degree;

  // Find nodes with zero in-degree
  for (const auto &entry : in_degree) {
    if (entry.second == 0) {
      zero_in_degree.push(entry.first);
    }
  }

  // Process nodes with zero in-degree
  while (!zero_in_degree.empty()) {
    std::string current = zero_in_degree.front();
    zero_in_degree.pop();
    result.push_back(current);

    for (const auto &neighbor : graph[current]) {
      in_degree[neighbor]--;
      if (in_degree[neighbor] == 0) {
        zero_in_degree.push(neighbor);
      }
    }
  }

  // Check if there was a cycle (if result does not include all nodes)
  if (result.size() != locations.size()) {
    return {};
  }

  return result;    
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  if (data.find(id) == data.end()) return false;
  Node node = data[id];
  double lat = node.lat;
  double lon = node.lon;
  if (lon >= square[0] && lon <= square[1] && lat <= square[2] && lat >= square[3]) return true;
  return false;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location
 * ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square
 * area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the
 * square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;

  for (auto& pair : data){
    if (inSquare(pair.first, square)) subgraph.push_back(pair.first);
  }
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true
 * if there is a cycle path inside the square, false otherwise.
 *
 * @param {std::vector<std::string>} subgraph: list of location ids in the
 * square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
    // Step 1: Build adjacency list for the subgraph within the square
    std::unordered_map<std::string, std::vector<std::string>> adj;
    for (const auto &id : subgraph) {
        for (const auto &neighbor : data[id].neighbors) {
            if (inSquare(neighbor, square)) {
                adj[id].push_back(neighbor);
            }
        }
    }

    // Step 2: DFS function to detect cycles
    std::unordered_set<std::string> visited;

    auto dfs = [&](const std::string &node, const std::string &parent, auto &dfs_ref) -> bool {
        visited.insert(node);
        for (const auto &neighbor : adj[node]) {
            if (neighbor != parent) {  // Skip the edge that led to this node
                if (visited.count(neighbor) || dfs_ref(neighbor, node, dfs_ref)) {
                    return true;  // Cycle found
                }
            }
        }
        return false;
    };

    // Step 3: Check for cycles in the subgraph
    for (const auto &node : subgraph) {
        if (!visited.count(node)) {
            if (dfs(node, "", dfs)) {
                std::cout << "Cycle Path: " << node << std::endl;
                return true;
            }
        }
    }

    return false;  // No cycle found
}

/**
 * FindNearby: Given a class name C, a location name L and a number r,
 * find all locations in class C on the map near L with the range of r and
 * return a vector of string ids
 *
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {double} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
    std::vector<std::pair<double, std::string>> ValidLocations;

    // Get the ID of the starting location
    std::string start_id = GetID(name);
    if (start_id.empty()) {
        std::cerr << "Location not found!" << std::endl;
        return res;
    }
    // Iterate over all nodes in the map
    for (const auto &d : data) {
        const Node &node = d.second;
        // Skip the starting location or nodes that don't match the attribute
        if (node.id == start_id || node.attributes.find(attributesName) == node.attributes.end()) {
            continue;
        }
        // Calculate the distance between the starting node and the current node
        double distance = CalculateDistance(start_id, node.id);
        // Check if the node is within the specified radius
        if (distance <= r) {
            ValidLocations.push_back({distance, node.id});
        }
    }
    // Sort the valid locations by distance (nearest to farthest)
    std::sort(ValidLocations.begin(), ValidLocations.end());
    // Collect the top k results
    for (int i = 0; i < std::min(k, (int)ValidLocations.size()); i++) {
        res.push_back(ValidLocations[i].second);
    }
    return res;
}

/**
 * Shortest Path to Visit All Nodes: Given a list of locations, return the shortest
 * path which visit all the places and no need to go back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::vector<std::string> }      : the shortest path
 */
std::vector<std::string> TrojanMap::TrojanPath(std::vector<std::string> &location_names) {
    if (location_names.empty()) return {};

    // Map names to IDs and sort to fix the starting point
    std::vector<std::pair<std::string, std::string>> named_ids;
    for (const auto& name : location_names) {
        std::string id = GetID(name);
        if (id.empty()) {
            std::cerr << "Location '" << name << "' not found on the map.\n";
            return {};
        }
        named_ids.emplace_back(id, name);
    }

    // Sort based on IDs to fix the starting node
    std::sort(named_ids.begin(), named_ids.end());

    // Use the fixed starting point for consistent results
    std::string start_id = named_ids.front().first;
    std::vector<std::string> path;
    path.push_back(start_id);

    // Greedy TSP from fixed start point
    std::unordered_set<std::string> visited;
    visited.insert(start_id);

    std::string current_id = start_id;
    while (visited.size() < named_ids.size()) {
        double min_distance = std::numeric_limits<double>::infinity();
        std::string next_id;
        std::vector<std::string> shortest_path;

        for (const auto& [id, name] : named_ids) {
            if (visited.find(id) == visited.end()) {
                auto temp_path = CalculateShortestPath_Dijkstra(GetName(current_id), name);
                double temp_distance = CalculatePathLength(temp_path);
                if (temp_distance < min_distance) {
                    min_distance = temp_distance;
                    next_id = id;
                    shortest_path = temp_path;
                }
            }
        }

        if (next_id.empty()) return {}; // No path found, return empty vector

        // Merge path avoiding duplicate start node
        shortest_path.erase(shortest_path.begin()); // Remove duplicate start node from the path
        path.insert(path.end(), shortest_path.begin(), shortest_path.end());
        visited.insert(next_id);
        current_id = next_id;
    }

    return path;
}

/**
 * Given a vector of queries, find whether there is a path between the two locations with the constraint of the gas tank.
 *
 * @param  {std::vector<std::pair<double, std::vector<std::string>>>} Q : a list of queries
 * @return {std::vector<bool> }      : existence of the path
 */
std::vector<bool> TrojanMap::Queries(const std::vector<std::pair<double, std::vector<std::string>>>& q) {
    std::vector<bool> ans(q.size());
    std::vector<std::tuple<std::string, std::string, double>> edges;

    // Debug: Start extracting edges
    // std::cout << "Extracting edges from the map..." << std::endl;

    // Extract edges from the map
    for (const auto& [id, node] : data) {
        for (const auto& neighbor_id : node.neighbors) {
            // Avoid duplicate edges
            if (id < neighbor_id) {
                double distance = CalculateDistance(id, neighbor_id);
                edges.emplace_back(id, neighbor_id, distance);
                // Debug: Edge added
                // std::cout << "Edge added: " << id << " - " << neighbor_id << ", Distance: " << distance << std::endl;
            }
        }
    }

    // Debug: All edges extracted
    // std::cout << "Total edges extracted: " << edges.size() << std::endl;

    // Sort edges by distance
    std::sort(edges.begin(), edges.end(), [](const auto& a, const auto& b) {
        return std::get<2>(a) < std::get<2>(b);
    });

    // Debug: Edges sorted by distance
    // std::cout << "Edges sorted by distance." << std::endl;

    // Process each query
    for (size_t i = 0; i < q.size(); ++i) {
        const double tank_capacity = q[i].first;
        const std::string& source_name = q[i].second[0];
        const std::string& destination_name = q[i].second[1];

        // Translate names to IDs
        std::string source_id = GetID(source_name);
        std::string destination_id = GetID(destination_name);

        // Check if nodes exist
        if (source_id.empty() || destination_id.empty()) {
            // std::cout << "One or both nodes do not exist in the map. Source: " << source_name
            //           << " (" << source_id << "), Destination: " << destination_name
            //           << " (" << destination_id << ")" << std::endl;
            ans[i] = false;
            continue;
        }

        // Debug: Node IDs found
        // std::cout << "Source ID: " << source_id << ", Destination ID: " << destination_id << std::endl;

        // Reset Union-Find
        UnionFind uf;
        for (const auto& [id, _] : data) {
            uf.makeSet(id);
        }

        // Debug: Union-Find initialized
        // std::cout << "Union-Find initialized for all nodes." << std::endl;

        // Add edges to Union-Find based on tank capacity
        for (const auto& [u, v, dist] : edges) {
            if (dist > tank_capacity) {
                // Debug: Skipping edge due to distance
                // std::cout << "Skipping edge: " << u << " - " << v << ", Distance: " << dist
                //           << " exceeds tank capacity: " << tank_capacity << std::endl;
                break;
            }
            uf.unionSet(u, v);
            // Debug: Edge added to Union-Find
            // std::cout << "Unioned: " << u << " - " << v << ", Distance: " << dist << std::endl;
        }

        // Check connectivity
        if (uf.find(source_id) == uf.find(destination_id)) {
            ans[i] = true;
            // std::cout << "Source and Destination are connected. Query result: true" << std::endl;
        } else {
            ans[i] = false;
            // std::cout << "Source and Destination are NOT connected. Query result: false" << std::endl;
        }
    }

    // Debug: All queries processed
    std::cout << "All queries processed. Results: ";
    for (bool result : ans) {
        std::cout << (result ? "true" : "false") << " ";
    }
    std::cout << std::endl;

    return ans;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * We have provided the code for you. Please do not need to change this function.
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0])) n.attributes.insert(word);
        if (isdigit(word[0])) n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

// genetic algorithm
std::vector<std::string> TrojanMap::GenerateRandomPath(const std::vector<std::string> &nodes) {
    std::vector<std::string> path = nodes;
    std::mt19937 g(42);
    std::shuffle(path.begin(), path.end(), g);
    return path;
}
void TrojanMap::MutatePath(std::vector<std::string> &path) {
    std::random_device rd;
    std::mt19937 g(rd());
    std::uniform_int_distribution<size_t> dist(0, path.size() - 1);
    size_t i = dist(g);
    size_t j = dist(g);
    std::swap(path[i], path[j]);
}
std::vector<std::string> TrojanMap::Crossover(const std::vector<std::string> &parent1, const std::vector<std::string> &parent2) {
    std::unordered_set<std::string> visited;
    std::vector<std::string> child;

    // Take a portion from parent1
    size_t midpoint = parent1.size() / 2;
    for (size_t i = 0; i < midpoint; ++i) {
        child.push_back(parent1[i]);
        visited.insert(parent1[i]);
    }

    // Fill the rest with parent2
    for (const auto &node : parent2) {
        if (visited.find(node) == visited.end()) {
            child.push_back(node);
        }
    }

    return child;
}

std::pair<double, std::vector<std::string>> TrojanMap::TravelingTrojan_GeneticAlgorithm(
    const std::vector<std::string> &location_names, int population_size, int generations, double mutation_rate) {
    
    if (location_names.empty()) {
        return {0.0, {}};  // Return zero distance and an empty path
    }
    std::vector<std::string> location_ids;
    
    // Convert location names to IDs using GetID
    for (const auto &name : location_names) {
        std::string id = GetID(name);
        if (id.empty()) {
            std::cerr << "Error: Location name \"" << name << "\" not found in the map!" << std::endl;
            return {0.0, {}};  // Return an empty result if any name is invalid
        }
        location_ids.push_back(id);
    }

    // Initialize the population
    std::vector<std::vector<std::string>> population;
    std::random_device rd;
    std::mt19937 g(rd());

    for (int i = 0; i < population_size; ++i) {
        std::vector<std::string> path = location_ids;
        std::shuffle(path.begin(), path.end(), g);
        population.push_back(path);
    }

    std::pair<double, std::vector<std::string>> best_solution = {std::numeric_limits<double>::max(), {}};

    // Main genetic algorithm loop
    for (int gen = 0; gen < generations; ++gen) {
        // Step 1: Evaluate fitness (distance) of each path
        std::vector<std::pair<double, std::vector<std::string>>> fitness_population;
        for (const auto &path : population) {
            double distance = CalculatePathLength(path);
            fitness_population.push_back({distance, path});
        }

        // Sort population by fitness (lower distance is better)
        std::sort(fitness_population.begin(), fitness_population.end());

        // Update the best solution
        if (fitness_population[0].first < best_solution.first) {
            best_solution = fitness_population[0];
        }

        // Print generation information
        std::cout << "Generation " << gen + 1 << ": Best distance = " << best_solution.first << std::endl;

        // Step 2: Selection (top 50%)
        population.clear();
        for (size_t i = 0; i < fitness_population.size() / 2; ++i) {
            population.push_back(fitness_population[i].second);
        }

        // Step 3: Crossover (create new population)
        while (population.size() < population_size) {
            std::uniform_int_distribution<size_t> dist(0, fitness_population.size() / 2 - 1);
            size_t parent1_idx = dist(g);
            size_t parent2_idx = dist(g);

            auto child = Crossover(fitness_population[parent1_idx].second, fitness_population[parent2_idx].second);
            population.push_back(child);
        }

        // Step 4: Mutation
        for (auto &path : population) {
            if (std::generate_canonical<double, 10>(g) < mutation_rate) {
                MutatePath(path);
            }
        }
    }

    // Ensure the best path forms a cycle by adding the starting location to the end
    best_solution.second.push_back(best_solution.second.front());

    return best_solution;
}
