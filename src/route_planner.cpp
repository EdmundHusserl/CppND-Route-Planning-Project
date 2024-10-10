#include "route_planner.h"
#include <algorithm>
#include <chrono> // For std::chrono::milliseconds
#include <thread> // For std::this_thread::sleep_for

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to
  // the starting and ending coordinates. Store the nodes you find in the
  // RoutePlanner's start_node and end_node attributes.
  this->m_Model = model;
  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->start_node->parent = nullptr;
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another
// node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*this->end_node);
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding
// all unvisited neighbors to the open list. Tips:
// - Use the FindNeighbors() method of the current_node to populate
// current_node.neighbors vector with all the
// neighbors.std::vector<RouteModel::Node> neighbors =
// current_node->FindNeighbors()
// - For each node in current_node.neighbors, set the parent, the h_value, the
// g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and
// set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value += neighbor->distance(*current_node);
    this->open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}

// TODO 5: Complete the NextNode method to sort the open list and return the
// next node. Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
  auto compare = [](RouteModel::Node *x0, RouteModel::Node *x1) -> bool {
    return x0->g_value + x0->h_value < x1->g_value + x1->h_value;
  };
  std::sort(this->open_list.begin(), this->open_list.end(), compare);
  RouteModel::Node *lowest_sum_node = this->open_list.front();
  this->open_list.erase(this->open_list.begin());
  return lowest_sum_node;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found
// from your A* search. Tips:
// - This method should take the current (final) node as an argument and
// iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to
// the distance variable.
// - The returned vector should be in the correct order: the start node should
// be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;
  RouteModel::Node *parent = current_node->parent;
  path_found.push_back(*current_node);

  while (current_node->x != this->start_node->x && current_node->y &&
         this->start_node->y) {
    distance += current_node->distance(*parent);
    path_found.insert(path_found.begin(), *parent);
    current_node = parent;
    parent = current_node->parent;
  }

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node
// to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method
// to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits.
// This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;
  this->open_list.push_back(this->start_node);

  while (this->open_list.size() > 0) {
    current_node = this->NextNode();
    this->open_list.erase(
        std::remove_if(
            this->open_list.begin(), this->open_list.end(),
            [](RouteModel::Node *node) {
              return node->visited; // Condition: Remove visited nodes
            }),
        this->open_list.end());
    if (current_node == this->end_node) {
      break;
    }
    this->AddNeighbors(current_node);
  }
  this->m_Model.path = this->ConstructFinalPath(current_node);
}