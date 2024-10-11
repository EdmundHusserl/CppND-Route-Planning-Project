#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  this->m_Model = model;
  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->start_node->parent = nullptr;
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*this->end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (RouteModel::Node *neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value =
        current_node->g_value + current_node->distance(*neighbor);
    neighbor->visited = true;
    this->open_list.push_back(neighbor);
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  auto compare = [](RouteModel::Node *x0, RouteModel::Node *x1) -> bool {
    return x0->g_value + x0->h_value > x1->g_value + x1->h_value;
  };
  std::sort(this->open_list.begin(), this->open_list.end(), compare);
  RouteModel::Node *lowest_sum_node = this->open_list.back();
  this->open_list.pop_back();
  return lowest_sum_node;
}

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;
  path_found.push_back(*current_node);

  while (current_node->parent != nullptr) {
    distance += current_node->distance(*current_node->parent);
    path_found.insert(path_found.begin(), *current_node->parent);
    current_node = current_node->parent;
  }

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;
  this->start_node->h_value = CalculateHValue(start_node);
  this->start_node->g_value = 0;
  this->start_node->visited = true;
  this->open_list.push_back(this->start_node);

  while (this->open_list.size() > 0) {
    current_node = this->NextNode();

    if (current_node->x == this->end_node->x &&
        current_node->y == this->end_node->y) {
      break;
    }
    this->AddNeighbors(current_node);
  }
  this->m_Model.path = this->ConstructFinalPath(current_node);
}