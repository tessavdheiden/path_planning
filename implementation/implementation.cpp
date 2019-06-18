#include <iostream>
#include <iomanip>
#include <stdio.h>      /* printf */
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cstdlib>
#include <math.h>       /* sqrt */
#include <chrono>
#include <thread>

#include "implementation.h"

namespace implementation{
    int SCALE = 1;
    struct GridLocation {
        int x, y;
    };
    GridLocation location_to_cell(int c, int r){
        return GridLocation{c / SCALE, r / SCALE};
    };
    std::pair<int, int> cell_to_location(GridLocation loc){
        return std::make_pair(loc.x * SCALE, loc.y * SCALE);
    };
}

    namespace std {
    /* implement hash function so we can put GridLocation into an unordered_set */
        template <> struct hash<implementation::GridLocation> {
            typedef implementation::GridLocation argument_type;
            typedef std::size_t result_type;
            std::size_t operator()(const implementation::GridLocation& id) const noexcept {
                return std::hash<int>()(id.x ^ (id.y << 4));
            }
        };
    }

namespace implementation {
    struct SquareGrid {
        static std::array<GridLocation, 8> DIRS;

        int width, height;
        std::unordered_set<GridLocation> walls;

        SquareGrid(int width_, int height_)
                : width(width_), height(height_) {}

        bool in_bounds(GridLocation id) const {
            return 0 <= id.x && id.x < width
                   && 0 <= id.y && id.y < height;
        }


        void set_walls(std::vector<uint8_t> data) {
            int grid_size_data = std::sqrt(data.size());
            for (int idx = 0; idx < data.size(); ++idx){
                int row = int(idx / grid_size_data);
                int col = (idx % grid_size_data) ;

                if (int(data[idx]) != 0) {
                    walls.insert(location_to_cell(col, row));
                }
            }
        }


        bool passable(GridLocation id) const {
            return walls.find(id) == walls.end();
        }

        std::vector<GridLocation> neighbors(GridLocation id) const {
            std::vector<GridLocation> results;

            for (GridLocation dir : DIRS) {
                GridLocation next{id.x + dir.x, id.y + dir.y};
                if (in_bounds(next) && passable(next)) {
                    results.push_back(next);
                }
            }

            if ((id.x + id.y) % 2 == 0) {
                // aesthetic improvement on square grids
                std::reverse(results.begin(), results.end());
            }

            return results;
        }
    };
    // This outputs a grid. Pass in a distances map if you want to print
// the distances, or pass in a point_to map if you want to print
// arrows that point to the parent location, or pass in a path vector
// if you want to draw the path.
    template<class Graph>
    void draw_grid(const Graph& graph, int field_width,
                   std::unordered_map<GridLocation, double>* distances=nullptr,
                   std::unordered_map<GridLocation, GridLocation>* point_to=nullptr,
                   std::vector<GridLocation>* path=nullptr) {
        for (int y = 0; y != graph.height; ++y) {
            for (int x = 0; x != graph.width; ++x) {
                GridLocation id {x, y};
                std::cout << std::left << std::setw(field_width);
                if (graph.walls.find(id) != graph.walls.end()) {
                    std::cout << std::string(field_width, '#');
                } else if (point_to != nullptr && point_to->count(id)) {
                    GridLocation next = (*point_to)[id];
                    if (next.x == x + 1) { std::cout << "> "; }
                    else if (next.x == x - 1) { std::cout << "< "; }
                    else if (next.y == y + 1) { std::cout << "v "; }
                    else if (next.y == y - 1) { std::cout << "^ "; }
                    else { std::cout << "* "; }
                } else if (distances != nullptr && distances->count(id)) {
                    std::cout << (*distances)[id];
                } else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()) {
                    std::cout << '@';
                } else {
                    std::cout << '.';
                }
            }
            std::cout << '\n';
        }
    }
    void add_rect(SquareGrid& grid, int x1, int y1, int x2, int y2) {
        for (int x = x1; x < x2; ++x) {
            for (int y = y1; y < y2; ++y) {
                grid.walls.insert(GridLocation{x, y});
            }
        }
    }


std::array<implementation::GridLocation, 8> SquareGrid::DIRS =
  {GridLocation{1, 1}, GridLocation{-1, -1}, GridLocation{-1, 1}, GridLocation{1, -1}, GridLocation{0, 1}, GridLocation{1, 0}, GridLocation{0, -1}, GridLocation{-1, 0}};

// Helpers for GridLocation

bool operator == (GridLocation a, GridLocation b) {
  return a.x == b.x && a.y == b.y;
}

bool operator != (GridLocation a, GridLocation b) {
  return !(a == b);
}

bool operator < (GridLocation a, GridLocation b) {
  return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

std::basic_iostream<char>::basic_ostream& operator<<(std::basic_iostream<char>::basic_ostream& out, const GridLocation& loc) {
  out << '(' << loc.x << ',' << loc.y << ')';
  return out;
}


struct GridWithWeights: SquareGrid {
  std::unordered_map<GridLocation, int> forests;
  std::vector<GridLocation> peaks;

  GridWithWeights(int w, int h): SquareGrid(w, h) {}

    double move_cost(GridLocation from_node, GridLocation to_node) const {
        double length = std::sqrt(std::pow((to_node.x - from_node.x), 2.) + std::pow((to_node.y - from_node.y), 2.));
        if ((forests.find(to_node) == forests.end()) or forests.find(from_node) == forests.end()){
            //if (forests.find(to_node) == forests.end())
            //    std::cout << to_node;
            //if (forests.find(from_node) == forests.end())
             //   std::cout << from_node;
            return length;
        }

        double height = forests.at(to_node) - forests.at(from_node);
        if (height == 0)
            return length;
        double distance = std::sqrt(std::pow(length, 2.) + std::pow(height, 2.));

        if (height < 0)
            return distance*1.5;
        else
            return distance*.5;

    }
  void set_weights(std::vector<uint8_t> data) {
      typedef GridLocation L;
      int grid_size_data = std::sqrt(data.size());
      int max = 0;
      for (int idx = 0; idx < data.size(); ++idx){
          int row = int(idx / grid_size_data);
          int col = (idx % grid_size_data) ;
          int val = data[idx];

          GridLocation loc = location_to_cell(col, row);
          if (int(data[idx]) > 0) {
              forests.insert({loc, int(data[idx])});
              if (int(data[idx]) >= 255) {
                  //std::cout << loc;
                  peaks.push_back(loc);
              }
          }
      }
  }

};




template<typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement>> elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};



template<typename Location, typename Graph>
void dijkstra_search
  (Graph graph,
   Location start,
   Location goal,
   std::unordered_map<Location, Location>& came_from,
   std::unordered_map<Location, double>& cost_so_far)
{
  PriorityQueue<Location, double> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;
  
  while (!frontier.empty()) {
    Location current = frontier.get();

    if (current == goal) {
      break;
    }

    for (Location next : graph.neighbors(current)) {
      double new_cost = cost_so_far[current] + graph.move_cost(current, next); // + graph.cost(current, next);
      if (cost_so_far.find(next) == cost_so_far.end()
          || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        came_from[next] = current;
        frontier.put(next, new_cost);
      }
    }
  }
}

template<typename Location>
std::vector<Location> reconstruct_path(
   Location start, Location goal,
   std::unordered_map<Location, Location> came_from
) {
  std::vector<Location> path;
  Location current = goal;
  while (current != start) {
    path.push_back(current);
    current = came_from[current];
  }
  path.push_back(start); // optional
  std::reverse(path.begin(), path.end());
  return path;
}

inline double heuristic(GridLocation a, GridLocation b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

template<typename Location, typename Graph>
void a_star_search
  (Graph graph,
   Location start,
   Location goal,
   std::unordered_map<Location, Location>& came_from,
   std::unordered_map<Location, double>& cost_so_far)
{
  PriorityQueue<Location, double> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;

    int x = 0;
    int y = 0;
    for (auto i = 0; i < graph.peaks.size(); i++) {
        x += graph.peaks.at(i).x;
        y += graph.peaks.at(i).y;
    }
    GridLocation steepest_peak;
    steepest_peak.x = int(x/graph.peaks.size());
    steepest_peak.y = int(y/graph.peaks.size());



    while (!frontier.empty()) {
    Location current = frontier.get();

    if (current == goal) {
      break;
    }

    for (Location next : graph.neighbors(current)) {
      double new_cost = cost_so_far[current] + graph.move_cost(current, next); // graph.cost(current, next);
      if (cost_so_far.find(next) == cost_so_far.end()
          || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + .5*((1 / heuristic(next, steepest_peak)) + heuristic(next, goal));
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }
  }
}

    GridWithWeights make_diagram(std::vector<uint8_t> overrides, std::vector<uint8_t> elevation) {
        int grid_size = std::sqrt(overrides.size()) / SCALE;
        GridWithWeights grid(grid_size, grid_size);
        grid.set_walls(overrides);
        grid.set_weights(elevation);
        return grid;
    }

    void findShortestPath(std::vector<std::pair<int, int>>& result, std::vector<int>& performance, std::vector<uint8_t> overrides, std::vector<uint8_t> elevation, int x1, int y1, int x2, int y2, int x3, int y3) {
        GridWithWeights grid = make_diagram(overrides, elevation);

        std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> queries(2);
        queries.at(0) = std::make_pair(std::make_pair(x1 + 50, y1 - 50), std::make_pair(x2 + 50, y2 + 50));
        queries.at(1) = std::make_pair(std::make_pair(x2 + 50, y2 + 50), std::make_pair(x3, y3 - 200));
        std::vector<std::unordered_map<GridLocation, GridLocation>> came_from(2);
        std::vector<std::unordered_map<GridLocation, double>> cost_so_far(2);

        std::vector<GridLocation> path;
        double computation_time = 0;
        int path_time = 0;
        for (int i = 0; i < queries.size(); ++i) {

            std::cout << "start = " << int(queries.at(i).first.first) << " " << int(queries.at(i).first.second) << std::endl;
            std::cout << "goal = " << int(queries.at(i).second.first) << " " << int(queries.at(i).second.second) << std::endl;
            
	     GridLocation start = location_to_cell(queries.at(i).first.first, queries.at(i).first.second);
            GridLocation goal = location_to_cell(queries.at(i).second.first, queries.at(i).second.second);

            auto t_start = std::chrono::high_resolution_clock::now();
            a_star_search(grid, start, goal, came_from.at(i), cost_so_far.at(i));
            auto t_end = std::chrono::high_resolution_clock::now();
            computation_time += std::chrono::duration<double, std::milli>(t_end-t_start).count();
            std::cout << "computation_time = " << int(computation_time) << std::endl;

            std::vector<GridLocation> path_single_query = reconstruct_path(start, goal, came_from.at(i));
            path.insert(path.end(), path_single_query.begin(), path_single_query.end());

        }

        for (int i = 0; i < path.size(); i++){
            result.push_back(cell_to_location(path.at(i)));
        }

    };
}
