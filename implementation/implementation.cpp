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
#include <string>
#include <memory>
#include "implementation.h"


namespace implementation {
    int SCALE = 4;

    GridLocation location_to_cell(int c, int r) {
        return GridLocation{c / SCALE, r / SCALE};
    };

    std::pair<int, int> cell_to_location(GridLocation loc) {
        return std::make_pair(loc.x * SCALE, loc.y * SCALE);
    };
}

//namespace std {
/* implement hash function so we can put GridLocation into an unordered_set */
template<>
struct std::hash<implementation::GridLocation> {
    typedef implementation::GridLocation argument_type;
    typedef std::size_t result_type;

    std::size_t operator()(const implementation::GridLocation &id) const noexcept {
        return std::hash<int>()(id.x ^ (id.y << 4));
    }
};
//}

namespace implementation {
    struct SquareGrid : Grid {
        static std::array<GridLocation, 8> DIRS;

        int width, height;
        std::unordered_set<GridLocation> walls;

        SquareGrid(int width_, int height_)
                : width(width_), height(height_) {}

        bool in_bounds(GridLocation id) const {
            return 0 <= id.x && id.x < width
                   && 0 <= id.y && id.y < height;
        }

        void set_walls(const std::vector<uint8_t> &data) {
            int grid_size_data = std::sqrt(data.size());
            for (int idx = 0; idx < data.size(); ++idx) {
                int row = int(idx / grid_size_data);
                int col = (idx % grid_size_data);

                if (int(data[idx]) != 0) {
                    walls.insert(location_to_cell(col, row));
                }
            }
        }

        bool passable(GridLocation id) const {
            return walls.find(id) == walls.end();
        }

        std::vector<GridLocation> neighbors(GridLocation id) const override {
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

    std::array<implementation::GridLocation, 8> SquareGrid::DIRS = {GridLocation{1, 1}, GridLocation{-1, -1},
                                                                    GridLocation{-1, 1}, GridLocation{1, -1},
                                                                    GridLocation{0, 1}, GridLocation{1, 0},
                                                                    GridLocation{0, -1}, GridLocation{-1, 0}};

// Helpers for GridLocation
    bool operator==(GridLocation a, GridLocation b) {
        return a.x == b.x && a.y == b.y;
    }

    bool operator!=(GridLocation a, GridLocation b) {
        return !(a == b);
    }

    bool operator<(GridLocation a, GridLocation b) {
        return std::tie(a.x, a.y) < std::tie(b.x, b.y);
    }

    std::basic_iostream<char>::basic_ostream &
    operator<<(std::basic_iostream<char>::basic_ostream &out, const GridLocation &loc) {
        out << '(' << loc.x << ',' << loc.y << ')';
        return out;
    }

    inline double distance(GridLocation a, GridLocation b) {
        return std::sqrt(std::pow((b.x - a.x), 2.) + std::pow((b.y - a.y), 2.));
    }

    struct GridWithWeights : SquareGrid {
        std::unordered_set<GridLocation> forests;

        GridWithWeights(int w, int h) : SquareGrid(w, h) {}

        double cost(GridLocation from_node, GridLocation to_node) const {
            return forests.find(to_node) != forests.end() ? 5 : 1;
        }
    };

    struct GridWithElevation : SquareGrid {
        std::unordered_map<GridLocation, int> hills;

        GridWithElevation(int w, int h) : SquareGrid(w, h) {}

        int cost(GridLocation node) const {
            return hills.find(node) != hills.end() ? hills.at(node) : 1;
        }

        int elevation(GridLocation from_node, GridLocation to_node) const {
            if ((hills.find(to_node) == hills.end()) or hills.find(from_node) == hills.end() or
                (to_node == from_node)) {
                return 0;
            } else
                return hills.at(to_node) - hills.at(from_node);
        }

        double distance(GridLocation from_node, GridLocation to_node) const {
            return int(std::sqrt(std::pow((to_node.x - from_node.x), 2.) + std::pow((to_node.y - from_node.y), 2.)));
        }

        void set_weights(const std::vector<uint8_t> &data) {
            int grid_size_data = std::sqrt(data.size());
            for (int idx = 0; idx < data.size(); ++idx) {
                int row = int(idx / grid_size_data);
                int col = (idx % grid_size_data);

                GridLocation loc = location_to_cell(col, row);
                if (int(data[idx]) > 0) {
                    hills.insert({loc, int(data[idx])});
                }
            }
        }
    };

    struct TravelingPoint {
        double move_cost(int elevation, double distance) {
            return (elevation == 0) ? distance : std::sqrt(std::pow(distance, 2.) + std::pow(elevation, 2.));
        }
    };

    struct Robot {
        double move_cost(int elevation, double distance) {
            if (elevation == 0) {
                return distance;
            } else {
                double distance_3d = std::sqrt(std::pow(distance, 2.) + std::pow(elevation, 2.));
                return (elevation < 0) ? distance_3d * .95 : distance_3d *
                                                          1.25; // if traveling downwards, time decreases with 5%. Travel upwards and downwards, may not be equally as coslty as on flat areas, so upwards increases time with 25%
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
            (std::shared_ptr<Graph> graph,
             Location start,
             Location goal,
             std::unordered_map<Location, Location> &came_from,
             std::unordered_map<Location, double> &cost_so_far) {
        PriorityQueue<Location, double> frontier;
        frontier.put(start, 0);

        came_from[start] = start;
        cost_so_far[start] = 0;

        Robot robot;

        while (!frontier.empty()) {
            Location current = frontier.get();

            if (current == goal) {
                break;
            }

            for (Location next : graph->neighbors(current)) {
                double new_cost = cost_so_far[current] + robot.move_cost(graph->elevation(current, next),
                                                                         graph->distance(current,
                                                                                         next)); //graph->move_cost(current, next); // + graph.cost(current, next);
                if (cost_so_far.find(next) == cost_so_far.end()
                    || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    came_from[next] = current;
                    frontier.put(next, new_cost);
                }
            }
        }
    }


    inline double heuristic(GridLocation a, GridLocation b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    template<typename Location, typename Graph>
    void a_star_search
            (std::shared_ptr<Graph> graph,
             Location start,
             Location goal,
             std::unordered_map<Location, Location> &came_from,
             std::unordered_map<Location, double> &cost_so_far) {
        PriorityQueue<Location, double> frontier;
        frontier.put(start, 0);

        came_from[start] = start;
        cost_so_far[start] = 0;

        TravelingPoint robot;

        while (!frontier.empty()) {
            Location current = frontier.get();

            if (current == goal) {
                break;
            }

            for (Location next : graph->neighbors(current)) {
                double new_cost = cost_so_far[current] + robot.move_cost(graph->elevation(current, next),
                                                                         graph->distance(current,
                                                                                         next)); //graph->move_cost(current, next); // + graph.cost(current, next);
                if (cost_so_far.find(next) == cost_so_far.end()
                    || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    double priority =
                            new_cost + heuristic(next, goal);
                    frontier.put(next, priority);
                    came_from[next] = current;
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

    template<typename Location>
    double path_length(const std::vector<Location> &path) {
        Location current = path.at(0);
        double path_length = 0;
        for (int i = 1; i < path.size(); ++i) {
            Location next = path.at(i);
            path_length += distance(current, next);
            current = next;
        }
        return path_length;
    }

    std::shared_ptr<Grid> make_grid(const std::vector<uint8_t> &overrides, const std::vector<uint8_t> &elevation) {
        int grid_size = std::sqrt(overrides.size()) / SCALE;
        std::shared_ptr<GridWithElevation> grid(new GridWithElevation(grid_size, grid_size));
        grid->set_walls(overrides);
        grid->set_weights(elevation);
        return grid;
    }

    void print_coordinate(std::pair<int, int> a) {
        printf("(%d, %d)\n", a.first, a.second);
    }

    void findShortestPath(void (*print_fun)(std::pair<int, int>), void (*search_fun)(std::shared_ptr<Grid>,
                                                                                     GridLocation,
                                                                                     GridLocation,
                                                                                     std::unordered_map<GridLocation, GridLocation> &,
                                                                                     std::unordered_map<GridLocation, double> &),
                          std::vector<std::pair<int, int>> &result, std::shared_ptr<Grid> grid,
                          const std::pair<std::pair<int, int>, std::pair<int, int>> &query) {

        std::cout << "start = ";
        print_fun(query.first);
        std::cout << "goal = ";
        print_fun(query.second);
        std::unordered_map<GridLocation, GridLocation> came_from;
        std::unordered_map<GridLocation, double> cost_so_far;
        GridLocation start = location_to_cell(query.first.first, query.first.second);
        GridLocation goal = location_to_cell(query.second.first, query.second.second);
        search_fun(grid, start, goal, came_from, cost_so_far);
        std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
        std::cout << "path_duration = " << int(cost_so_far[goal]) << std::endl;
        std::cout << "path_length = " << int(path_length(path)) << std::endl;
        for (auto pos : path)
            result.push_back(cell_to_location(pos));
    }

    void template_specialization(std::vector<std::pair<int, int>> &result, std::shared_ptr<Grid> grid,
                                 const std::pair<std::pair<int, int>, std::pair<int, int>> &query) {

        std::unordered_map<GridLocation, GridLocation> came_from;
        std::unordered_map<GridLocation, double> cost_so_far;
        GridLocation start = location_to_cell(query.first.first, query.first.second);
        GridLocation goal = location_to_cell(query.second.first, query.second.second);
        a_star_search(grid, start, goal, came_from, cost_so_far);
    }

    void template_specialization_dijkstra(std::vector<std::pair<int, int>> &result, std::shared_ptr<Grid> grid,
                                          const std::pair<std::pair<int, int>, std::pair<int, int>> &query) {

        std::unordered_map<GridLocation, GridLocation> came_from;
        std::unordered_map<GridLocation, double> cost_so_far;
        GridLocation start = location_to_cell(query.first.first, query.first.second);
        GridLocation goal = location_to_cell(query.second.first, query.second.second);
        dijkstra_search(grid, start, goal, came_from, cost_so_far);
    }

    // This outputs a grid. Pass in a distances map if you want to print
    // the distances, or pass in a point_to map if you want to print
    // arrows that point to the parent location, or pass in a path vector
    // if you want to draw the path.
    template<class Graph>
    void draw_grid(const Graph &graph, int field_width,
                   std::unordered_map<GridLocation, double> *distances = nullptr,
                   std::unordered_map<GridLocation, GridLocation> *point_to = nullptr,
                   std::vector<GridLocation> *path = nullptr) {
        for (int y = 0; y != graph.height; ++y) {
            for (int x = 0; x != graph.width; ++x) {
                GridLocation id{x, y};
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

    void add_rect(SquareGrid &grid, int x1, int y1, int x2, int y2) {
        for (int x = x1; x < x2; ++x) {
            for (int y = y1; y < y2; ++y) {
                grid.walls.insert(GridLocation{x, y});
            }
        }
    }

}
