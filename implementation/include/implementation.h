#ifndef __IMPLEMENTATION_H__
#define __IMPLEMENTATION_H__

#include <unordered_map>
#include <iostream>
#include <queue>

namespace implementation {
    struct GridLocation {
        int x, y;
    };

    struct Grid {
        virtual std::vector<GridLocation> neighbors(GridLocation id) const = 0;
        virtual int elevation(GridLocation from_node, GridLocation to_node) const = 0;
        virtual double distance(GridLocation from_node, GridLocation to_node) const = 0;
    };

    Grid *make_grid(const std::vector<uint8_t> &overrides, const std::vector<uint8_t> &elevation);

    void print_coordinate(std::pair<int, int> a);

    template<typename Location, typename Graph>
    void dijkstra_search
            (Graph *graph,
             Location start,
             Location goal,
             std::unordered_map<Location, Location> &came_from,
             std::unordered_map<Location, double> &cost_so_far);

    template<typename Location, typename Graph>
    void a_star_search
            (Graph* graph,
             Location start,
             Location goal,
             std::unordered_map<Location, Location> &came_from,
             std::unordered_map<Location, double> &cost_so_far);

    void findShortestPath(void (*print_fun)(std::pair<int, int>), void (*search_fun)(implementation::Grid *,
                                                                                      implementation::GridLocation ,
                                                                                      implementation::GridLocation ,
                                                                                      std::unordered_map<implementation::GridLocation, implementation::GridLocation> &,
                                                                                      std::unordered_map<implementation::GridLocation, double> &),
                           std::vector<std::pair<int, int>> &result, Grid *grid,
                           const std::pair<std::pair<int, int>, std::pair<int, int>> &query);
}



#endif // __IMPLEMENTATION_H__




















