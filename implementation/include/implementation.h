#ifndef __IMPLEMENTATION_H__
#define __IMPLEMENTATION_H__

#include <unordered_map>

namespace implementation {
    enum Algorithm {DIJKSTRA, A_STAR};
    enum Rover {SMALL, BIG};

    struct GridLocation {
        int x, y;
    };
    struct Grid{
        virtual std::vector<GridLocation> neighbors(GridLocation id) const = 0;
        virtual double move_cost(GridLocation from_node, GridLocation to_node) const = 0;
    };

    Grid* make_grid(const std::vector<uint8_t>& overrides, const std::vector<uint8_t>& elevation, Rover rov);

    void findShortestPath(std::vector<std::pair<int, int>>& result, Grid* grid, const std::pair<std::pair<int, int>, std::pair<int, int>>& query, Algorithm algo);
    void writeResult(std::string filepath, std::vector<std::pair<int, int>>& result);
}

#endif // __IMPLEMENTATION_H__