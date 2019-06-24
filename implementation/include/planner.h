#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <sstream>
#include <fstream>
#include "implementation.h"


std::pair<std::pair<int, int>, std::pair<int, int>> make_query(int x1, int y1, int x2, int y2) {
    return std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2));
};

class Planner {
private:
    implementation::Grid *grid;
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> queries;
    std::vector<std::vector<std::pair<int, int>>> paths;
    std::vector<std::pair<int, int>> result;

public:
    void set_data(const std::vector<uint8_t> &overrides, const std::vector<uint8_t> &elevation) {
        implementation::Rover rover = implementation::SMALL;
        grid = implementation::make_grid(overrides, elevation, rover);
    }

    void set_queries(std::vector<int> vec) {
        int i = 0;
        while (i < vec.size() - 2) {
            queries.push_back(make_query(vec[i], vec[i + 1], vec[i + 2], vec[i + 3]));
            i += 2;
        }
        paths = std::vector<std::vector<std::pair<int, int>>>(queries.size());
    }

    void search_threads() {
        std::vector<std::thread> threads(queries.size());

        for (int i = 0; i < queries.size(); i++) {
            threads.at(i) = std::thread(implementation::findShortestPath, implementation::print_coordinate,
                                        implementation::a_star_search<implementation::GridLocation, implementation::Grid>,
                                        std::ref(paths.at(i)), std::ref(grid), std::ref(queries.at(i)));
        }
        for (int i = 0; i < queries.size(); i++) {
            threads.at(i).join();
        }
    }

    void search() {
        for (int i = 0; i < queries.size(); i++) {
            implementation::findShortestPath(implementation::print_coordinate,
                                             implementation::a_star_search<implementation::GridLocation, implementation::Grid>,
                                             std::ref(paths.at(i)), grid,
                                             std::ref(queries.at(i)));
        }
        for (int i = 1; i < queries.size(); i++) {
            paths.at(0).insert(paths.at(0).end(), paths.at(i).begin(), paths.at(i).end());
        }
        result = paths.at(0);
    }

    void save_result(std::string filepath) {
        std::ofstream out(filepath);
        std::stringstream ss;

        for (auto location : result)
            ss << location.first << " " << location.second << "\n";

        out << ss.str();
        out.close();
    }

};


#endif //__PLANNER_H__
