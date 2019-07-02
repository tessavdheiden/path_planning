#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <iostream>
#include <vector>
#include <chrono>
#include <string>
#include <sstream>
#include <fstream>
#include <thread>
#include "implementation.h"


std::pair<std::pair<int, int>, std::pair<int, int>> make_query(const int x1, const int y1, const int x2, const int y2) {
    return std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2));
};

class Planner {
public:
    Planner(){};
    ~Planner(){}

    void run(const std::vector<uint8_t> &overrides, const std::vector<uint8_t> &elevation, const std::vector<int>& vec) {
        set_data(overrides, elevation);
        set_queries(vec);
        auto start = std::chrono::high_resolution_clock::now();
        search();
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Time taken by algorithm: "
                  << duration.count() << " microseconds" << std::endl;
    }

    std::vector<std::pair<int, int>> get_result() {
        return result;
    }

private:
    std::shared_ptr<implementation::Grid> grid;
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> queries;
    std::vector<std::vector<std::pair<int, int>>> paths;
    std::vector<std::pair<int, int>> result;

    void set_data(const std::vector<uint8_t> &overrides, const std::vector<uint8_t> &elevation) {
        grid = implementation::make_grid(overrides, elevation);
    }

    void set_queries(std::vector<int> vec) {
        for (size_t i = 0; i < vec.size()-2; i+=2){
            queries.push_back(make_query(vec[i], vec[i + 1], vec[i + 2], vec[i + 3]));
        }
        paths = std::vector<std::vector<std::pair<int, int>>>(queries.size());
    }

    void search() {
        for (int i = 0; i < queries.size(); i++) {
            implementation::findShortestPath(implementation::print_coordinate,
                                             implementation::dijkstra_search<implementation::GridLocation, implementation::Grid>,
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

        for (auto location : result) {
            ss << location.first << " " << location.second << "\n";
        }

        out << ss.str();
        out.close();
    }

};


#endif //__PLANNER_H__
