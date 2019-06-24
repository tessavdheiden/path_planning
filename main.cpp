#include "visualizer.h"
#include "implementation.h"
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <iostream>
#include <thread>


#ifdef _MSC_VER
static const char* PATH_SEP = "\\";
#else
static const char* PATH_SEP = "/";
#endif

// Bits used in the overrides image bytes
enum OverrideFlags
{
    OF_RIVER_MARSH = 0x10,
    OF_INLAND = 0x20,
    OF_WATER_BASIN = 0x40
};

// Some constants
enum {
    IMAGE_DIM = 2048, // Width and height of the elevation and overrides image

    ROVER_X = 159,
    ROVER_Y = 1520,
    BACHELOR_X = 1303,
    BACHELOR_Y = 85,
    WEDDING_X = 1577,
    WEDDING_Y = 1294
};

std::ifstream::pos_type fileSize(const std::string& filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in.good())
    {
        throw std::exception();
    }
    return in.tellg();
}

std::vector<uint8_t> loadFile(const std::string& filename, size_t expectedFileSize)
{
    size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    std::vector<uint8_t> data(fsize);
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)&data[0], fsize);
    return data;
}

bool donut(int x, int y, int x1, int y1)
{
    int dx = x - x1;
    int dy = y - y1;
    int r2 = dx * dx + dy * dy;
    return r2 >= 150 && r2 <= 400;
}

std::pair<std::pair<int, int>, std::pair<int, int>> make_query(int x1, int y1, int x2, int y2){ return std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2));}

void merge_result(std::vector<std::pair<int, int>> &p1, std::vector<std::pair<int, int>> &p2) {
    p1.insert(p1.end(), p2.begin(), p2.end());
}


static bool MULTI_THREAD = false;

int main(int argc, char** argv) {

    const size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
    // Address assets relative to application location
    std::string anchor = std::string(".") + PATH_SEP;
    std::string pname = argv[0];
    auto lastpos = pname.find_last_of("/\\");
    if (lastpos != std::string::npos) {
        anchor = pname.substr(0, lastpos) + PATH_SEP;
    }
    auto elevation = loadFile(anchor + "assets" + PATH_SEP + "elevation.data", expectedFileSize);
    auto overrides = loadFile(anchor + "assets" + PATH_SEP + "overrides.data", expectedFileSize);

    std::vector<std::pair<int, int>> result1;
    std::pair<std::pair<int, int>, std::pair<int, int>> query1 = make_query(ROVER_X, ROVER_Y, BACHELOR_X, BACHELOR_Y);
    std::vector<std::pair<int, int>> result2;
    std::pair<std::pair<int, int>, std::pair<int, int>> query2 = make_query(BACHELOR_X, BACHELOR_Y, WEDDING_X, WEDDING_Y);

    implementation::Rover rover = implementation::BIG;
    implementation::Grid* grid = implementation::make_grid(overrides, elevation, rover);

    std::string filepath = "../results/a_star_path_big_rover.txt";


    auto t_start = std::chrono::high_resolution_clock::now();
    if (MULTI_THREAD){
        std::thread first(implementation::findShortestPath, implementation::print_coordinate, implementation::a_star_search<implementation::GridLocation, implementation::Grid>, std::ref(result1), std::ref(grid), std::ref(query1));
        std::thread second(implementation::findShortestPath, implementation::print_coordinate, implementation::a_star_search<implementation::GridLocation, implementation::Grid>, std::ref(result2), std::ref(grid), std::ref(query2));
        first.join();
        second.join();
        std::cout << "hello world";
    }
    else{
        //void (* func_ptr)(int) = &(implementation::search<int>);
        //func_ptr(1);

        //SearchFunctor a_star_search_functor(dikstra, graph,
          //                                  start,
            //                                goal,
              //                              &came_from,
                //                            &cost_so_far);


        implementation::findShortestPath(implementation::print_coordinate, implementation::a_star_search<implementation::GridLocation, implementation::Grid>, result1, grid, query1);
        //implementation::findShortestPath2(implementation::print_coordinate, func_ptr,  result2, grid, query2);
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration<double, std::milli>(t_end - t_start).count();

    merge_result(result1, result2);

    implementation::writeResult(filepath, result1);
    std::ofstream of("../results/pic.bmp", std::ofstream::binary);

    visualizer::writeBMP(
            of,
            &elevation[0],
            IMAGE_DIM,
            IMAGE_DIM,
            [&](size_t x, size_t y, uint8_t elevation) {

                // Marks interesting positions on the map
                if (donut(x, y, ROVER_X, ROVER_Y) ||
                    donut(x, y, BACHELOR_X, BACHELOR_Y) ||
                    donut(x, y, WEDDING_X, WEDDING_Y)) {
                    return uint8_t(visualizer::IPV_PATH);
                }

                // Signifies water
                if ((overrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
                    elevation == 0) {
                    return uint8_t(visualizer::IPV_WATER);
                }

                // Signifies normal ground color
                if (elevation < visualizer::IPV_ELEVATION_BEGIN) {
                    elevation = visualizer::IPV_ELEVATION_BEGIN;
                }
                return elevation;
            });
    of.flush();
#if __APPLE__
    auto res = system("open pic.bmp");
    (void)res;
#endif
    return 0;

}



