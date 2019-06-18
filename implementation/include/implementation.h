#ifndef __IMPLEMENTATION_H__
#define __IMPLEMENTATION_H__



namespace implementation {
    void writeResult(std::vector<std::pair<int, int>> result);
    void findShortestPath(std::vector<std::pair<int, int>>& result, std::vector<int>& performance, const std::vector<uint8_t>& overrides, const std::vector<uint8_t>& elevation, int x1, int y1, int x2, int y2, int x3, int y3);
}

#endif // __IMPLEMENTATION_H__