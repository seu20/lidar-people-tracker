#pragma once
#include "Protocol.h"
#include <vector>
#include <cstdint>

enum class CellState : uint8_t { EMPTY, OCCUPIED };

struct Cell {
    int row = 0;
    int col = 0;
    Cell() = default;
    Cell(int r, int c) : row(r), col(c) {}
};

class Grid {
private:
    //셀마다의 거리
    int dist_per_cell;
    int cell_origin;
    
    // 행, 열 수
    int rows;
    int cols;

    // 좌표 이동 (위, 오, 아, 왼, 오위, 오아, 왼아, 왼위)
    static constexpr int dr[8] = {-1, 0, 1, 0, -1, 1, 1, -1};
    static constexpr int dc[8] = {0, 1, 0, -1, 1, 1, -1, -1};

    //격자
    std::vector<CellState> cells;
    // centroid의 모임
    std::vector<Point2D> clusters;

    bool isBound(int idx) const;
    Point2D SensorToPoint(const SensorPoint &sp);    //m
    Cell PointToCell(const Point2D& point) const;     // m -> cm 변환 후 origin 에 맞게 변환
    Point2D CellToPoint(const Cell& cell) const;
    Point2D CentroidCalculate(const Cell &sum_clusters, int num_clusters) const;
    Cell IndexToCell(int index) const;
    int CellToIndex(int row, int col) const;
public:
    Grid(int max_range, int cell_size_cm);
    ~Grid();
    void InsertPoints(const std::vector<SensorPoint> &points);
    std::vector<Point2D> Cluster();
    void Clear();
};

