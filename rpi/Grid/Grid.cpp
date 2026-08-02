#include "Grid.h"
#include <cmath>
#include <queue>
#include <algorithm>

// 생성자, 소멸자
Grid::Grid(int max_range_cm, int cell_size_cm)
{
    dist_per_cell = cell_size_cm;
    cell_origin   = (max_range_cm + cell_size_cm - 1) / cell_size_cm;  // 올림
    rows = cols   = cell_origin * 2;
    cells.assign(static_cast<size_t>(rows) * cols, CellState::EMPTY);
}
Grid::~Grid()
{
    cells.clear();
}

// Private 함수
bool Grid::isBound(int idx) const
{
    if (idx < 0 || idx >= rows * cols)
    {
        return false;
    }
    return true;
}

Point2D Grid::SensorToPoint(const SensorPoint& sp)
{
    Point2D p;
    p.x = sp.dist * std::cos(sp.angle);  // 단위: m
    p.y = sp.dist * std::sin(sp.angle);  // 단위: m
    return p;
}

Cell Grid::PointToCell(const Point2D &point) const
{
    Cell c;
    c.row = static_cast<int>((point.x * 100 / dist_per_cell) + cell_origin);  // m to cell
    c.col = static_cast<int>((point.y * 100 / dist_per_cell) + cell_origin);
    return c;
}

Point2D Grid::CellToPoint(const Cell& cell) const  // 객체의 셀 정보를 Point2D 구조로 반환
{
    Point2D p;
    p.x = static_cast<float>((cell.row - cell_origin) * dist_per_cell);
    p.y = static_cast<float>((cell.col - cell_origin) * dist_per_cell);
    return p;
}

Point2D Grid::CentroidCalculate(const Cell& sum_clusters, int num_clusters) const
{
    float avg_row = static_cast<float>(sum_clusters.row) / num_clusters;
    float avg_col = static_cast<float>(sum_clusters.col) / num_clusters;
    Point2D centroid;
    centroid.x = (avg_row - cell_origin) * dist_per_cell / 100.0f;  // CellToPoint와 동일 로직
    centroid.y = (avg_col - cell_origin) * dist_per_cell / 100.0f;
    return centroid;
}

Cell Grid::IndexToCell(int index) const {
    Cell cell;
    cell.row = index / cols;
    cell.col = index % cols;
    return cell;
}

int Grid::CellToIndex(int row, int col) const
{
    return row * cols + col;
}

// Public 함수

void Grid::InsertPoints(const std::vector<SensorPoint> &points)
{
    for (auto& s_point : points)
    {
        Cell c = PointToCell(SensorToPoint(s_point));
        if (c.row < 0 || c.row >= rows || c.col < 0 || c.col >= cols) continue;
        int idx = CellToIndex(c.row, c.col);
        if (!isBound(idx)) continue;

        if (cells[idx] == CellState::EMPTY)
        {
            cells[idx] = CellState::OCCUPIED;
        }
    }
}

std::vector<Point2D> Grid::Cluster()    //bfs를 통한 클러스터
{
    std::vector<uint8_t> visited(cells.size(), 0);
    std::vector<Point2D> centroids;
    centroids.reserve(20);  // 대략 예상되는 클러스터 개수만큼 미리 할당, reallocation 방지
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            if ((cells[CellToIndex(r, c)] == CellState::EMPTY) || visited[CellToIndex(r, c)]) continue;
            std::queue<int> q;
            visited[CellToIndex(r, c)] = 1;
            q.push(CellToIndex(r, c));
            int cell_count =  0;
            int sum_clusters_r = 0;
            int sum_clusters_c = 0;
            while(!q.empty())
            {
                int curr_index = q.front();
                cell_count++;
                Cell curr_cell = IndexToCell(curr_index);
                sum_clusters_r += curr_cell.row;
                sum_clusters_c += curr_cell.col;
                q.pop();
                for (int d = 0; d < 8; ++d)
                {
                    int next_row = curr_cell.row + dr[d];
                    int next_col = curr_cell.col + dc[d];
                    if (next_row < 0 || next_row >= rows || next_col < 0 || next_col >= cols) continue;
                    int next_index = CellToIndex(next_row, next_col);
                    if (isBound(next_index) &&
                        !visited[next_index] &&
                        cells[next_index] == CellState::OCCUPIED)
                    {
                        q.push(next_index);
                        visited[next_index] = 1;
                    }
                }
            }
            static constexpr int MIN_CLUSTER_CELLS = 6;
            if (cell_count < MIN_CLUSTER_CELLS) continue;   // MIN_CLUSTER_CELLS 를 못 넘으면 cluster로 판단 X

            centroids.push_back(CentroidCalculate(Cell(sum_clusters_r, sum_clusters_c), cell_count));
        }      
    }
    return centroids;
}

void Grid::Clear()
{
    cells.assign(rows * cols, CellState::EMPTY);
}
