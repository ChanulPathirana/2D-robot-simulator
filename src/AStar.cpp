
#include "AStar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

using namespace std;

namespace {
 AStar::Timing timingData;
 struct TimingScope {
    std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    ~TimingScope() {
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        ++timingData.calls;
        timingData.totalMs += ms;
        timingData.maxMs = std::max(timingData.maxMs, ms);
    }
 };

double heuristic(GridCell a, GridCell b) {
    // Octile distance for 8-direction movement.
    int dx = abs(a.x - b.x);
    int dy = abs(a.y - b.y);

    return max(dx, dy) +
           (sqrt(2.0) - 1.0) * min(dx, dy);
}

struct Node {
    GridCell cell;
    double f;

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

} // namespace

AStar::Timing AStar::timing() { return timingData; }
void AStar::resetTiming() { timingData = {}; }

vector<GridCell> AStar::findPath(
    const NavigationGrid& grid,
    GridCell start,
    GridCell goal
) {
    TimingScope timingScope;
    int cols = grid.getCols();
    int rows = grid.getRows();

    if (grid.isBlocked(start.x, start.y) ||
        grid.isBlocked(goal.x, goal.y)) {
        return {};
    }

    auto index = [cols](GridCell c) {
        return c.y * cols + c.x;
    };

    int total = rows * cols;

    vector<double> gScore(
        total,
        numeric_limits<double>::infinity()
    );

    vector<int> parent(total, -1);

    priority_queue<
        Node,
        vector<Node>,
        greater<Node>
    > open;

    gScore[index(start)] = 0.0;

    open.push({
        start,
        heuristic(start, goal)
    });

    const int directions[8][2] = {
        { 1,  0}, {-1,  0},
        { 0,  1}, { 0, -1},
        { 1,  1}, { 1, -1},
        {-1,  1}, {-1, -1}
    };

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        int currentIndex = index(current.cell);

        // Ignore outdated queue entries.
        if (current.f >
            gScore[currentIndex] +
            heuristic(current.cell, goal) + 1e-9) {
            continue;
        }

        if (current.cell.x == goal.x &&
            current.cell.y == goal.y) {

            vector<GridCell> path;

            int at = currentIndex;

            while (at != -1) {
                path.push_back({
                    at % cols,
                    at / cols
                });

                at = parent[at];
            }

            reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& dir : directions) {

            int nx = current.cell.x + dir[0];
            int ny = current.cell.y + dir[1];

            if (grid.isBlocked(nx, ny)) {
                continue;
            }

            // Prevent diagonal corner cutting.
            if (dir[0] != 0 && dir[1] != 0) {
                if (grid.isBlocked(
                        current.cell.x + dir[0],
                        current.cell.y
                    ) ||
                    grid.isBlocked(
                        current.cell.x,
                        current.cell.y + dir[1]
                    )) {
                    continue;
                }
            }

            GridCell next{nx, ny};

            double moveCost =
                (dir[0] != 0 && dir[1] != 0)
                ? sqrt(2.0)
                : 1.0;

            double tentative =
                gScore[currentIndex] + moveCost;

            int nextIndex = index(next);

            if (tentative < gScore[nextIndex]) {
                gScore[nextIndex] = tentative;
                parent[nextIndex] = currentIndex;

                open.push({
                    next,
                    tentative + heuristic(next, goal)
                });
            }
        }
    }

    // No path exists.
    return {};
}
