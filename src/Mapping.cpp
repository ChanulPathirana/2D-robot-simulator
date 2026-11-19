#include "Mapping.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace std;

namespace {

double logit(double probability) {
    return log(probability) - log1p(-probability);
}

bool validProbability(double probability) {
    return isfinite(probability) && probability > 0.0 && probability < 1.0;
}

double snapGridBoundary(double coordinate) {
    if (!isfinite(coordinate)) return coordinate;
    const double nearest = nearbyint(coordinate);
    const double tolerance = 32.0 * numeric_limits<double>::epsilon()
        * max(1.0, abs(coordinate));
    return abs(coordinate - nearest) <= tolerance ? nearest : coordinate;
}

} // namespace

Mapping::Mapping(double minX, double minY, double maxX, double maxY,
                 double size, MappingParameters settings)
    : cellSize(size), originX(minX), originY(minY),
      maxX(maxX), maxY(maxY), parameters(settings)
{
    if (!isfinite(minX) || !isfinite(minY) ||
        !isfinite(maxX) || !isfinite(maxY) || !isfinite(size) ||
        maxX <= minX || maxY <= minY || size <= 0.0) {
        throw invalid_argument("Invalid map dimensions");
    }
    if (!validProbability(settings.occupiedObservationProbability) ||
        !validProbability(settings.freeObservationProbability) ||
        !validProbability(settings.occupiedThreshold) ||
        !validProbability(settings.freeThreshold) ||
        settings.freeObservationProbability >= 0.5 ||
        settings.occupiedObservationProbability <= 0.5 ||
        settings.freeThreshold >= 0.5 || settings.occupiedThreshold <= 0.5 ||
        !isfinite(settings.minLogOdds) || !isfinite(settings.maxLogOdds) ||
        settings.minLogOdds >= 0.0 || settings.maxLogOdds <= 0.0) {
        throw invalid_argument("Invalid mapping parameters");
    }

    const double columnCount = ceil((maxX - minX) / cellSize);
    const double rowCount = ceil((maxY - minY) / cellSize);
    if (!isfinite(columnCount) || !isfinite(rowCount) ||
        columnCount < 1.0 || rowCount < 1.0 ||
        columnCount > numeric_limits<int>::max() ||
        rowCount > numeric_limits<int>::max() ||
        columnCount * rowCount > numeric_limits<int>::max()) {
        throw invalid_argument("Map has too many cells");
    }
    cols = static_cast<int>(columnCount);
    rows = static_cast<int>(rowCount);

    occupiedIncrement = logit(settings.occupiedObservationProbability);
    freeIncrement = logit(settings.freeObservationProbability);
    occupiedThresholdLogOdds = logit(settings.occupiedThreshold);
    freeThresholdLogOdds = logit(settings.freeThreshold);

    logOdds.assign(static_cast<size_t>(rows) * cols, 0.0);
    observed.assign(logOdds.size(), 0);
}

CellState Mapping::getCell(int x, int y) const {
    if (!inBounds(x, y)) return CellState::Unknown;
    const size_t index = static_cast<size_t>(y) * cols + x;
    if (!observed[index]) return CellState::Unknown;
    if (logOdds[index] <= freeThresholdLogOdds) return CellState::Free;
    if (logOdds[index] >= occupiedThresholdLogOdds) return CellState::Occupied;
    return CellState::Unknown;
}

double Mapping::getOccupancyProbability(int x, int y) const {
    if (!inBounds(x, y)) return 0.5;
    const double value = logOdds[static_cast<size_t>(y) * cols + x];
    return 1.0 / (1.0 + exp(-value));
}

int Mapping::getRows() const { return rows; }
int Mapping::getCols() const { return cols; }
double Mapping::getCellSize() const { return cellSize; }
double Mapping::getMinX() const { return originX; }
double Mapping::getMinY() const { return originY; }
double Mapping::getMaxX() const { return maxX; }
double Mapping::getMaxY() const { return maxY; }

double Mapping::gridToWorldX(int x) const {
    return originX + (x + 0.5) * cellSize;
}

double Mapping::gridToWorldY(int y) const {
    return originY + (y + 0.5) * cellSize;
}

void Mapping::reset() {
    fill(logOdds.begin(), logOdds.end(), 0.0);
    fill(observed.begin(), observed.end(), 0);
}

bool Mapping::inBounds(int x, int y) const {
    return x >= 0 && x < cols && y >= 0 && y < rows;
}

void Mapping::addEvidence(int x, int y, double increment) {
    if (!inBounds(x, y)) return;
    const size_t index = static_cast<size_t>(y) * cols + x;
    logOdds[index] = clamp(logOdds[index] + increment,
                           parameters.minLogOdds, parameters.maxLogOdds);
    observed[index] = 1;
}

void Mapping::update(const vector<LidarReading>& readings,
                     double sensorX, double sensorY) {
    if (!isfinite(sensorX) || !isfinite(sensorY)) return;
    const double startX = (sensorX - originX) / cellSize;
    const double startY = (sensorY - originY) / cellSize;
    if (!(startX >= 0.0 && startX < cols &&
          startY >= 0.0 && startY < rows)) return;
    for (const auto& reading : readings) updateRay(reading, sensorX, sensorY);
}

void Mapping::updateRay(const LidarReading& reading,
                        double sensorX, double sensorY) {
    if (!isfinite(reading.angle) || !isfinite(reading.distance) ||
        reading.distance <= 0.0) return;

    // Treat numerical axis components below this scale as zero. A six-metre
    // ray then drifts by less than a trillionth of a metre on that axis.
    const double rawDx = cos(reading.angle);
    const double rawDy = sin(reading.angle);
    const double dx = abs(rawDx) < 1e-12 ? 0.0 : rawDx;
    const double dy = abs(rawDy) < 1e-12 ? 0.0 : rawDy;
    const int stepX = (dx > 0.0) - (dx < 0.0);
    const int stepY = (dy > 0.0) - (dy < 0.0);
    const double infinity = numeric_limits<double>::infinity();

    // A ray leaving an exact grid boundary in a negative direction starts
    // in the cell on that side, rather than briefly marking the other cell.
    double startGridX = snapGridBoundary((sensorX - originX) / cellSize);
    double startGridY = snapGridBoundary((sensorY - originY) / cellSize);
    if (stepX < 0) startGridX = nextafter(startGridX, -infinity);
    if (stepY < 0) startGridY = nextafter(startGridY, -infinity);
    int x = static_cast<int>(floor(startGridX));
    int y = static_cast<int>(floor(startGridY));
    if (!inBounds(x, y)) return;

    int hitCellX = -1, hitCellY = -1;
    if (reading.hit) {
        const double endX = fma(reading.distance, dx, sensorX);
        const double endY = fma(reading.distance, dy, sensorY);
        if (!isfinite(endX) || !isfinite(endY)) return;
        double endGridX = snapGridBoundary((endX - originX) / cellSize);
        double endGridY = snapGridBoundary((endY - originY) / cellSize);
        if (stepX != 0) endGridX = nextafter(endGridX, stepX > 0 ? infinity : -infinity);
        if (stepY != 0) endGridY = nextafter(endGridY, stepY > 0 ? infinity : -infinity);
        if (endGridX >= 0.0 && endGridX < cols &&
            endGridY >= 0.0 && endGridY < rows) {
            hitCellX = static_cast<int>(floor(endGridX));
            hitCellY = static_cast<int>(floor(endGridY));
        }
    }

    double tMaxX = infinity, tMaxY = infinity;
    double tDeltaX = infinity, tDeltaY = infinity;
    if (stepX != 0) {
        const double boundaryX = originX + (stepX > 0 ? x + 1 : x) * cellSize;
        tMaxX = max(0.0, (boundaryX - sensorX) / dx);
        tDeltaX = cellSize / abs(dx);
    }
    if (stepY != 0) {
        const double boundaryY = originY + (stepY > 0 ? y + 1 : y) * cellSize;
        tMaxY = max(0.0, (boundaryY - sensorY) / dy);
        tDeltaY = cellSize / abs(dy);
    }

    // DDA visits each crossed cell once. The endpoint is excluded from free
    // updates and receives occupied evidence once after traversal.
    const double endpointTolerance = 1e-12 * max(1.0, reading.distance);
    for (int steps = 0; inBounds(x, y) && steps < rows + cols + 2; ++steps) {
        if (reading.hit && x == hitCellX && y == hitCellY) break;
        addEvidence(x, y, freeIncrement);

        const double nextT = min(tMaxX, tMaxY);
        if (!isfinite(nextT) ||
            (reading.hit && nextT > reading.distance + endpointTolerance) ||
            (!reading.hit && nextT >= reading.distance - endpointTolerance)) break;

        const double cornerTolerance = 1e-12 * max(1.0, nextT);
        if (tMaxX <= nextT + cornerTolerance) {
            x += stepX;
            tMaxX += tDeltaX;
        }
        if (tMaxY <= nextT + cornerTolerance) {
            y += stepY;
            tMaxY += tDeltaY;
        }
    }
    if (reading.hit) addEvidence(hitCellX, hitCellY, occupiedIncrement);
}
