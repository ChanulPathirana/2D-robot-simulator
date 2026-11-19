#include "Mapping.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>

using namespace std;

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw runtime_error(message);
}

void near(double actual, double expected, const char* message, double tolerance = 1e-10) {
    require(abs(actual - expected) <= tolerance, message);
}

void ray(Mapping& map, double angle, double distance, bool hit,
         double x = 0.5, double y = 0.5) {
    map.update({{angle, distance, hit}}, x, y);
}

Mapping smallMap() { return Mapping(0.0, 0.0, 5.0, 5.0, 1.0); }

void testInitialAndReset() {
    Mapping map = smallMap();
    for (int y = 0; y < map.getRows(); ++y) {
        for (int x = 0; x < map.getCols(); ++x) {
            require(map.getCell(x, y) == CellState::Unknown, "new cell is not unknown");
            near(map.getOccupancyProbability(x, y), 0.5, "new probability is not 0.5");
        }
    }
    require(map.getCell(-1, 0) == CellState::Unknown, "out-of-bounds state");
    near(map.getOccupancyProbability(5, 0), 0.5, "out-of-bounds probability");
    ray(map, 0.0, 2.0, true);
    map.reset();
    for (int y = 0; y < map.getRows(); ++y) {
        for (int x = 0; x < map.getCols(); ++x) {
            require(map.getCell(x, y) == CellState::Unknown, "reset state");
            near(map.getOccupancyProbability(x, y), 0.5, "reset probability");
        }
    }
}

void testRepeatedAndConflictingEvidence() {
    Mapping map = smallMap();
    ray(map, 0.0, 2.0, true);
    near(map.getOccupancyProbability(2, 0), 0.70, "one hit probability");
    require(map.getCell(2, 0) == CellState::Occupied, "one hit classification");
    const double once = map.getOccupancyProbability(2, 0);
    ray(map, 0.0, 2.0, true);
    require(map.getOccupancyProbability(2, 0) > once, "repeated hits did not strengthen evidence");

    for (int i = 0; i < 100; ++i) ray(map, 0.0, 2.0, true);
    const double maximum = 1.0 / (1.0 + exp(-4.0));
    near(map.getOccupancyProbability(2, 0), maximum, "upper log-odds clamp");
    ray(map, 0.0, 3.0, false);
    require(map.getCell(2, 0) == CellState::Occupied,
            "one clear ray erased strong occupied evidence");
    for (int i = 0; i < 100; ++i) ray(map, 0.0, 3.0, false);
    const double minimum = 1.0 / (1.0 + exp(4.0));
    near(map.getOccupancyProbability(2, 0), minimum, "lower log-odds clamp");
    require(map.getCell(2, 0) == CellState::Free,
            "later clear observations did not remove a temporary obstacle");

    Mapping balanced = smallMap();
    ray(balanced, 0.0, 2.0, true);
    ray(balanced, 0.0, 3.0, false);
    near(balanced.getOccupancyProbability(2, 0), 0.5,
         "conflicting evidence did not balance");
    require(balanced.getCell(2, 0) == CellState::Unknown,
            "inconclusive observed cell should classify unknown");
}

void testRayTraversal() {
    Mapping hit = smallMap();
    ray(hit, 0.0, 2.0, true);
    require(hit.getCell(0, 0) == CellState::Free, "start cell not free");
    require(hit.getCell(1, 0) == CellState::Free, "preceding cell not free");
    require(hit.getCell(2, 0) == CellState::Occupied, "endpoint not occupied");
    require(hit.getCell(3, 0) == CellState::Unknown, "cell behind hit was updated");

    Mapping noHit = smallMap();
    ray(noHit, 0.0, 2.0, false);
    require(noHit.getCell(2, 0) == CellState::Free, "no-hit range not free");
    require(noHit.getCell(3, 0) == CellState::Unknown,
            "no-hit ray created evidence beyond its range");

    Mapping exactPositive = smallMap();
    ray(exactPositive, 0.0, 1.5, true);
    require(exactPositive.getCell(1, 0) == CellState::Free,
            "positive boundary preceding cell");
    require(exactPositive.getCell(2, 0) == CellState::Occupied,
            "positive boundary endpoint ownership");

    Mapping exactNegative = smallMap();
    ray(exactNegative, numbers::pi, 1.5, true, 3.5, 0.5);
    require(exactNegative.getCell(2, 0) == CellState::Free,
            "negative boundary preceding cell");
    require(exactNegative.getCell(1, 0) == CellState::Occupied,
            "negative boundary endpoint ownership");

    Mapping originBoundary = smallMap();
    ray(originBoundary, numbers::pi, 0.5, true, 2.0, 0.5);
    require(originBoundary.getCell(1, 0) == CellState::Occupied,
            "negative ray from grid boundary started in wrong cell");
    require(originBoundary.getCell(2, 0) == CellState::Unknown,
            "grid-boundary origin marked cell behind the ray");

    Mapping noHitBoundary = smallMap();
    ray(noHitBoundary, 0.0, 1.5, false);
    require(noHitBoundary.getCell(1, 0) == CellState::Free,
            "no-hit boundary preceding cell");
    require(noHitBoundary.getCell(2, 0) == CellState::Unknown,
            "no-hit boundary advanced past range");

    Mapping diagonal = smallMap();
    ray(diagonal, numbers::pi / 4.0, sqrt(4.5), true);
    require(diagonal.getCell(0, 0) == CellState::Free, "diagonal start cell");
    require(diagonal.getCell(1, 1) == CellState::Free, "diagonal crossing");
    require(diagonal.getCell(2, 2) == CellState::Occupied,
            "exact grid-corner endpoint");
    require(diagonal.getCell(1, 0) == CellState::Unknown,
            "diagonal corner updated side cell");
    require(diagonal.getCell(0, 1) == CellState::Unknown,
            "diagonal corner updated other side cell");

    Mapping diagonalNegative = smallMap();
    ray(diagonalNegative, 5.0 * numbers::pi / 4.0, sqrt(4.5), true, 3.5, 3.5);
    require(diagonalNegative.getCell(2, 2) == CellState::Free,
            "negative diagonal preceding cell");
    require(diagonalNegative.getCell(1, 1) == CellState::Occupied,
            "negative diagonal grid-corner endpoint");
    require(diagonalNegative.getCell(2, 3) == CellState::Unknown,
            "negative diagonal updated side cell");

    Mapping noHitDiagonal = smallMap();
    ray(noHitDiagonal, numbers::pi / 4.0, sqrt(4.5), false);
    require(noHitDiagonal.getCell(1, 1) == CellState::Free,
            "diagonal no-hit preceding cell");
    require(noHitDiagonal.getCell(2, 2) == CellState::Unknown,
            "diagonal no-hit advanced past exact range");

    Mapping vertical = smallMap();
    ray(vertical, numbers::pi / 2.0, 2.0, true, 1.5, 0.5);
    require(vertical.getCell(1, 2) == CellState::Occupied, "near-zero X direction");
    require(vertical.getCell(2, 1) == CellState::Unknown,
            "near-zero X direction drifted sideways");
}

void testInvalidInputsAndConfiguration() {
    Mapping map = smallMap();
    const double nan = numeric_limits<double>::quiet_NaN();
    const double infinity = numeric_limits<double>::infinity();
    ray(map, nan, 1.0, true);
    ray(map, 0.0, nan, true);
    ray(map, 0.0, infinity, true);
    ray(map, 0.0, 0.0, true);
    ray(map, 0.0, -1.0, true);
    ray(map, 0.0, 1.0, true, -1.0, 0.5);
    ray(map, 0.0, 1.0, true, nan, 0.5);
    ray(map, 0.0, 1.0, true, 0.5, infinity);
    for (int y = 0; y < map.getRows(); ++y) {
        for (int x = 0; x < map.getCols(); ++x) {
            require(map.getCell(x, y) == CellState::Unknown, "invalid scan changed map");
        }
    }

    MappingParameters settings;
    settings.occupiedObservationProbability = 0.8;
    settings.freeObservationProbability = 0.2;
    settings.minLogOdds = -2.0;
    settings.maxLogOdds = 2.0;
    settings.occupiedThreshold = 0.7;
    settings.freeThreshold = 0.3;
    Mapping tuned(0.0, 0.0, 5.0, 5.0, 1.0, settings);
    ray(tuned, 0.0, 2.0, true);
    near(tuned.getOccupancyProbability(2, 0), 0.8, "custom observation probability");
    for (int i = 0; i < 100; ++i) ray(tuned, 0.0, 2.0, true);
    near(tuned.getOccupancyProbability(2, 0), 1.0 / (1.0 + exp(-2.0)),
         "custom log-odds bound");
}

} // namespace

int main() {
    try {
        testInitialAndReset();
        testRepeatedAndConflictingEvidence();
        testRayTraversal();
        testInvalidInputsAndConfiguration();
        cout << "Mapping tests passed\n";
    } catch (const exception& error) {
        cerr << "Mapping test failed: " << error.what() << '\n';
        return 1;
    }
}
