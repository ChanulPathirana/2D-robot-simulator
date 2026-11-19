#pragma once

struct GridCell {
    int x;
    int y;
};

class NavigationGrid {
public:
    virtual ~NavigationGrid() = default;

    virtual bool isBlocked(int x, int y) const = 0;
    virtual GridCell worldToGrid(double x, double y) const = 0;
    virtual double gridToWorldX(int x) const = 0;
    virtual double gridToWorldY(int y) const = 0;
    virtual int getRows() const = 0;
    virtual int getCols() const = 0;
};
