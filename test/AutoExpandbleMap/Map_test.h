#ifndef _MAP_TEST_H
#define _MAP_TEST_H

#include "../../src/core/maps/grid_map.h"
#include "../../src/tiny_slam/tiny_grid_cells.h"

struct HeWi {
  int height;
  int width;
  int x;
  int y;
};

GridMap map(std::shared_ptr<GridCellFactory>(new TinyBaseCellFactory));

HeWi AutoExpandbleMap_Test(const DiscretePoint2D& Cell_coor) {
  HeWi param;  

  map.update_cell(Cell_coor, {0.5, 0.5});
  param.height = map.height();
  param.width  = map.width();
  param.x      = map.map_center_x();
  param.y      = map.map_center_y();
  
  return param;
}

bool CellValue_test() {
  int x0 = map.map_center_x() - 1  - map.width();
  int y0 = map.map_center_y() - 1 - map.height();
  int i = 0, j = 0;

  if (map.map_center_x() == 0)
    x0 = 0;
  if (map.map_center_y() == 0)
    y0 = 0;

  for (i = x0; i < map.width(); ++i)
    for (j = y0; j < map.height(); ++j)
      if (map.cell_value({i, j}) != 0.5)
        return false;
  return true;
}

#endif
