#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <memory>
#include <cmath>

#include "cell_occupancy_estimator.h"
#include "grid_cell_factory.h"
#include "../geometry_utils.h"

class GridMap {
public:
  using Cell = std::shared_ptr<GridCell>;
public:
  // TODO: cp, mv ctors, dtor
  GridMap(std::shared_ptr<GridCellFactory> cell_factory):
    // TODO: replace hardcoded value with params
    _width(2), _height(2), _m_per_cell(0.1),
    _cell_factory(cell_factory), _cells(_height) {
    for (auto &row : _cells) {
      for (int i = 0; i < _width; i++) {
        row.push_back(cell_factory->create_cell());
      }
    }
  }

  int width() const { return _width; }
  int height() const { return _height; }
  double scale() const { return _m_per_cell; }

  const std::vector<std::vector<Cell>> cells() const { return _cells; }

  void update_cell(const DiscretePoint2D& cell_coord,
                   const Occupancy &new_value, double quality = 1.0) {
    // TODO: bounds check
    _cells[cell_coord.y][cell_coord.x]->set_value(new_value, quality);
  }

  double cell_value(const DiscretePoint2D& cell_coord) const {
    return _cells[cell_coord.y][cell_coord.x]->value();
  }

  const Cell &cell(const DiscretePoint2D& cell_coord) const {
    return _cells[cell_coord.y][cell_coord.x];
  }

  const Cell &cell(int x, int y) const {
    return _cells[x][y];
  }

  Cell &cell(const DiscretePoint2D& cell_coord) {
    return _cells[cell_coord.y][cell_coord.x];
  }

  bool has_cell(const DiscretePoint2D& cell_coord) const {
    return 0 <= cell_coord.x && cell_coord.x < _width &&
           0 <= cell_coord.y && cell_coord.y < _height;
  }

  /*!
   * Function gets the number of a cell in the grid map where point \f$(x, y)\f$ locates.
   * It calculates the coordinates in grid map using information about width and height (count in cells) and
   * scale information (m/cell). If the cell coordinates are out of range in the map, so map is resized to become
   * bigger and then recalculate the cell coordinates.
   *
   * NOTE: this is the only function to get location in grid (count in cells) map
   * from location in world (count in meters) map.Do not try to manipulate with map directly inputing
   * "DiscretePoint2D" object. Create it by using this method.
   */
  DiscretePoint2D world_to_cell(double x, double y){
	//calculate location in grid map
	int cell_x = std::floor(_width /2 + x/_m_per_cell);
    int cell_y = std::floor(_height/2 + y/_m_per_cell);

    //find if it needs to resize map
	int width_added =  (cell_x<0 ? -cell_x : (cell_x>=width() ? cell_x-width()+1 : 0));
	int height_added = (cell_y<0 ? -cell_y : (cell_y>=height()? cell_y-height()+1 : 0));

	//resize with the rule: then farer new point locates then more "k" in *k*100% from map size
	// states are 200%, 400%, 800% etc
	int added_from_one_side_width  = std::ceil(std::log((double)width_added/width()  + 1)/std::log(2.0));
	int added_from_one_side_height = std::ceil(std::log((double)height_added/height() + 1)/std::log(2.0));
	resize_map_for_width(std::floor(2*added_from_one_side_width*width()));
	resize_map_for_height(std::floor(2*added_from_one_side_height*height()));

	//recalculate the cell coordinates after map resizing
	cell_x = std::floor(_width /2 + x/_m_per_cell);
	cell_y = std::floor(_height/2 + y/_m_per_cell);

    return DiscretePoint2D(cell_x, cell_y);
  }

  double cell_scale() const { return _m_per_cell; }

  Rectangle world_cell_bounds(const DiscretePoint2D &cell_coord) {
    Rectangle bounds;
    bounds.bot = (cell_coord.y + _height/2) * _m_per_cell;
    bounds.top = bounds.bot + _m_per_cell;
    bounds.left = (cell_coord.x + _width/2) * _m_per_cell;
    bounds.right = bounds.left + _m_per_cell;
    return bounds;
  }

  /*!
   * Function to change the height of map.
   * If new height is bigger than current height then new rows of empty cells will be added to map
   * (one half to the beginning and another to the end). If new height is smaller, then the half of difference
   * of rows in old and new map will be removed from the beginning of the map and another half will be removed from the end.
   * If value of new_height is odd, then extra row will be added (removed) from the beginning.
   * \param[in] new_height - the new value of height for grid map
   */
  void resize_map_for_height(const int new_height){
	if(new_height <= 0)
      return;
    //If it is needed to make map bigger
    if(new_height > height()){

      int amount_rows_add       = new_height-height(); //amount of rows to add to the map
      int amount_rows_add_begin = ((amount_rows_add%2)? amount_rows_add/2 +1 : amount_rows_add/2 ); //amount of rows to add a the beginning of row vector (some problems will be with odd value of new_height)
      int amount_rows_add_end   = amount_rows_add/2; //amount of rows to add a the ending of row vector

      //create matrix with amount of rows equal the amount of added rows
      std::vector<std::vector<Cell>> new_cells(amount_rows_add);

      //push all values at the first rows with empty cells
      for(int i = 0; i < amount_rows_add_begin; i++){
        for(int j = 0; j < width(); j++){
          new_cells.at(i).push_back(_cell_factory->create_cell());
        }
      }
      //insert the cells located on map before to new map
      new_cells.insert(new_cells.begin()+amount_rows_add_begin, _cells.begin(), _cells.end());

      //push all values at the and rows with empty cells
      for(int i = 0; i < amount_rows_add_end; i++){
        for(int j = 0; j < width(); j++){
          new_cells.at(new_height-i-1).push_back(_cell_factory->create_cell());
        }
      }
      _cells = new_cells;
      _height = new_height;
      return;
    }
    //if there is no need to resize
    if(new_height == height())
      return;

    //if it is needed to make map smaller

    int amount_rows_remove       = height() - new_height;
    int amount_rows_remove_begin = ((amount_rows_remove%2)? amount_rows_remove/2 +1 : amount_rows_remove/2 );
    int amount_rows_remove_end   = amount_rows_remove/2;

    std::vector<std::vector<Cell>> new_cells;
    //insert in empty vector rows from _cells, but only those rows which are in the vectors middle.
    new_cells.insert(new_cells.end(),_cells.begin()+amount_rows_remove_begin, _cells.end()-amount_rows_remove_end);
    _cells = new_cells;
    _height = new_height;
    return;
  }

  /*!
   * Function to change the width of map.
   * If new width is bigger than current height then new rows of empty cells will be added to map
   * (one half to the beginning and another to the end). If new width is smaller, then the half of difference
   * of columns in old and new map will be removed from the beginning of the map and another half will be removed from the end.
   * If value of new_width is odd, then extra column will be added (removed) from the beginning.
   * \param[in] new_width - the new value of width for grid map
   */
  void resize_map_for_width(const int new_width){
	if(new_width <= 0)
      return;
    //If it is needed to make map bigger
    if(new_width > width()){
      int amount_cols_add       = new_width-width(); //amount of columns to add to the map
      int amount_cols_add_begin = ((amount_cols_add%2)? amount_cols_add/2 +1 : amount_cols_add/2 ); //amount of columns to add a the beginning of column vector (some problems will be with odd value of new_width)
      int amount_cols_add_end   = amount_cols_add/2; //amount of rows to add a the ending of columns vector

      //for every row in _cells
      for(std::vector<Cell> &row: _cells){

        //insert empty cells at the beginning
        for(int i = 0; i < amount_cols_add_begin; i++){
          row.insert(row.begin(),_cell_factory->create_cell());
        }

        //insert empty cells at the ending
        for(int i = 0; i < amount_cols_add_end; i++){
          row.insert(row.end(),_cell_factory->create_cell());
        }
      }
      _width = new_width;
      return;
    }
    //if there is no need to resize
    if(new_width == width()){
    	return;
    }
    //if it is needed to make map smaller

    int amount_cols_remove       = width() - new_width;
    int amount_cols_remove_begin = ((amount_cols_remove%2)? amount_cols_remove/2 +1 : amount_cols_remove/2 );
    int amount_cols_remove_end   = amount_cols_remove/2;

    //for every row in the map
    for(std::vector<Cell> &row: _cells){
    	//erase first and last values
    	row.erase(row.begin(),row.begin()+amount_cols_remove_begin);
    	row.erase(row.end()-amount_cols_remove_end,row.end());
    }
    _width = new_width;
    return;
  }

private: // fields
  int _width, _height;
  double _m_per_cell;
  std::shared_ptr<GridCellFactory> _cell_factory;
  std::vector<std::vector<Cell>> _cells;
};

#endif
