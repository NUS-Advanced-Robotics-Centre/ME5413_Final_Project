/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Stephan Kurzawe
 *********************************************************************/
#ifndef COSTMAP_PROHIBITION_LAYER_H_
#define COSTMAP_PROHIBITION_LAYER_H_

#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <tf/transform_datatypes.h>
#include <mutex>
#include <geometry_msgs/PoseArray.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_prohibition_layer/CostmapProhibitionLayerConfig.h>
#include <dynamic_reconfigure/server.h>

#include <unordered_map>

namespace costmap_prohibition_layer_namespace
{
    
// point with integer coordinates  
struct PointInt
{
    int x;
    int y;
};
    
class CostmapProhibitionLayer : public costmap_2d::Layer
{
public:
    
  /**
   * default constructor
   */
  CostmapProhibitionLayer();

  /**
   * destructor
   */
  virtual ~CostmapProhibitionLayer();
  /**
   * function which get called at initializing the costmap
   * define the reconfige callback, get the reoslution
   * and read the prohibitions from the ros-parameter server
   */
  virtual void onInitialize();

  /**
   * This is called by the LayeredCostmap to poll this plugin
   * as to how much of the costmap it needs to update.
   * Each layer can increase the size of this bounds. 
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                            double *min_x, double *min_y, double *max_x, double *max_y);
  
  /**
   * function which get called at every cost updating procdure
   * of the overlayed costmap. The before readed costs will get
   * filled
   */
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                           int max_i, int max_j);

private:
    
  /**
   * overlayed reconfigure callback function
   */
  void reconfigureCB(CostmapProhibitionLayerConfig& config, uint32_t level);

  /**
   * Compute bounds in world coordinates for the current set of points and polygons.
   * The result is stored in class members _min_x, _min_y, _max_x and _max_y.
   */ 
  void computeMapBounds();
  
  /**
   * Set cost in a Costmap2D for a polygon (polygon may be located outside bounds)
   * 
   * @param master_grid    reference to the Costmap2D object
   * @param polygon        polygon defined by a vector of points (in world coordinates)
   * @param cost           the cost value to be set (0,255)
   * @param min_i          minimum bound on the horizontal map index/coordinate
   * @param min_j          minimum bound on the vertical map index/coordinate
   * @param max_i          maximum bound on the horizontal map index/coordinate
   * @param max_j          maximum bound on the vertical map index/coordinate
   * @param fill_polygon   if true, tue cost for the interior of the polygon will be set as well    
   */
  void setPolygonCost(costmap_2d::Costmap2D &master_grid, const std::vector<geometry_msgs::Point>& polygon,
                      unsigned char cost, int min_i, int min_j, int max_i, int max_j, bool fill_polygon);
  
  /**
   * Convert polygon (in map coordinates) to a set of cells in the map
   * 
   * @remarks This method is mainly based on Costmap2D::convexFillCells() but accounts
   *          for a self-implemented polygonOutlineCells() method and allows negative map coordinates
   * 
   * @param polygon             polygon defined  by a vector of map coordinates
   * @param[out] polygon_cells  new cells in map coordinates are pushed back on this container
   * @param fill                if true, the interior of the polygon will be considered as well
   */
  void rasterizePolygon(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells, bool fill);
  
  /**
   * Extract the boundary of a polygon in terms of map cells
   * 
   * @remarks This method is based on Costmap2D::polygonOutlineCells() but accounts
   *          for a self-implemented raytrace algorithm and allows negative map coordinates
   * 
   * @param polygon             polygon defined  by a vector of map coordinates
   * @param[out] polygon_cells  new cells in map coordinates are pushed back on this container
   */
  void polygonOutlineCells(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells);
  
  /**
   * Rasterize line between two map coordinates into a set of cells
   * 
   * @remarks Since Costmap2D::raytraceLine() is based on the size_x and since we want to rasterize 
   *          polygons that might also be located outside map bounds we provide a modified raytrace
   *          implementation (also Bresenham) based on the integer version presented here:
   *          http://playtechs.blogspot.de/2007/03/raytracing-on-grid.html
   * 
   * @param x0          line start x-coordinate (map frame)
   * @param y0          line start y-coordinate (map frame)
   * @param x1          line end x-coordinate (map frame)
   * @param y1          line end y-coordinate (map frame)
   * @param[out] cells  new cells in map coordinates are pushed back on this container
   */
  void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells);

  
  /**
   * read the prohibition areas in YAML-Format from the
   * ROS parameter server in the namespace of this
   * plugin
   * e.g. /move_base/global_costmap/prohibition_layer/param
   *
   * @param nhandle     pointer to the ros-Node handle
   * @param param       name of the parameter where the
   *                    prohibition areas saved in YAML format
   *
   * @return bool       true if the parsing was successful
   *                    false if it wasn't
    */
  bool parseProhibitionListFromYaml(ros::NodeHandle* nhandle, const std::string& param);

  /**
 * get a geometry_msgs::Point from a YAML-Array
 * The z-coordinate get always written to zero!
 *
 * @param val         YAML-array with to point-coordinates (x and y)
 * @param point       variable where the determined point get saved
 *
 * @return bool       true if the determining was successful
 *                    false if it wasn't
  */
  bool getPoint(XmlRpc::XmlRpcValue& val, geometry_msgs::Point& point);

  dynamic_reconfigure::Server<CostmapProhibitionLayerConfig>* _dsrv;            //!< dynamic_reconfigure server for the costmap
  std::mutex _data_mutex;                                                       //!< mutex for the accessing _prohibition_points and _prohibition_polygons
  double _costmap_resolution;                                                   //!< resolution of the overlayed costmap to create the thinnest line out of two points
  bool _fill_polygons;                                                          //!< if true, all cells that are located in the interior of polygons are marked as obstacle as well
  std::vector<geometry_msgs::Point> _prohibition_points;                        //!< vector to save the lonely points in source coordinates
  std::vector<std::vector<geometry_msgs::Point>> _prohibition_polygons;         //!< vector to save the polygons (including lines) in source coordinates
  double _min_x, _min_y, _max_x, _max_y;                                        //!< cached map bounds
};
}
#endif
