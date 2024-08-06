#ifndef CalculatePolygonArea_H
#define CalculatePolygonArea_H
#include<iostream>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <utility>
#include </home/m11115061/boost_1_85_0/boost/geometry.hpp>
#include </home/m11115061/boost_1_85_0/boost/geometry/geometries/polygon.hpp>
#include </home/m11115061/boost_1_85_0/boost/geometry/geometries/adapted/boost_tuple.hpp>
#include </home/m11115061/boost_1_85_0/boost/core/no_exceptions_support.hpp>
#include </home/m11115061/boost_1_85_0/boost/assign.hpp>
BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
using boost::assign::tuple_list_of;
typedef boost::geometry::model::polygon<boost::tuple<int,int>> Polygon_g;

//Process the input file that contain the tuple info of each net
void Inputfile_CalculateAreaofNet(std::ifstream& file,std::vector<Polygon_g> &polygons);

//Calculate the area of each net and store the result in area_resul
void CalculateAreaofNet(std::vector<Polygon_g> &polygons,std::vector<double> &area_result);

//Output the result to CSV file each area*1e-8 because the resolution is 1e-4, so the area is 1e-8
void Outputfile_AreaResult_toCSV(std::vector<Polygon_g> &polygons);

#endif
