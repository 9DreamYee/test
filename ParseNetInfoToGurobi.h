#ifndef COMMONBOUNDARY_H
#define COMMONBOUNDARY_H
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> point_t;
typedef bg::model::linestring<point_t> line_t;
typedef bg::model::polygon<point_t> polygon_t;

//struct宣告
struct netInfo{
    int netID;
    double pad_x,pad_y;
    double ball_x,ball_y;
    double areaInitial;
    point_t startPoint0;
    point_t startPoint1;
    std::vector<line_t> boundarySegments;
    std::vector<line_t> innerBoundarySegments;
    std::vector<line_t> outterBoundarySegments;
    line_t InitialRoute;
    netInfo();
};
struct commonBoundary{
    int boundaryID;
    int netA,netB;
    point_t startPoint;
    line_t boundarySegment;
    //若該邊界線為corner net則加入initialRouteSegment
    line_t initialRouteSegment;
    //每條邊界線平移後對多邊形面積的線性影響係數
    double alpha;
    point_t netA_pad,netB_pad;
    double shiftMin,shiftMax;
    // 0 -> x, 1 -> y , 2 -> corner net需特例MILP處理
    int shift_Direction;
    int shiftAmount;
    commonBoundary();
};
//函式宣告

// 計算單位面積線性影響係數ˋ
double cal_shifted_area(const line_t &line,const double &unit,const int &shifted_direction);
//Update CommonBounary info: alpha,shift_Direction,shiftAmount
//特例處理 commonBoundary Boundary ID 1
void UpdateCommonBoundaryInfo(std::vector<commonBoundary> &commonBoundaries);
//Update CommonBounary info: alpha,shift_Direction,shiftAmount
void UpdateCommonBoundaryInfo(std::vector<commonBoundary> &commonBoundaries);
// output CommonBoundaries info to commonBoundary_toFurobi.txt
void outputCommonBoundaries(std::vector<commonBoundary> &commonBoundaries);
// output Nets info to netInfo_toGurobi.txt
void outputNetsInfo(std::vector<netInfo> &nets);
// build commonBoundaries from nets info
std::vector<commonBoundary> buildCommonBoundaries(std::vector<netInfo> &nets);
//parse commonBoundary info from xxx_nets_info.txt
std::vector<netInfo> parseNetsInfo(std::ifstream &file);
#endif
