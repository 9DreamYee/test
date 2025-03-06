#ifndef COMMONBOUNDARY_H
#define COMMONBOUNDARY_H
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <cmath>
#include <cstdlib>
namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> point_t;
typedef bg::model::linestring<point_t> line_t;
typedef bg::model::polygon<point_t> polygon_t;

struct Rectangle{
    double rect_x;
    double rect_y;
    double rect_w;
    double rect_h;
    Rectangle();
    Rectangle(double x, double y, double w, double h);
};
//struct宣告
struct netInfo{
    int netID;
    double pad_x,pad_y;
    double ball_x,ball_y;
    double areaInitial;
    double area;
    point_t pad;
    point_t ball;
    int boundary0ID;
    int boundary1ID;
    point_t startPoint0;
    point_t startPoint1;
    std::vector<line_t> boundarySegments;
    std::vector<line_t> innerBoundarySegments;
    std::vector<line_t> outterBoundarySegments;
    line_t InitialRoute;
    line_t ExtendedInitialRoute;
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
    //alpha_corner 為corner net獨有的線性影響係數, normal net皆設為0
    double alpha,alpha_corner;
    double initial_route_alpha;
    point_t netA_pad,netB_pad;
    point_t cornerPoint;
    line_t cornerLine;
    //shiftMax_corner為corner net獨有的平移上限, normal net皆設為0
    double shiftMin,shiftMax,shiftMax_corner;
    // 0 -> x, 1 -> y , 2 -> corner net需特例MILP處理
    int shift_Direction;
    int shiftAmount;
    //協助計算phase2_delta
    // boundary_move_direction根據shiftAmount的正負來決定要將面積調整至哪個pad的方向 0代表往netA 1代表往netB
    int boundary_move_direction;
    //corner_area為corner net獨有的面積,切換成corner line需要扣除的面積值
    double cornerArea;
    commonBoundary();
};
//函式宣告

//協助判斷特例處理邊界線起點與終點, 不在同個方向上平移的狀況
bool getCornerBoundaryDirection(const commonBoundary &CB, const Rectangle &r, const Rectangle &outterRect);
void cal_corner_area(commonBoundary &CB, line_t &corner_line,commonBoundary &temp_CB, const double &original_area, const double &unit,
       const Rectangle &innerRect, const Rectangle &outterRect );
// 計算單位面積線性影響係數ˋ
double cal_shifted_area(const line_t &line,const double &unit,const int &shifted_direction);
//重新排序Boundary
std::pair<int, double> getNetEdgeKey(const commonBoundary &CB, const Rectangle &r);
void sortBoundaryClockwise(std::vector<commonBoundary> &commonBoundaries, const Rectangle &InnerRect);
//Update CommonBounary info: alpha,shift_Direction,shiftAmount
void UpdateCommonBoundaryInfo(std::vector<commonBoundary> &commonBoundaries, std::vector<netInfo> &nets, Rectangle &innerRect, Rectangle &outterRect);

void Phase2UpdateAllInfo_normal_nets(std::vector<double> &deltaVector,std::vector<int> &bVector, 
        std::vector<commonBoundary> &commonBoundaries,std::vector<netInfo> &nets);
// output CommonBoundaries info to commonBoundary_toFurobi.txt
void outputCommonBoundaries(std::vector<commonBoundary> &commonBoundaries);
// output Nets info to netInfo_toGurobi.txt
void outputNetsInfo(std::vector<netInfo> &nets);
//output CommonBoundaries info to drawing
void outputCommonBoundaries_drawing(std::vector<commonBoundary> &commonBoundaries);
//output 每條net的邊界線與pad & ball資訊給柏丞
void outputPolyInfo(std::vector<netInfo> &nets);
//output Nets info to drawing
void outputNetsInfo_drawing(std::vector<netInfo> &nets);
// build commonBoundaries from nets info
std::vector<commonBoundary> buildCommonBoundaries(std::vector<netInfo> &nets);
//parse commonBoundary info from xxx_nets_info.txt
std::vector<netInfo> parseNetsInfo(std::ifstream &file);
//After phase2 only update normal nets
#endif
