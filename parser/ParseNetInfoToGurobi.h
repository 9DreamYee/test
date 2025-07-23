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
    polygon_t netPolygon;
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
    //corner net的innerRect點座標
    point_t cornerPoint;
    //corner net的斜線
    line_t cornerLine;
    //shiftMin為corner net獨有的平移上限, normal net皆設為0
    //對於corner net來說 shiftMin初始邊界線平移到inner Rect cornerPoint的上限 shiftMax轉成slash/原邊界線從inner Rect cornerPoint可平移的上限
    double shiftMin,shiftMax,shiftMax_corner;
    // 0 -> x, 1 -> y , 2 -> corner net需特例MILP處理
    int shift_Direction;
    int shiftAmount;
    //對於corner net是否採用斜線
    bool isSlash;
    //紀錄phase 2平移後與目標面積的誤差
    double phase2_deviation;
    //協助計算phase2_delta
    // boundary_move_direction根據shiftAmount的正負來決定要將面積調整至哪個pad的方向 0代表往netA 1代表往netB
    // 0 邊界線往netA移動 1 邊界線往netB移動
    int boundary_move_direction;
    //判斷邊界位在哪個方位
    int startPointDirection;
    //phase3 stage_3協助判斷temp_line2_downto_outterRect需要從哪裡開始預留pitch高度
    bool is_Intersect,in_Stage3;
    line_t Temp_line2_downto_outterRect;
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
std::pair<int, double> getNetEdgeKey( commonBoundary &CB, const Rectangle &r);
void sortBoundaryClockwise(std::vector<commonBoundary> &commonBoundaries, const Rectangle &InnerRect);
//Update CommonBounary info: alpha,shift_Direction,shiftAmount
void UpdateCommonBoundaryInfo(std::vector<commonBoundary> &commonBoundaries, std::vector<netInfo> &nets, Rectangle &innerRect, Rectangle &outterRect);

void Phase2UpdateAllInfo_normal_nets(std::vector<double> &deltaVector,std::vector<int> &bVector, 
        std::vector<commonBoundary> &commonBoundaries,std::vector<netInfo> &nets, const Rectangle &InnerRect);
 //計算從bendingPoint轉角的面積值
double cal_bendingPoint_area(line_t &temp_line, line_t &last_line, line_t &cornerLine);
bool isIntersect_with_otherBoundary(const line_t &temp_line2,const commonBoundary &CB, const commonBoundary &adjacent_boundary, const std::vector<netInfo> &nets, std::vector<point_t> &intersectionPoints, point_t &ball);
 //從bendingPoint轉角45度
void rotate_45degree(line_t &line, line_t &last_line, long double &x_bias, long double &y_bias);
//二分逼近stage1_1的面積
line_t binary_search_trapezoid(const double &target_area, const line_t &last_line, const double &x_offset, const double &y_offset, bool isHorizontal);
//Phase3 改變邊界線角度來達到目標面積值
void Phase3(std::vector<commonBoundary> &commonBoundaries, std::vector<netInfo> &nets, std::vector<double> &deltaVector,std::vector<int> &bVector);
void Phase3UpdateAllInfo(std::vector<commonBoundary> &commonBoundaries, std::vector<netInfo> &nets);
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
std::vector<netInfo> parseNetsInfo(std::ifstream &file, double &resolution, Rectangle &innerRect, Rectangle &outterRect);
//After phase2 only update normal nets
#endif
