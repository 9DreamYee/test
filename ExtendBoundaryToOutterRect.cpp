#include <cstdio>
#include <iomanip>
#include <limits>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <unordered_set>
#include <sstream>
#include <fstream>
#include "net.h"
/*#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;
*/
/*
struct Point {
  int a;
  int b;
  Point(int x, int y) : a(x), b(y) {}
};

struct Segment {
  Point p0;
  Point p1;
  Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};
struct Rectangle{
    double rect_x;
    double rect_y;
    double rect_w;
    double rect_h;
};

namespace boost {
namespace polygon {

template <>
struct geometry_concept<Point> {
  typedef point_concept type;
};

template <>
struct point_traits<Point> {
  typedef int coordinate_type;

  static inline coordinate_type get(
      const Point& point, orientation_2d orient) {
    return (orient == HORIZONTAL) ? point.a : point.b;
  }
};

template <>
struct geometry_concept<Segment> {
  typedef segment_concept type;
};

template <>
struct segment_traits<Segment> {
  typedef int coordinate_type;
  typedef Point point_type;

  static inline point_type get(const Segment& segment, direction_1d dir) {
    return dir.to_int() ? segment.p1 : segment.p0;
  }
};
}  // polygon
}  // boost
*/
void InputFile_EliminatedInnerBoundaryResult(std::ifstream &file,std::vector<Segment> &segments,std::vector<Segment> &nets){
    std::string str,temp,str_start_x,str_start_y,str_end_x,str_end_y;
    std::stringstream ss;
    std::ofstream outfile("ExtendBoundaryToOutterRect_result.txt");
    double start_x,start_y,end_x,end_y;
    bool all_net_info_end= false;
    if (!file) {
        std::cerr << "Unable to open file\n"<<"usage ./ExtendBoundaryToOutterRect <input_file>(eliminated_inner_boundary_result.txt)\n";
    }

    while(getline(file,str)){
        ss.clear();
        ss.str("");
        if(str == "all_net info start:"){
            continue;
        }
        if (str == "all_net info end"){
            all_net_info_end = true;
            //nets.emplace_back(str);
            continue;
        }
        if(!all_net_info_end){
            ss.clear();
            ss.str("");
            ss<<str;
            ss>>temp;
            ss<<str;
            ss>>str_start_x;
            ss<<str;
            ss>>str_start_y;
            ss<<str;
            ss>>str_end_x;
            ss<<str;
            ss>>str_end_y;
            start_x = std::stod(str_start_x);
            start_y = std::stod(str_start_y);
            end_x = std::stod(str_end_x);
            end_y = std::stod(str_end_y);
            Segment net(start_x,start_y,end_x,end_y);
            nets.emplace_back(net);
        }
        else if(all_net_info_end){
            ss<<str;
            ss>>temp;
            ss<<str;
            ss>>str_start_x;
            ss<<str;
            ss>>str_start_y;
            ss<<str;
            ss>>str_end_x;
            ss<<str;
            ss>>str_end_y;
            start_x = std::stod(str_start_x);
            start_y = std::stod(str_start_y);
            end_x = std::stod(str_end_x);
            end_y = std::stod(str_end_y);
            Segment segment(start_x,start_y,end_x,end_y);
            segments.emplace_back(segment); 
        }
    }//while(getline) end
}
void OutputFileBoundariesToDrawing(std::vector<std::vector<Segment>> &boundaries,std::vector<std::vector<Segment>> &NetsBoundaries){
    std:: ofstream outfile("ExtendBoundaryToOutterRect_result.txt");
    outfile<<"all_net info start, the number of nets:\n"<<NetsBoundaries.size()<<"\n";
    for(int i = 0; i < NetsBoundaries.size(); i++){
       outfile<<"Net"<<i<<":\n"; 
       for(auto Netseg: NetsBoundaries[i]){
           outfile<<"Net_segment: "<<std::scientific<<Netseg.p0.a<<" "<<Netseg.p0.b<<" "<<Netseg.p1.a<<" "<<Netseg.p1.b<<"\n";
       }
    }
    outfile<<"all_net info end, the number of boundaries:\n"<<boundaries.size()<<"\n";
    for(int i = 0;i < boundaries.size(); i++){
        outfile<<"boundary"<<i<<":\n";
        for(auto seg : boundaries[i]){
            //std::cout<<"_view.drawLine("<<std::scientific<<seg.p0.a<<","<<std::scientific<<seg.p0.b<<","<<std::scientific<<seg.p1.a<<","<<std::scientific<<seg.p1.b<<");\n";
            outfile<<"boundary: "<<std::scientific<<seg.p0.a<<" "<<seg.p0.b<<" "<<seg.p1.a<<" "<<seg.p1.b<<"\n";
        }
    }
}

int euclidian_distance(const Segment& segment, const Segment& rect){
    //return std::sqrt(std::pow(s1.p0.a - s2.p0.a,2) + std::pow(s1.p0.b - s2.p0.b,2));
    return 
        std::min({
            boost::polygon::euclidean_distance(rect,segment.p0),
            boost::polygon::euclidean_distance(rect,segment.p1),
            boost::polygon::euclidean_distance(segment,rect.p0),
            boost::polygon::euclidean_distance(segment,rect.p1)
        });
}
void ExploreBoundary(int index,const std::vector<Segment> &segments,std::unordered_set<int> &visited,std::vector<Segment> &boundary){
    boundary.emplace_back(segments[index]);
    visited.emplace(index);
    int Explore_cnt = 0;
    for(int i = 0; i < segments.size(); i++){
        bool isAbuts = false;
        isAbuts = boost::polygon::abuts(segments[index],segments[i]);
        if(visited.find(i) == visited.end() && isAbuts){
            ExploreBoundary(i,segments,visited,boundary);
        }
    }
}
std::vector<std::vector<Segment>> groupSegments(const std::vector<Segment> &segments){
    std::vector<std::vector<Segment>> boundaries;
    std::unordered_set<int> visited;
    int group_cnt = 0;
    for(int i = 0; i < segments.size(); i++){
        if(visited.find(i) == visited.end()){
            std::vector<Segment> boundary;
            ExploreBoundary(i,segments,visited,boundary);
            boundaries.emplace_back(boundary);
        }
    }

    return boundaries;
}
void findClosestSegment(const std::vector<Segment>& boundary, Rectangle &rect,int &Rect_side, int &Segment_side,int &closest_segment_p0_a,int &closest_segment_p0_b,
    int &closest_segment_p1_a,int &closest_segment_p1_b){
    Segment closest_segment = {0,0,0,0};
    Segment rect_left = {int(rect.rect_x),int(rect.rect_y),int(rect.rect_x),int(rect.rect_y) + int(rect.rect_h)}; 
    Segment rect_right = {int(rect.rect_x + rect.rect_w),int(rect.rect_y),int(rect.rect_x + rect.rect_w),int(rect.rect_y) + int(rect.rect_h)};
    Segment rect_top =  {int(rect.rect_x),int(rect.rect_y)+int(rect.rect_h),int(rect.rect_x + rect.rect_w),int(rect.rect_y)+int(rect.rect_h)};
    Segment rect_bottom = {int(rect.rect_x),int(rect.rect_y),int(rect.rect_x + rect.rect_w),int(rect.rect_y)};
    int minDistance = std::numeric_limits<int>::max();
    //判斷segmentts與哪邊最相近
    for(auto segment : boundary){
        
        int DistanceBetweenTop = euclidian_distance(segment,rect_top);
        int DistanceBetweenBottom = euclidian_distance(segment,rect_bottom);
        int DistanceBetweenLeft = euclidian_distance(segment,rect_left);
        int DistanceBetweenRight = euclidian_distance(segment,rect_right);
        int Distance_Point_startBetweenRect = 0;
        int Distance_Point_endBetweenRect = 0;
        if(minDistance > DistanceBetweenTop){
            minDistance = DistanceBetweenTop;
            Rect_side = 1;
            closest_segment = segment;
        }
        if(minDistance > DistanceBetweenBottom){
            minDistance = DistanceBetweenBottom;
            Rect_side = 2;
            closest_segment = segment;
        }
        if(minDistance > DistanceBetweenLeft){
            minDistance = DistanceBetweenLeft;
            Rect_side = 3;
            closest_segment = segment;
        }
        if(minDistance > DistanceBetweenRight){
            minDistance = DistanceBetweenRight;
            Rect_side = 4;
            closest_segment = segment;
        }
        switch(Rect_side){
            case 1:
                Distance_Point_startBetweenRect = boost::polygon::euclidean_distance(rect_top,closest_segment.p0);
                Distance_Point_endBetweenRect = boost::polygon::euclidean_distance(rect_top,closest_segment.p1);
                Segment_side = (Distance_Point_startBetweenRect < Distance_Point_endBetweenRect) ? 1 : 2;
                break;
            case 2:
                Distance_Point_startBetweenRect = boost::polygon::euclidean_distance(rect_bottom,closest_segment.p0);
                Distance_Point_endBetweenRect = boost::polygon::euclidean_distance(rect_bottom,closest_segment.p1);
                Segment_side = (Distance_Point_startBetweenRect < Distance_Point_endBetweenRect) ? 1 : 2;
                break;
            case 3:
                Distance_Point_startBetweenRect = boost::polygon::euclidean_distance(rect_left,closest_segment.p0);
                Distance_Point_endBetweenRect = boost::polygon::euclidean_distance(rect_left,closest_segment.p1);
                Segment_side = (Distance_Point_startBetweenRect < Distance_Point_endBetweenRect) ? 1 : 2;
                break;
            case 4:
                Distance_Point_startBetweenRect = boost::polygon::euclidean_distance(rect_right,closest_segment.p0);
                Distance_Point_endBetweenRect = boost::polygon::euclidean_distance(rect_right,closest_segment.p1);
                Segment_side = (Distance_Point_startBetweenRect < Distance_Point_endBetweenRect) ? 1: 2;
                break;
        }
    }
    closest_segment_p0_a = closest_segment.p0.a;
    closest_segment_p0_b = closest_segment.p0.b;
    closest_segment_p1_a = closest_segment.p1.a;
    closest_segment_p1_b = closest_segment.p1.b;
    //std::cout<<"Rect_side: "<<Rect_side<<" Segment_side: "<<Segment_side<<std::endl;
}// end findClosestSegment
void extendBoundaryToOutterRect(std::vector<Segment> &boundary,Segment &closest_segment,Rectangle &rect,int &Rect_side,int &Segment_side){
   
    for (auto &seg:boundary){
        if((closest_segment.p0.a == seg.p1.a) &&(closest_segment.p0.b == seg.p1.b)&&
                (closest_segment.p1.a == seg.p0.a) &&(closest_segment.p1.b == seg.p0.b)){
            seg.p0.a = closest_segment.p0.a;
            seg.p0.b = closest_segment.p0.b;
            seg.p1.a = closest_segment.p1.a;
            seg.p1.b = closest_segment.p1.b;
        }
        if((closest_segment.p0.a == seg.p0.a) && (closest_segment.p0.b == seg.p0.b) && (closest_segment.p1.a == seg.p1.a) && (closest_segment.p1.b == seg.p1.b)){
            //std::cout<<"seg ori_coord: "<<seg.p0.a<<" "<<seg.p0.b<<" "<<seg.p1.a<<" "<<seg.p1.b<<std::endl;i
            //std::cout<<"rect_coord:"<<rect.rect_x<<" "<<rect.rect_y<<" "<<rect.rect_w<<" "<<rect.rect_h<<std::endl;      
            switch(Rect_side){
                case 1:
                    switch(Segment_side){
                        case 1:
                            seg.p0.b = int(rect.rect_y) + int(rect.rect_h);
                            break;
                        case 2:
                            seg.p1.b = int(rect.rect_y) + int(rect.rect_h);
                            break;
                    }
                    break;
                case 2:
                    switch(Segment_side){
                        case 1:
                            seg.p0.b = int(rect.rect_y);
                            break;
                        case 2:
                            seg.p1.b = int(rect.rect_y);
                            break;
                    }
                    break;
                case 3:
                    switch(Segment_side){
                        case 1:
                            seg.p0.a = int(rect.rect_x);
                            break;
                        case 2:
                            seg.p1.a = int(rect.rect_x);
                            break;
                    }
                    break;
                case 4:
                    switch(Segment_side){
                        case 1:
                            seg.p0.a = int (rect.rect_x) + int(rect.rect_w);
                            break;
                        case 2:
                            seg.p1.a = int(rect.rect_x) + int(rect.rect_w);
                            break;
                    }
                    break;
            }
        
        //std::cout<<"seg new_coord: "<<seg.p0.a<<" "<<seg.p0.b<<" "<<seg.p1.a<<" "<<seg.p1.b<<std::endl;   
        }
    }
}
void Eliminate_RedudantSegment(std::vector<Segment> &boundary){
    std::unordered_set<int> visited;
    std::vector <Segment> new_boundary;
    for(int i = 0; i < boundary.size(); i++){
       for(int j = i+1; j < boundary.size(); j++){
           if((boundary[i].p0.a == boundary[j].p0.a && boundary[i].p0.b == boundary[j].p0.b && boundary[i].p1.a == boundary[j].p1.a && boundary[i].p1.b == boundary[j].p1.b)){
                visited.emplace(j);
                break;
           }
           if(boundary[i].p0.a == boundary[j].p1.a && boundary[i].p0.b == boundary[j].p1.b && boundary[i].p1.a == boundary[j].p0.a && boundary[i].p1.b == boundary[j].p0.b){
                visited.emplace(j);
                break;
           }
       }
    }
    for (int i = 0; i < boundary.size();i++){
        if((visited.find(i) == visited.end())){
            new_boundary.emplace_back(boundary[i]);
        }
    }
    boundary = new_boundary;
    new_boundary.clear();
    visited.clear();
}
void Eliminate_IsolatedSegment(std::vector<Segment> &boundary){
    std::unordered_set<int> visited; //segments that need to be keep
    std::vector <Segment> new_boundary;
    int inv_i_p0_a = 0,inv_i_p0_b = 0,inv_i_p1_a = 0,inv_i_p1_b = 0;
    bool isAbuts = false,is_invi_Abuts = false;
    for (int i = 0; i < boundary.size();i++){
            inv_i_p0_a = boundary[i].p1.a;
            inv_i_p0_b = boundary[i].p1.b;
            inv_i_p1_a = boundary[i].p0.a;
            inv_i_p1_b = boundary[i].p0.b;
            Segment inv_i (inv_i_p0_a,inv_i_p0_b,inv_i_p1_a,inv_i_p1_b);
        for(int j = i+1; j < boundary.size();j++){
            isAbuts = false;
            is_invi_Abuts = false;
            isAbuts = boost::polygon::abuts(boundary[i],boundary[j]);
            is_invi_Abuts = boost::polygon::abuts(inv_i,boundary[j]);
            if(isAbuts||is_invi_Abuts){
                visited.emplace(i);
                visited.emplace(j);
            }   
        }
    }
    for (auto i : visited){
        new_boundary.emplace_back(boundary[i]);
    }
    boundary = new_boundary;
    new_boundary.clear();
    visited.clear();
}
int main(int argc,char * argv[]){
    //usage ./ExtendBoundaryToOutterRect <input_file>(eliminated_inner_boundary_result.txt)
    std::ifstream file(argv[1]);
    double outter_rect_x = stod(argv[2]);
    double outter_rect_y = stod(argv[3]);
    double outter_rect_w = stod(argv[4]);
    double outter_rect_h = stod(argv[5]);

    std::string temp,str,str_start_x,str_start_y,str_end_x,str_end_y;
    std::stringstream ss;
    std::vector<Segment> segments;
    std::vector<Segment> nets;
    std::vector<std::vector<Segment>> boundaries;
    std::vector<std::vector<Segment>> NetsBoundaries;
    //std::vector<std::vector<Segment>> InitialRouting_extendedtoOutterRect;
    int Segment_side = 0; //closest Segment_side: 1 for start, 2 for end
    int Rect_side = 0; //closest Rect_side: 1 for top, 2 for bottom, 3 for left, 4 for right
    int closest_segment_p0_a = 0,closest_segment_p0_b = 0,closest_segment_p1_a = 0,closest_segment_p1_b = 0;
    double start_x,start_y,end_x,end_y;
    Segment closest_segment = {0,0,0,0};

    //Rectangle rect = {-1.4e+8,-1.4e+8,2.8e+8,2.8e+8};
    Rectangle rect = {outter_rect_x,outter_rect_y,outter_rect_w,outter_rect_h};

    InputFile_EliminatedInnerBoundaryResult(file,segments,nets);
    boundaries = groupSegments(segments);  
    NetsBoundaries = groupSegments(nets);
    //InitialRouting_extendedtoOutterRect = NetsBoundaries;
    //extend each boundary to outter rect
    for(int i = 0; i < boundaries.size();i++){
        findClosestSegment(boundaries[i],rect,Rect_side,Segment_side,closest_segment_p0_a,closest_segment_p0_b,closest_segment_p1_a,closest_segment_p1_b);
        closest_segment.p0.a = closest_segment_p0_a;
        closest_segment.p0.b = closest_segment_p0_b;
        closest_segment.p1.a = closest_segment_p1_a;
        closest_segment.p1.b = closest_segment_p1_b;
        extendBoundaryToOutterRect(boundaries[i],closest_segment,rect,Rect_side,Segment_side);
    }
    //extend each initial routing to outter rect
    /*
    for(int i = 0; i < InitialRouting_extendedtoOutterRect.size();i++){
        findClosestSegment(InitialRouting_extendedtoOutterRect[i],rect,Rect_side,Segment_side,closest_segment_p0_a,closest_segment_p0_b,closest_segment_p1_a,closest_segment_p1_b);
        closest_segment.p0.a = closest_segment_p0_a;
        closest_segment.p0.b = closest_segment_p0_b;
        closest_segment.p1.a = closest_segment_p1_a;
        closest_segment.p1.b = closest_segment_p1_b;
        extendBoundaryToOutterRect(InitialRouting_extendedtoOutterRect[i],closest_segment,rect,Rect_side,Segment_side);
    }
    */
    
    for(int i = 0; i < boundaries.size(); i++){
       Eliminate_RedudantSegment(boundaries[i]);
       Eliminate_IsolatedSegment(boundaries[i]);
    }
    //OutputFileBoundariesToDrawing(boundaries,NetsBoundaries,InitialRouting_extendedtoOutterRect);
    OutputFileBoundariesToDrawing(boundaries,NetsBoundaries);
}//main end


