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
#include <utility>
#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;


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
struct Boundary{
    Point startPoint;
    Point endPoint;
    int BoundaryID;
    std::vector<Segment> BoundarySegments;
    Boundary(Point start,Point end,int ID,std::vector<Segment> seg):startPoint(start),endPoint(end),BoundaryID(ID),BoundarySegments(seg){}
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

class Net{
    public :
        //Net's startPoint & endPoint
        Point startPoint;
        Point endPoint;
        //Net's boundary start point & end point

        // Net's segment
        std::vector<Segment> NetSegments;
        std::vector<Boundary> Boundaries;
        //Net's boundary start point & end point
        /*std::pair<Point,Point> topBoundaryStart, topBoundaryEnd;
        std::pair<Point,Point> bottomBoundaryStart, bottomBoundaryEnd;
        std::pair<Point,Point> leftBoundaryStart, leftBoundaryEnd;
        std::pair<Point,Point> rightBoundaryStart, rightBoundaryEnd;*/
        //Vector of four Net's boundary(top,bottom,left,right)
        std::vector<Segment> topBoundarySegmets,bottomBoundarySegmets,
            leftBoundarySegmets,rightBoundarySegmets;
        //Constructor
        Net(Point start,Point end)
            : startPoint(start),endPoint(end){}
        //Add net's segment 
        void AddNetSegment(const Segment &segment){
            NetSegments.emplace_back(segment);
            //std::cout<<"Net_segment: "<<segment.p0.a<<" "<<segment.p0.b<<" "<<segment.p1.a<<" "<<segment.p1.b<<"\n";
        }
        void AddBoundary(Point &start,Point &end,const int &ID, const std::vector<Segment> &BoundarySegments){
            //Boundary(Point start,Point end,int ID,std::vector<Segment> seg):
            Boundaries.emplace_back(Boundary(start,end,ID,BoundarySegments));
        }

        void PrintNetSegments(){
            for(int i = 0;i < NetSegments.size();i++){
                std::cout<<"Net_segment: "<<NetSegments[i].p0.a<<" "<<NetSegments[i].p0.b<<" "<<NetSegments[i].p1.a<<" "<<NetSegments[i].p1.b<<"\n";
            }
        }
        /*void PrintBoundarySegments(){
            for(int i = 0;i < BoundarySegments.size();i++){
                std::cout<<"Boundary_segment: "<<BoundarySegments[i].p0.a<<" "<<BoundarySegments[i].p0.b<<" "<<BoundarySegments[i].p1.a<<" "<<BoundarySegments[i].p1.b<<"\n";
            }
        }*/
};
void ProcessingSegmentInfo(std::string &str,double &start_x,double &start_y,double &end_x,double &end_y){
    std::string temp,str_start_x,str_start_y,str_end_x,str_end_y;
    std::stringstream ss;
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
}//end of ProcessingSegmentInfo
 //Find the start point and end point of a net/boundary
void FindStartAndEndPoint(std::vector<Segment> &one_net,Point &start,Point &end){
    std::vector<Point> net_points;
    int max = std::numeric_limits<int>::min();
    int distance = 0;
    Point temp_start(0,0);
    Point temp_end(0,0);
    for(int i = 0;i < one_net.size();i++){
        net_points.emplace_back(one_net[i].p0);
        net_points.emplace_back(one_net[i].p1);
    }
    for(int i = 0;i < net_points.size();i++){
        for(int j = i+1;j < net_points.size();j++){
           distance = boost::polygon::manhattan_distance(net_points[i],net_points[j]);
           if(distance > max){
                max = distance;
                temp_start = net_points[i];
                temp_end = net_points[j];
           }
        }
    }
    start = temp_start;
    end = temp_end;
    
}//end of FindStartAndEndPoint
void CreatingNet(std::vector<Segment> &one_net,Point &start,Point &end,std::vector<Net> &nets){
    Net net(start,end);
    for(int i = 0;i < one_net.size();i++){
        net.AddNetSegment(one_net[i]);
    } 
    nets.emplace_back(net);
} //end of CreatingNet
void CreatingBoundary(std::vector<Segment> &one_boundary,Point &start,Point &end,int ID,std::vector<Boundary> &boundaries){
    Boundary boundary(start,end,ID,one_boundary);//Boundary(Point start,Point end,int ID,std::vector<Segment> seg):
    /*for(int i = 0;i<one_boundary.size();i++){
        boundary.emplace_back(one_boundary[i]);
    }*/
    boundaries.emplace_back(boundary);
}
void InputFile_ExtendBoundaryToOutterRect_result(std::ifstream &file,std::vector<Segment> &AllBoundarySegments,std::vector<Segment> &AllnetsSegments,std::vector<Net> &nets,std::vector<Boundary> &boundaries){
    std::string str,Net_filiter,Boundary_filiter;
    std::stringstream ss;
    std::vector<Segment> one_net;
    std::vector<Segment> one_boundary;
    double start_x,start_y,end_x,end_y;
    bool all_net_info_end= false;
    int cnt = 1,net_num = 0, boundary_num = 0,ID = 0;
    Point start(0,0);
    Point end(0,0);
    if (!file) {
        std::cerr << "Unable to open file\n"<<"usage ./OctilinearizeBoundary <input_file>(ExtendBoundaryToOutter_result.txt)\n";
    }
    while(getline(file,str)){
        ss.clear();
        ss.str("");
        if(str =="all_net info start, the number of nets:"){
            getline(file,str);
            net_num = std::stoi(str);
            getline(file,str);
            continue;
        }
        if (str == "all_net info end, the number of boundaries:"){
            //Start processing boundary info
            cnt = 1;
            all_net_info_end = true;
            getline(file,str); //str = the number of boundaries
            boundary_num = std::stoi(str);
            getline(file,str); //str = boundary0:;
            //processing the last net info
            FindStartAndEndPoint(one_net,start,end);
            CreatingNet(one_net,start,end,nets);
            one_net.clear();
            continue;
        } 
         //processing net Info & Creating net 
        if(!all_net_info_end){
            Net_filiter = "Net" + std::to_string(cnt) + ":";
            if(str == Net_filiter){
                cnt++;
                FindStartAndEndPoint(one_net,start,end);
                CreatingNet(one_net,start,end,nets);
                one_net.clear();
                continue;
            }

            ProcessingSegmentInfo(str,start_x,start_y,end_x,end_y);
            Segment net(start_x,start_y,end_x,end_y); 
            AllnetsSegments.emplace_back(net);
            one_net.emplace_back(net);
            }
        //Processing boundary Info 保留boundaryID的作法 加快後續移動boundary後更新net成員函數的ˊ時間
        else if(all_net_info_end){
            if(boundary_num){
                Boundary_filiter = "boundary" + std::to_string(cnt) + ":";
                if(str == Boundary_filiter){
                    ID = cnt-1;
                    cnt++;
                    FindStartAndEndPoint(one_boundary,start,end);
                    CreatingBoundary(one_boundary,start,end,ID,boundaries);
                    one_boundary.clear();
                    boundary_num--;
                    continue;
                }
                ProcessingSegmentInfo(str,start_x,start_y,end_x,end_y);
                Segment BoundarySegment(start_x,start_y,end_x,end_y);
                AllBoundarySegments.emplace_back(BoundarySegment);
                one_boundary.emplace_back(BoundarySegment);
            }
        }    
    }//while(getline) end
    ID = cnt-1;
    FindStartAndEndPoint(one_boundary,start,end);
    CreatingBoundary(one_boundary,start,end,ID,boundaries);
    one_boundary.clear();
}

void OutputFile_OctilinearizeBoundary(std::ifstream &file,std::vector<Segment> &segments,std::vector<Segment> &nets){
    std::ofstream outfile("OctilinearizeBoundary_result.txt");
    for(int i = 0; i < nets.size(); i++){
        outfile<<nets[i].p0.a<<" "<<nets[i].p0.b<<" "<<nets[i].p1.a<<" "<<nets[i].p1.b<<std::endl;
    }
    for(int i = 0; i < segments.size(); i++){
        outfile<<segments[i].p0.a<<" "<<segments[i].p0.b<<" "<<segments[i].p1.a<<" "<<segments[i].p1.b<<std::endl;
    } 
}
int main(int argc,char* argv[]){
    std::ifstream file(argv[1]);
    std::vector<Segment> AllBoundarySegments;
    std::vector<Segment> AllnetsSegments;
    std::vector<Net> nets;
    std::vector<Boundary> boundaries;
    //std::vector<Segment> BoundarySegments;
    //std::vector<Segment> NetSegments;
    InputFile_ExtendBoundaryToOutterRect_result(file,AllBoundarySegments,AllnetsSegments,nets,boundaries);
    //OutputFile_OctilinearizeBoundary(file,AllBoundarySegments,AllnetsSegments);
    //nets.erase(nets.begin());
    /*for (int i = 0;i < nets.size();i++){
        std::cout<<"Net:"<<i<<" "<<nets[i].startPoint.a<<" "<<nets[i].startPoint.b<<" "<<nets[i].endPoint.a<<" "<<nets[i].endPoint.b<<"\n";
        nets[i].PrintNetSegments(); 
    }*/
    return 0;
}//end of  main