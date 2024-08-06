#ifndef NET_H
#define NET_H
#include <vector>
#include </home/m11115061/boost_1_85_0/boost/polygon/polygon.hpp>
#include </home/m11115061/boost_1_85_0/boost/polygon/voronoi.hpp>
#include </home/m11115061/boost_1_85_0/boost/geometry.hpp>
#include </home/m11115061/boost_1_85_0/boost/geometry/geometries/point_xy.hpp>
#include </home/m11115061/boost_1_85_0/boost/geometry/geometries/segment.hpp>
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
//Define geometry type of Point and Segment to calculate covered_by()
typedef boost::geometry::model::d2::point_xy<int> Point_g;
typedef boost::geometry::model::segment<Point_g> Segment_g;

class Net{
    public:
        Point startPoint;
        Point endPoint;
        // Net's segment
        std::vector<Segment> NetSegments;
        std::vector<Boundary> Boundaries;
        //Vector of InnerRectBoundary & OutterRectBoundary of nets
        std::vector<Boundary> InnerRectBoundarySegments,OutterRectBoundarySegments;
        //Constructor
        Net(Point start,Point end);
        
        //Add segment of net
        void AddNetSegment(const Segment &segment);
        //Add Boundary of net
        void AddBoundary(Point &start,Point &end,const int &ID, const std::vector<Segment> &BoundarySegments);
        //Add InnerRectBoundary of net
        void AddInnerRectBoundarySegments(Point &start,Point &end,const int &ID,const std::vector<Segment> &BoundarySegments);
        //Add OutterRectBoundary of net 
        void AddOutterRectBoundarySegments(Point &start,Point &end,const int &ID,const std::vector<Segment> &BoundarySegments);
        //Calculate length of each net
        int CalculateNetLength();
        //Print net's segments information to drawing
        void PrintNetSegments_drawing();
        //Print Boundary of net information to drawing
        void PrintBoundarySegments_drawing();
        //Print tuple of Boundary of net to calculate area of net
        void PrintBoundaryTuple();
};
#endif
