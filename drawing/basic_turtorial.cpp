// Boost.Polygon library voronoi_basic_tutorial.cpp file

//          Copyright Andrii Sydorchuk 2010-2012.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

// See http://www.boost.org for updates, documentation, and revision history.

#include <cstdio>
#include <vector>
#include <iostream>
#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

//#include "voronoi_visual_utils.hpp"

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

// Traversing Voronoi edges using edge iterator.
int iterate_primary_edges1(const voronoi_diagram<double>& vd) {
  int result = 0;
  std::cout<<"\nusing edge iterator result: "<<std::endl;
  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
       it != vd.edges().end(); ++it) {
    if (it->is_primary()){
        if (it->vertex0()!=nullptr && it->vertex1()!=nullptr){
            double start_x = it->vertex0()->x();
            double start_y = it->vertex0()->y();
            double end_x = it->vertex1()->x();
            double end_y = it->vertex1()->y();

            std::cout<<"Edge start point: "<<start_x<<","<<start_y<<std::endl;
            std::cout<<"Edge end point: "<<end_x<<","<<end_y<<std::endl;
        }
      ++result;  
    }
  }
  std::cout<<"end of edge iterator result"<<std::endl;
  return result;
}

// Traversing Voronoi edges using cell iterator.
int iterate_primary_edges2(const voronoi_diagram<double> &vd) {
  int result = 0;
  std::cout<<"\nusing cell iterator result"<<std::endl;
  for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
       it != vd.cells().end(); ++it) {
    const voronoi_diagram<double>::cell_type& cell = *it;
    const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();
    // This is convenient way to iterate edges around Voronoi cell.
    do {
      if (edge->is_primary()){
        if (edge->vertex0()!=nullptr && edge->vertex1()!=nullptr){
            double start_x = edge->vertex0()->x();
            double start_y = edge->vertex0()->y();
            double end_x = edge->vertex1()->x();
            double end_y = edge->vertex1()->y();
            std::cout<<"Edge start point: "<<start_x<<","<<start_y<<std::endl;
            std::cout<<"Edge end point: "<<end_x<<","<<end_y<<std::endl;
        }
        ++result;
      }
      edge = edge->next();
    } while (edge != cell.incident_edge());
  }
  std::cout<<"end of cell iterator result"<<std::endl;
  return result;
}

// Traversing Voronoi edges using vertex iterator.
// As opposite to the above two functions this one will not iterate through
// edges without finite endpoints and will iterate only once through edges
// with single finite endpoint.
int iterate_primary_edges3(const voronoi_diagram<double> &vd) {
  int result = 0;
  std::cout<<"\nusing vertex iterator result: "<<std::endl;
  for (voronoi_diagram<double>::const_vertex_iterator it =
       vd.vertices().begin(); it != vd.vertices().end(); ++it) {
    const voronoi_diagram<double>::vertex_type& vertex = *it;
    const voronoi_diagram<double>::edge_type* edge = vertex.incident_edge();
    // This is convenient way to iterate edges around Voronoi vertex.
    do {
      if (edge->is_primary()){
        if (edge->vertex0()!=nullptr && edge->vertex1()!=nullptr){
            double start_x = edge->vertex0()->x();
            double start_y = edge->vertex0()->y();
            double end_x = edge->vertex1()->x();
            double end_y = edge->vertex1()->y();
            std::cout<<"Edge start point: "<<start_x<<","<<start_y<<std::endl;
            std::cout<<"Edge end point: "<<end_x<<","<<end_y<<std::endl;
        }
          ++result;
      }
      edge = edge->rot_next();
    } while (edge != vertex.incident_edge());
  }
  std::cout<<"end of vertex iterator result"<<std::endl;
  return result;
}

int main() {
  // Preparing Input Geometries.
  std::vector<Point> points;
  std::vector<Segment> segments;
segments.push_back(Segment(-50000000,37673500, -90000000,77673500));
segments.push_back(Segment(-90000000,77673500, -90000000,80000000));
segments.push_back(Segment(-44207300,50000000, -44207300,54207300));
segments.push_back(Segment(-44207300,54207300, -80000000,90000000));
segments.push_back(Segment(-35811400,50000000, -35811400,54061600));
segments.push_back(Segment(-35811400,54061600, -60000000,78250200));
segments.push_back(Segment(-60000000,78250200, -60000000,90000000));
segments.push_back(Segment(-27184900,50000000, -50000000,72815100));
segments.push_back(Segment(-50000000,72815100, -50000000,90000000));
segments.push_back(Segment(-20153100,50000000, -20153100,70153100));
segments.push_back(Segment(-20153100,70153100, -40000000,90000000));
segments.push_back(Segment(-13500300,50000000, -13500300,73500300));
segments.push_back(Segment(-13500300,73500300, -30000000,90000000));
segments.push_back(Segment(-6508600,50000000, -6508600,76508600));
segments.push_back(Segment(-6508600,76508600, -20000000,90000000));
segments.push_back(Segment(-150100,50000000, -150100,69849900));
segments.push_back(Segment(-150100,69849900, 20000000,90000000));
segments.push_back(Segment(6218300,50000000, 6218300,66218300));
segments.push_back(Segment(6218300,66218300, 30000000,90000000));
segments.push_back(Segment(12753800,50000000, 12753800,62753800));
segments.push_back(Segment(12753800,62753800, 40000000,90000000));
segments.push_back(Segment(19399300,50000000, 19399300,59399300));
segments.push_back(Segment(19399300,59399300, 50000000,90000000));
segments.push_back(Segment(26459700,50000000, 26459700,56459700));
segments.push_back(Segment(26459700,56459700, 60000000,90000000));
segments.push_back(Segment(33645200,50000000, 33645200,50943600));
segments.push_back(Segment(33645200,50943600, 72701600,90000000));
segments.push_back(Segment(72701600,90000000, 80000000,90000000));
segments.push_back(Segment(43651600,50000000, 77251800,83600200));
segments.push_back(Segment(77251800,83600200, 83600200,83600200));
segments.push_back(Segment(83600200,83600200, 90000000,90000000));
segments.push_back(Segment(50000000,43723200, 86276800,80000000));
segments.push_back(Segment(86276800,80000000, 90000000,80000000));
segments.push_back(Segment(50000000,37981000, 67981000,37981000));
segments.push_back(Segment(67981000,37981000, 90000000,60000000));
segments.push_back(Segment(50000000,30427000, 70427000,30427000));
segments.push_back(Segment(70427000,30427000, 90000000,50000000));
segments.push_back(Segment(50000000,23060200, 73060200,23060200));
segments.push_back(Segment(73060200,23060200, 90000000,40000000));
segments.push_back(Segment(50000000,13493300, 73493300,13493300));
segments.push_back(Segment(73493300,13493300, 90000000,30000000));
segments.push_back(Segment(50000000,6885000, 76885000,6885000));
segments.push_back(Segment(76885000,6885000, 90000000,20000000));
segments.push_back(Segment(50000000,486800, 69513200,486800));
segments.push_back(Segment(69513200,486800, 90000000,-20000000));
segments.push_back(Segment(50000000,-5859700, 65859700,-5859700));
segments.push_back(Segment(65859700,-5859700, 90000000,-30000000));
segments.push_back(Segment(50000000,-12280300, 62280300,-12280300));
segments.push_back(Segment(62280300,-12280300, 90000000,-40000000));
segments.push_back(Segment(50000000,-18797200, 58797200,-18797200));
segments.push_back(Segment(58797200,-18797200, 90000000,-50000000));
segments.push_back(Segment(50000000,-25596900, 55596900,-25596900));
segments.push_back(Segment(55596900,-25596900, 90000000,-60000000));
segments.push_back(Segment(50000000,-33844500, 90000000,-73844500));
segments.push_back(Segment(90000000,-73844500, 90000000,-80000000));
segments.push_back(Segment(48867200,-48169400, 90000000,-89302200));
segments.push_back(Segment(90000000,-89302200, 90000000,-90000000));
segments.push_back(Segment(44286800,-50000000, 44286800,-54286800));
segments.push_back(Segment(44286800,-54286800, 80000000,-90000000));
segments.push_back(Segment(38294200,-50000000, 38294200,-68294200));
segments.push_back(Segment(38294200,-68294200, 60000000,-90000000));
segments.push_back(Segment(28598300,-50000000, 28598300,-68598300));
segments.push_back(Segment(28598300,-68598300, 50000000,-90000000));
segments.push_back(Segment(21554800,-50000000, 21554800,-71554800));
segments.push_back(Segment(21554800,-71554800, 40000000,-90000000));
segments.push_back(Segment(14856000,-50000000, 14856000,-74856000));
segments.push_back(Segment(14856000,-74856000, 30000000,-90000000));
segments.push_back(Segment(8337300,-50000000, 8337300,-78337300));
segments.push_back(Segment(8337300,-78337300, 20000000,-90000000));
segments.push_back(Segment(1945600,-50000000, 1945600,-68054400));
segments.push_back(Segment(1945600,-68054400, -20000000,-90000000));
segments.push_back(Segment(-4398200,-50000000, -4398200,-64398200));
segments.push_back(Segment(-4398200,-64398200, -30000000,-90000000));
segments.push_back(Segment(-40000000,-90000000, -10853000,-60853000));
segments.push_back(Segment(-10853000,-60853000, -10853000,-50000000));
segments.push_back(Segment(-17942500,-50000000, -17942500,-57942500));
segments.push_back(Segment(-17942500,-57942500, -50000000,-90000000));
segments.push_back(Segment(-24895700,-50000000, -24895700,-54895700));
segments.push_back(Segment(-24895700,-54895700, -60000000,-90000000));
segments.push_back(Segment(-32073500,-50000000, -32073500,-50960500));
segments.push_back(Segment(-32073500,-50960500, -71113000,-90000000));
segments.push_back(Segment(-71113000,-90000000, -80000000,-90000000));
segments.push_back(Segment(-41364900,-50000000, -74965100,-83600200));
segments.push_back(Segment(-74965100,-83600200, -83600200,-83600200));
segments.push_back(Segment(-83600200,-83600200, -90000000,-90000000));
segments.push_back(Segment(-49900000,-45994800, -55994800,-45994800));
segments.push_back(Segment(-55994800,-45994800, -90000000,-80000000));
segments.push_back(Segment(-50000000,-35157800, -65157800,-35157800));
segments.push_back(Segment(-65157800,-35157800, -90000000,-60000000));
segments.push_back(Segment(-50000000,-27904500, -67904500,-27904500));
segments.push_back(Segment(-67904500,-27904500, -90000000,-50000000));
segments.push_back(Segment(-50000000,-20770700, -70770700,-20770700));
segments.push_back(Segment(-70770700,-20770700, -90000000,-40000000));
segments.push_back(Segment(-49968300,-14965600, -74965600,-14965600));
segments.push_back(Segment(-74965600,-14965600, -90000000,-30000000));
segments.push_back(Segment(-50000000,-7071000, -77071000,-7071000));
segments.push_back(Segment(-77071000,-7071000, -90000000,-20000000));
segments.push_back(Segment(-50000000,-694400, -69305600,-694400));
segments.push_back(Segment(-69305600,-694400, -90000000,20000000));
segments.push_back(Segment(-50000000,5656000, -65656000,5656000));
segments.push_back(Segment(-65656000,5656000, -90000000,30000000));
segments.push_back(Segment(-50000000,12135500, -62135500,12135500));
segments.push_back(Segment(-62135500,12135500, -90000000,40000000));
segments.push_back(Segment(-50000000,18700600, -58700600,18700600));
segments.push_back(Segment(-58700600,18700600, -90000000,50000000));
segments.push_back(Segment(-50000000,25620800, -55620800,25620800));
segments.push_back(Segment(-55620800,25620800, -90000000,60000000));
segments.push_back(Segment(-50000000,46057700, -90000000,86057700));
segments.push_back(Segment(-90000000,86057700, -90000000,90000000));

  // Construction of the Voronoi Diagram.
  voronoi_diagram<double> vd;
  construct_voronoi(points.begin(), points.end(),
                    segments.begin(), segments.end(),
                    &vd);
  //construct_voronoi(segments.begin(), segments.end(), &vd);
  // Traversing Voronoi Graph.
  {
    printf("Traversing Voronoi graph.\n");
    printf("Number of visited primary edges using edge iterator: %d\n",
        iterate_primary_edges1(vd));
    printf("\n");
    printf("Number of visited primary edges using cell iterator: %d\n",
        iterate_primary_edges2(vd));
    printf("\n");
    printf("Number of visited primary edges using vertex iterator: %d\n",
        iterate_primary_edges3(vd));
    printf("\n");
  }

  // Using color member of the Voronoi primitives to store the average number
  // of edges around each cell (including secondary edges).
  {
    printf("Number of edges (including secondary) around the Voronoi cells:\n");
    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
         it != vd.edges().end(); ++it) {
      std::size_t cnt = it->cell()->color();
      it->cell()->color(cnt + 1);
    }
    for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
         it != vd.cells().end(); ++it) {
      printf("%lu ", it->color());
    }
    printf("\n");
    printf("\n");
  }
  //traverses the voroni edges around the voronoi cell and prints the coordinates of the edges
  /*
  const voronoi_diagram<double>::voronoi_edge<double>* edge = cell->incident_edge();
do {
  edge = edge->next();
  if (edge->is_primary())
    printf("Edge: (%d, %d) -> (%d, %d)\n",
           x(*edge->vertex0()), y(*edge->vertex0()),
           x(*edge->vertex1()), y(*edge->vertex1()));
  else
    printf("Edge: (%d, %d) -> (%d, %d) (secondary)\n",
           x(*edge->vertex0()), y(*edge->vertex0()),
           x(*edge->vertex1()), y(*edge->vertex1()));
} while (edge != cell->incident_edge());
*/
  // Linking Voronoi cells with input geometries.
  
  /*{
    unsigned int cell_index = 0;
    for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
         it != vd.cells().end(); ++it) {
      if (it->contains_point()) {
        if (it->source_category() ==
            boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
          std::size_t index = it->source_index();
          Point p = points[index];
          printf("Cell #%u contains a point: (%d, %d).\n",
                 cell_index, x(p), y(p));
        } else if (it->source_category() ==
                   boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
          std::size_t index = it->source_index() - points.size();
          Point p0 = low(segments[index]);
          printf("Cell #%u contains segment start point: (%d, %d).\n",
                 cell_index, x(p0), y(p0));
        } else if (it->source_category() ==
                   boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) {
          std::size_t index = it->source_index() - points.size();
          Point p1 = high(segments[index]);
          printf("Cell #%u contains segment end point: (%d, %d).\n",
                 cell_index, x(p1), y(p1));
        }
      } else {
        std::size_t index = it->source_index() - points.size();
        Point p0 = low(segments[index]);
        Point p1 = high(segments[index]);
        printf("Cell #%u contains a segment: ((%d, %d), (%d, %d)). \n",
               cell_index, x(p0), y(p0), x(p1), y(p1));
      }
      ++cell_index;
    }
  }*/
  return 0;
}
