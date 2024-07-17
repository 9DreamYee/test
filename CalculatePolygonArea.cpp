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

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/core/no_exceptions_support.hpp>

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)

#include <boost/assign.hpp>
using boost::assign::tuple_list_of;
typedef boost::geometry::model::polygon<boost::tuple<int,int>> Polygon;
void InputfileCalculateAreaofNet(std::ifstream& file,std::vector<Polygon> &polygons){
    std::string str,str_x,str_y;
    std::stringstream ss;
    std::vector<boost::tuple<int,int> > tuple_of_net;
    int x,y;
    char dummy;
    bool isNetTuple = false;
    while(std::getline(file,str)){
        if(str == "Net's tuple begin:"){
            isNetTuple = true;
            continue;
        }
        if(str == "Net's tuple end"){
            Polygon poly;
            for(int i = 0; i < tuple_of_net.size();i++){
                boost::geometry::exterior_ring(poly).emplace_back(tuple_of_net[i]);
            }
            polygons.emplace_back(poly);
            isNetTuple = false;
            tuple_of_net.clear();
            continue;
        }
        if(isNetTuple){
            std::istringstream iss(str);
            iss >> dummy >> x >> dummy >> y >> dummy;
            tuple_of_net.emplace_back(boost::make_tuple(x,y));
        }
    }
}
void Ouptufile_AreaResult_toCSV(std::vector<Polygon> &polygons){
    std::ofstream outfile("polygon_areas.csv");
    if(!outfile.is_open()){
        std::cerr<<"Cannot open file to write"<<std::endl;
    }
    outfile<<"Polygon Number,Area"<<std::endl;
    for(int i = 0; i < polygons.size();i++){
        boost::geometry::correct(polygons[i]);
        outfile<<i<<","<<std::scientific<<std::setprecision(5)<<boost::geometry::area(polygons[i])<<std::endl;
    }
    outfile.close();
}
int main(int argc, char** argv){
    std::ifstream file(argv[1]);
    std::vector<Polygon> polygons;
    InputfileCalculateAreaofNet(file,polygons);
    Ouptufile_AreaResult_toCSV(polygons);
    /*for(int i = 0; i < polygons.size();i++){
        //std::cout<<"polygon before corrected: "<<boost::geometry::dsv(polygons[i])<<std::endl;
        //std::cout<<"Area before corrected of polygon "<<i<<" is: "<<boost::geometry::area(polygons[i])<<std::endl;
        boost::geometry::correct(polygons[i]);
        //std::cout<<"polygon After corrected: "<<boost::geometry::dsv(polygons[i])<<std::endl;
        std::cout<<"Area of polygon "<<i<<" is: "<<boost::geometry::area(polygons[i])<<std::endl;
    }*/
}