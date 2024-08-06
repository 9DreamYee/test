#include "CalculatePolygonArea.h"
//Inputfile that need contain tuple info of each net
void Inputfile_CalculateAreaofNet(std::ifstream& file,std::vector<Polygon_g> &polygons){
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
            Polygon_g poly;
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
//Ouput the result to CSV file ; each area*1e-8 because the resolution is 1e-4, so the area is 1e-8
void Outputfile_AreaResult_toCSV(std::vector<Polygon_g> &polygons){
    std::ofstream outfile("polygon_areas.csv");
    if(!outfile.is_open()){
        std::cerr<<"Cannot open file to write"<<std::endl;
    }
    outfile<<"Polygon Number,Area"<<std::endl;
    for(int i = 0; i < polygons.size();i++){
        boost::geometry::correct(polygons[i]);
        //std::cout<<i<<","<<std::scientific<<std::setprecision(5)<<boost::geometry::area(polygons[i])*1e-8<<std::endl;
        outfile<<i<<","<<std::scientific<<std::setprecision(5)<<boost::geometry::area(polygons[i])*1e-8<<std::endl;
    }
    outfile.close();
}
void CalculateAreaofNet(std::vector<Polygon_g> &polygons,std::vector<double> &area_result){
    for(int i = 0; i < polygons.size();i++){
        boost::geometry::correct(polygons[i]);
        area_result.emplace_back(boost::geometry::area(polygons[i])*1e-8); 
    }
}
/*
int main(int argc, char** argv){
    std::ifstream file(argv[1]);
    std::vector<Polygon_g> polygons;
    InputfileCalculateAreaofNet(file,polygons);
    Outputfile_AreaResult_toCSV(polygons);
}*/
