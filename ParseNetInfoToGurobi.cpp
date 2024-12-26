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
    netInfo(): netID(-1),
               pad_x(0.0), pad_y(0.0),
               ball_x(0.0), ball_y(0.0),
               areaInitial(0.0)
    {}
};
struct commonBoundary{
    int boundaryID;
    int netA,netB;
    point_t startPoint;
    line_t boundarySegment;
    point_t netA_pad,netB_pad;
    double shiftAmount;
    commonBoundary(): boundaryID(-1),
                     netA(-1),netB(-1),
                     shiftAmount(0.0)
    {}

};
void outputCommonBoundaries(std::vector<commonBoundary> &commonBoundaries){
    std::ofstream outfile("commonBoundary_toGurobi.txt");
    for(auto commonBoundary:commonBoundaries){
        outfile<<"BoundaryID: "<<commonBoundary.boundaryID<<std::endl;
        outfile<<"NetA: "<<commonBoundary.netA<<std::endl;
        outfile<<"NetB: "<<commonBoundary.netB<<std::endl;
        outfile<<"StartPoint: "<<bg::get<0>(commonBoundary.startPoint)<<" "<<bg::get<1>(commonBoundary.startPoint)<<std::endl;
        /*outfile<<"BoundarySegment: ";
        for(auto point:commonBoundary.boundarySegment){
            outfile<<bg::get<0>(point)<<" "<<bg::get<1>(point)<<" ";
        }*/
        outfile<<std::endl;
        outfile<<"NetA_pad: "<<bg::get<0>(commonBoundary.netA_pad)<<" "<<bg::get<1>(commonBoundary.netA_pad)<<std::endl;
        outfile<<"NetB_pad: "<<bg::get<0>(commonBoundary.netB_pad)<<" "<<bg::get<1>(commonBoundary.netB_pad)<<std::endl;
        outfile<<"ShiftAmount: "<<commonBoundary.shiftAmount<<std::endl;
    }
}
void outputNetsInfo(std::vector<netInfo> &nets){
    std::ofstream outfile("netInfo_toGurobi.txt");
    /*
    for(auto net:nets){
        std::cout<<"NetID: "<<net.netID<<std::endl;
        std::cout<<"Pad Location: "<<net.pad_x<<" "<<net.pad_y<<std::endl;
        std::cout<<"Ball Location: "<<net.ball_x<<" "<<net.ball_y<<std::endl;
        std::cout<<"boundarySegment.size() = "<<net.boundarySegments.size()<<std::endl;
        std::cout<<"startPoint0:"<<bg::get<0>(net.startPoint0)<<" "<<bg::get<1>(net.startPoint0)<<std::endl;
        std::cout<<"startPoint1:"<<bg::get<0>(net.startPoint1)<<" "<<bg::get<1>(net.startPoint1)<<std::endl;
        std::cout<<"innerBoundarySegment.size() = "<<net.innerBoundarySegments.size()<<std::endl;
        std::cout<<"outterBoundarySegment.size() = "<<net.outterBoundarySegments.size()<<std::endl;
        std::cout<<"Area Initial: "<<net.areaInitial<<std::endl;
    }
    */
    for(auto net:nets){
        outfile<<"NetID: "<<net.netID<<std::endl;
        outfile<<"Pad Location: "<<net.pad_x<<" "<<net.pad_y<<std::endl;
        outfile<<"Ball Location: "<<net.ball_x<<" "<<net.ball_y<<std::endl;
        outfile<<"boundarySegment.size() = "<<net.boundarySegments.size()<<std::endl;
        outfile<<"startPoint0:"<<bg::get<0>(net.startPoint0)<<" "<<bg::get<1>(net.startPoint0)<<std::endl;
        outfile<<"startPoint1:"<<bg::get<0>(net.startPoint1)<<" "<<bg::get<1>(net.startPoint1)<<std::endl;
        outfile<<"innerBoundarySegment.size() = "<<net.innerBoundarySegments.size()<<std::endl;
        outfile<<"outterBoundarySegment.size() = "<<net.outterBoundarySegments.size()<<std::endl;
        outfile<<"Area Initial: "<<net.areaInitial<<std::endl;
    }
}
bool isSameBoundary(const point_t p, const point_t q){
    auto eq = [&](double a, double b){
        return std::fabs(a-b) < 1e-6;
    };
    bool case1 = eq(p.x(),q.x()) && eq(p.y(),q.y());
    return case1;
}
std::vector<commonBoundary> buildCommonBoundaries(std::vector<netInfo> &nets){
    std::vector<commonBoundary> commonBoundaries;
    std::vector<point_t> usedPoint;
    int boundary_count = 0;
    for(int i = 0;i < nets.size();i++){ 
        point_t temp_startPoint0 = nets[i].startPoint0;
        point_t temp_startPoint1 = nets[i].startPoint1;
        for(int j = i+1; j < nets.size();j++){
           if(nets[i].startPoint0.x() == nets[j].startPoint0.x() && nets[i].startPoint0.y() == nets[j].startPoint0.y()){
               commonBoundary temp_commonBoundary;
               temp_commonBoundary.boundaryID = boundary_count++;
               temp_commonBoundary.netA = nets[i].netID;
               temp_commonBoundary.netB = nets[j].netID;
               temp_commonBoundary.startPoint = nets[i].startPoint0;
               temp_commonBoundary.boundarySegment = nets[i].boundarySegments[0];
               temp_commonBoundary.netA_pad = point_t(nets[i].pad_x,nets[i].pad_y);
               temp_commonBoundary.netB_pad = point_t(nets[j].pad_x,nets[j].pad_y);
               commonBoundaries.emplace_back(temp_commonBoundary);
           }
           else if(nets[i].startPoint0.x() == nets[j].startPoint1.x() && nets[i].startPoint0.y() == nets[j].startPoint1.y()){
               commonBoundary temp_commonBoundary;
               temp_commonBoundary.boundaryID = boundary_count++;
               temp_commonBoundary.netA = nets[i].netID;
               temp_commonBoundary.netB = nets[j].netID;
               temp_commonBoundary.startPoint = nets[i].startPoint0;
               temp_commonBoundary.boundarySegment = nets[i].boundarySegments[0];
               temp_commonBoundary.netA_pad = point_t(nets[i].pad_x,nets[i].pad_y);
               temp_commonBoundary.netB_pad = point_t(nets[j].pad_x,nets[j].pad_y);
               commonBoundaries.emplace_back(temp_commonBoundary); 
           }
           else if(nets[i].startPoint1.x() == nets[j].startPoint0.x() && nets[i].startPoint1.y() == nets[j].startPoint0.y()){
               commonBoundary temp_commonBoundary;
               temp_commonBoundary.boundaryID = boundary_count++;
               temp_commonBoundary.netA = nets[i].netID;
               temp_commonBoundary.netB = nets[j].netID;
               temp_commonBoundary.startPoint = nets[i].startPoint1;
               temp_commonBoundary.boundarySegment = nets[i].boundarySegments[1];
               temp_commonBoundary.netA_pad = point_t(nets[i].pad_x,nets[i].pad_y);
               temp_commonBoundary.netB_pad = point_t(nets[j].pad_x,nets[j].pad_y);
               commonBoundaries.emplace_back(temp_commonBoundary);
           }
           else if(nets[i].startPoint1.x() == nets[j].startPoint1.x() && nets[i].startPoint1.y() == nets[j].startPoint1.y()){
               commonBoundary temp_commonBoundary;
               temp_commonBoundary.boundaryID = boundary_count++;
               temp_commonBoundary.netA = nets[i].netID;
               temp_commonBoundary.netB = nets[j].netID;
               temp_commonBoundary.startPoint = nets[i].startPoint1;
               temp_commonBoundary.boundarySegment = nets[i].boundarySegments[1];
               temp_commonBoundary.netA_pad = point_t(nets[i].pad_x,nets[i].pad_y);
               temp_commonBoundary.netB_pad = point_t(nets[j].pad_x,nets[j].pad_y);
               commonBoundaries.emplace_back(temp_commonBoundary);
           }
        }
    }
    return commonBoundaries;
}
std::vector<netInfo> parseNetsInfo(std::ifstream &file){
    std::vector<netInfo> nets;
    std::set<point_t> startPoints;
    netInfo currentNet;
    commonBoundary currentCommonBoundary;
    bool inNetSection = false;
    //Parsing coord to linestring lambda function
    auto parseLineString = [&](const std::string coords) -> line_t{
        line_t ls;
        std::stringstream ss(coords);
        double val;
        std::vector<double> nums;
        while(ss >> val){
            nums.emplace_back(val);
        }
        for(size_t i = 0; i < nums.size();i+=2){
            bg::append(ls,point_t(nums[i],nums[i+1]));
        }
        return ls;
    };
    auto parseStartPoint = [&](const std::string coords) -> point_t{
        point_t startPoint;
        std::stringstream ss(coords);
        double x,y;
        ss >> x >> y;
        return point_t(x,y);
    };
    std::string line;
    while(std::getline(file,line)){
        if(line.empty()) continue;
        //trimming  whitespace & \t
        auto startPos = line.find_first_not_of(" \t");
        if(startPos == std::string ::npos) continue;
        line.erase(0,startPos);
        //Detect Netx:
        if(line.rfind("Net",0) == 0 && line.find(":")!= std::string::npos && line.find("_area") == std::string::npos){
            if(inNetSection){
                nets.emplace_back(currentNet);
                currentNet = netInfo();
                //commonBoundaries.emplace_back(currentCommonBoundary);
            }
            inNetSection = true;
            //parse netID
            size_t posColon = line.find(":");
            std::string afterNet = line.substr(3,posColon-3); //get net ID  "0","15" etc
            currentNet.netID = std::stoi(afterNet);
            continue;
        }
        //pad_loaction
        if(line.rfind("pad_location",0) == 0 && line.find(":")!= std::string::npos){
            std::stringstream ss(line.substr(13));
            ss >> currentNet.pad_x >> currentNet.pad_y;
            continue;
        }
        //ball_location
        if(line.rfind("ball_location",0) == 0 && line.find(":")!=std::string::npos){
            std::stringstream ss (line.substr(14));
            ss>>currentNet.ball_x>>currentNet.ball_y;
            continue;
        }
        //Initail route
        if(line.rfind("Initial_route_segment:",0) == 0 && line.find(":")!= std::string::npos){
            auto pos = line.find(":");
            currentNet.InitialRoute = parseLineString(line.substr(pos+1));
            continue;
        }
        // Boundary_segmnet_? info_start -> ignore
        if(line.rfind("Boundary_segment_",0) == 0 && line.find(":")!= std::string::npos){
            continue;
        }
        //InnerBoundart_segment_? info_start -> ignore
        if(line.rfind("InnerBoundary_segment_",0) == 0 && line.find(":")!= std::string::npos){
            continue;
        }
        //OutterBoundary_segment_? info_start -> ignore
        if(line.rfind("OutterBoundary_segment_",0) == 0 && line.find(":")!= std::string::npos){
            continue;
        }
        
        //Boundary_0_startPoint
        if(line.rfind("Boundary_0_startPoint",0) == 0){
            auto pos = line.find(":");
            point_t startPoint = parseStartPoint(line.substr(pos+1));
            currentNet.startPoint0 = startPoint;
            continue;
        }
        //Boundary_1_startPoint
        if(line.rfind("Boundary_1_startPoint",0) == 0){
            auto pos = line.find(":");
            point_t startPoint = parseStartPoint(line.substr(pos+1));
            currentNet.startPoint1 = startPoint;
            continue;
        }
        //Boundary_x_segment
        if(line.rfind("Boundary_",0) == 0 && line.find("_segment:")!= std::string::npos){
            auto pos = line.find(":");
            if(pos != std::string::npos){
                std::string coords = line.substr(pos+1);
                line_t ls = parseLineString(coords);
                currentNet.boundarySegments.emplace_back(ls);
            }
            continue;
        }
        //InnerBoundary_x_segment
        if(line.find("InnerBoundary_",0) == 0 && line.find("_segment:")!= std::string::npos){
            auto pos = line.find(":");
            if(pos != std::string::npos){
                std::string coords = line.substr(pos+1);
                line_t ls = parseLineString(coords);
                currentNet.innerBoundarySegments.emplace_back(ls);
            }
            continue;
        }
        //OutterBoundary_x_segment
        if(line.rfind("OutterBoundary_",0) == 0 && line.find("_segment:")!= std::string::npos){
            auto pos = line.find(":");
            if(pos != std::string::npos){
                std::string coords = line.substr(pos+1);
                line_t ls = parseLineString(coords);
                currentNet.outterBoundarySegments.emplace_back(ls);
            }
            continue;
        }
        //_area
        if(line.find("area") != std::string::npos){
            auto pos = line.find(":");
            if (pos!=std::string::npos){
                std::string coords = line.substr(pos+1);
                double val = std::stod(coords);
                currentNet.areaInitial = val;
            }
            continue;
        }
    }
    if(inNetSection){
        nets.emplace_back(currentNet);
    }
    return nets;
}
int main(int argc, char* argv[]){
    std::ifstream file(argv[1]);
    auto nets = parseNetsInfo(file);
    auto commonBoundaries = buildCommonBoundaries(nets);
    file.close();
    file.open(argv[1]);
    outputNetsInfo(nets);
    outputCommonBoundaries(commonBoundaries);
    file.close();
    return 0;
}
