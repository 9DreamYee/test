#include "ParseNetInfoToGurobi.h"
netInfo::netInfo():
    netID(-1),
    pad_x(0.0), pad_y(0.0),
    ball_x(0.0), ball_y(0.0),
    areaInitial(0.0)
    {}

commonBoundary::commonBoundary(): 
    boundaryID(-1),
    netA(-1),netB(-1),
    alpha(0.0),
    shiftAmount(0),shiftMin(0),shiftMax(0)
    {}
double cal_shifted_area(const line_t &line,const double &unit,const int &shifted_direction){
    polygon_t poly;
    line_t shifted_line;
    double area = 0;
    // shifted_direction = 0 -> x
    if (shifted_direction == 0){
        for(const auto &point : line){
            shifted_line.push_back(point_t(point.x() + unit, point.y()));
        }
    }
    // shifted_direction = 1 -> y
    else if (shifted_direction == 1){
        for(const auto &point : line){
            shifted_line.push_back(point_t(point.x(), point.y() + unit));
        }
    }
    else if(shifted_direction == 2){
        return 0;
    }
    for(const auto &point : line){
        bg::append(poly, point);
    }
    for(auto it = shifted_line.rbegin(); it != shifted_line.rend(); it++){
        bg::append(poly, *it);
    }
    bg::correct(poly);
    std::string reason;
    if (!bg::is_valid(poly, reason)) {
        //std::cout << "Polygon is valid" << std::endl;
        std::cout << "Polygon is invalid: " << reason << std::endl;
    }
    area = bg::area(poly)*1e-08;
    return area;

}
//Update CommonBounary info: alpha,shift_Direction,shiftAmount
void UpdateCommonBoundaryInfo(std::vector<commonBoundary> &commonBoundaries){
    std::ifstream file("Gurobi_area_assignment_result.txt");
    std::string line;
    int rows = commonBoundaries.size(), cols = 2;
    int temp_value = 0,row = 0,col = 0;
    double unit = 1;
    //std::vector<std::vector<int>> matrix(rows,std::vector<int> (cols));
    std::vector<std::vector<int>> matrix(rows,std::vector<int>(2));
    //特例處理 commonBoundary Boundary ID 1
    int temp_id = 0;
    point_t temp_point;
    temp_id = commonBoundaries[1].netA;
    commonBoundaries[1].netA = commonBoundaries[1].netB;
    commonBoundaries[1].netB = temp_id;
    temp_point = commonBoundaries[1].netA_pad;
    commonBoundaries[1].netA_pad = commonBoundaries[1].netB_pad;
    commonBoundaries[1].netB_pad = temp_point;
     
    auto modIdx = [&] (int idx){
        return (idx+rows) % rows;
    };
    //shift_Direction shiftMax,shiftMin
    for (auto &commonBoundary:commonBoundaries){
        //在x座標平移
        if(commonBoundary.netA_pad.y() == commonBoundary.netB_pad.y()){
            commonBoundary.shift_Direction = 0;
            //shiftMin = netA_pad.x - startPoint.x 
            //shiftMax = netB_pad.x - startPoint.x
            commonBoundary.shiftMin = commonBoundary.netA_pad.x() - commonBoundary.startPoint.x();
            commonBoundary.shiftMax = commonBoundary.netB_pad.x() - commonBoundary.startPoint.x();
        }
        //在y座標平移
        else if (commonBoundary.netA_pad.x() == commonBoundary.netB_pad.x()){
            commonBoundary.shift_Direction = 1;
            commonBoundary.shiftMin = commonBoundary.netA_pad.y() - commonBoundary.startPoint.y();
            commonBoundary.shiftMax = commonBoundary.netB_pad.y() - commonBoundary.startPoint.y();
        }
        else{
            commonBoundary.shift_Direction = 2;
        }
    }
    //alpha 
    for (auto &commonBoundary:commonBoundaries){
        commonBoundary.alpha = cal_shifted_area(commonBoundary.boundarySegment,unit,commonBoundary.shift_Direction);

    }
    //shiftAmount
    while(std::getline(file,line)){
       std::stringstream ss(line);
       while(ss >> temp_value){
           matrix[row][col] = temp_value;
           col++;
           if(col == cols){
               col = 0;
               row++;
           }
       }
    }
    for(int i = 0;i < rows;i++){
        for(int j = 0;j < cols ;j++){
            int ip1 = modIdx(i+1);
            int im1 = modIdx(i-1);
            for (auto &commonBoundary:commonBoundaries){
                if (matrix[i][0]!= 0){
                  if(commonBoundary.netA == i && commonBoundary.netB == ip1 || commonBoundary.netA == ip1 && commonBoundary.netB == i){
                      commonBoundary.shiftAmount = matrix[i][0];
                  }
                }
                if(matrix[i][1]!=0){
                    if(commonBoundary.netA == i && commonBoundary.netB == im1 || commonBoundary.netA == im1 && commonBoundary.netB == i){
                        commonBoundary.shiftAmount = matrix[i][1];
                    }
                }
            }
            /*
            if(matrix[i][j] != 0){
                int ip1 = modIdx(i+1);
                int im1 = modIdx(i-1);
                std::cout<<"matrix["<<i<<"]["<<j<<"] = "<<matrix[i][j]<<std::endl;
                for(auto &commonBoundary:commonBoundaries){
                    if(commonBoundary.netA == i && commonBoundary.netB == ip1 || commonBoundary.netA == j && commonBoundary.netB == i){
                        commonBoundary.shiftAmount = matrix[i][j];
                    }
                }
            }
            */
        }
    }
    file.close();
}
void outputCommonBoundaries(std::vector<commonBoundary> &commonBoundaries){
    std::ofstream outfile("commonBoundary_toGurobi.txt");
    for(auto commonBoundary:commonBoundaries){
        outfile<<"BoundaryID: "<<commonBoundary.boundaryID<<std::endl;
        outfile<<"NetA: "<<commonBoundary.netA<<std::endl;
        outfile<<"NetB: "<<commonBoundary.netB<<std::endl;
        outfile<<"StartPoint: "<<bg::get<0>(commonBoundary.startPoint)<<" "<<bg::get<1>(commonBoundary.startPoint)<<std::endl;
        outfile<<"BoundarySegment: ";
        for(auto point:commonBoundary.boundarySegment){
            outfile<<bg::get<0>(point)<<" "<<bg::get<1>(point)<<" ";
        }
        outfile<<std::endl;
        outfile<<"NetA_pad: "<<bg::get<0>(commonBoundary.netA_pad)<<" "<<bg::get<1>(commonBoundary.netA_pad)<<std::endl;
        outfile<<"NetB_pad: "<<bg::get<0>(commonBoundary.netB_pad)<<" "<<bg::get<1>(commonBoundary.netB_pad)<<std::endl;
        outfile<<"Alpha: "<<commonBoundary.alpha<<std::endl;
        outfile<<"ShiftDirection: "<<commonBoundary.shift_Direction<<std::endl;
        outfile<<"ShiftMin: "<<commonBoundary.shiftMin<<std::endl;
        outfile<<"ShiftMax: "<<commonBoundary.shiftMax<<std::endl;
        outfile<<"ShiftAmount: "<<commonBoundary.shiftAmount<<std::endl;
        outfile<<std::endl;
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
        outfile<<"ExtendedInitialRoute: ";
        for (const auto & point : net.ExtendedInitialRoute){
            outfile<<point.x()<<" "<<point.y()<<" ";
        }
        outfile<<std::endl;
        outfile<<"boundarySegment.size() = "<<net.boundarySegments.size()<<std::endl;
        outfile<<"startPoint0:"<<bg::get<0>(net.startPoint0)<<" "<<bg::get<1>(net.startPoint0)<<std::endl;
        outfile<<"startPoint1:"<<bg::get<0>(net.startPoint1)<<" "<<bg::get<1>(net.startPoint1)<<std::endl;
        outfile<<"innerBoundarySegment.size() = "<<net.innerBoundarySegments.size()<<std::endl;
        outfile<<"outterBoundarySegment.size() = "<<net.outterBoundarySegments.size()<<std::endl;
        outfile<<"Area Initial: "<<net.areaInitial<<std::endl;
    }
}
/*
bool isSameBoundary(const point_t p, const point_t q){
    auto eq = [&](double a, double b){
        return std::fabs(a-b) < 1e-6;
    };
    bool case1 = eq(p.x(),q.x()) && eq(p.y(),q.y());
    return case1;
}*/

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
        //Extended Initial route
        if(line.rfind("Extended_Initial_route_segment:",0) == 0 && line.find(":") != std::string::npos){
            auto pos = line.find(":");
            currentNet.ExtendedInitialRoute = parseLineString(line.substr(pos+1));
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
    //同時特例處利Boundary 1
    UpdateCommonBoundaryInfo(commonBoundaries);
    outputNetsInfo(nets);
    outputCommonBoundaries(commonBoundaries);
    file.close();
    return 0;
}

