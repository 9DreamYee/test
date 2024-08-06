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
#include "net.h"
#include "CalculatePolygonArea.h"
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
void FindStartAndEndPoint(std::vector<Segment> &one_net,Point &start,Point &end, Point &InnerRect_CenterPoint,bool &all_net_info_end){
    std::vector<Point> net_points;
    int max = std::numeric_limits<int>::min();
    int distance = 0;
    Point temp_start(0,0);
    Point temp_end(0,0);
    int distance_BetweenStartAndCenter = 0;
    int distance_BetweenEndAndCenter = 0;
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
    //Check the start point and the end point which is closer to the InnerRect_CenterPoint, closer one is the start point
    distance_BetweenStartAndCenter = boost::polygon::manhattan_distance(temp_start,InnerRect_CenterPoint);
    distance_BetweenEndAndCenter = boost::polygon::manhattan_distance(temp_end,InnerRect_CenterPoint);
    if(!all_net_info_end){
        if(distance_BetweenStartAndCenter > distance_BetweenEndAndCenter){
            start = temp_end;
            end = temp_start;
            std::reverse(one_net.begin(),one_net.end());
            for(int i = 0;i < one_net.size();i++){
                Point temp_point(0,0);
                temp_point.a = one_net[i].p0.a;
                temp_point.b = one_net[i].p0.b;
                one_net[i].p0.a = one_net[i].p1.a;
                one_net[i].p0.b = one_net[i].p1.b;
                one_net[i].p1.a = temp_point.a;
                one_net[i].p1.b = temp_point.b;
            }
        }
        else{
        start = temp_start;
        end = temp_end;
        }
    }
    //std::cout<<"distance_BetweenStartAndCenter: "<<distance_BetweenStartAndCenter<<"distance_BetweenEndAndCenter: "<<distance_BetweenEndAndCenter<<std::endl;
    else if (all_net_info_end){
        start = temp_start;
        end = temp_end;
        
    }
}//end of FindStartAndEndPoint
void CreatingNet(std::vector<Segment> &one_net,Point &start,Point &end,std::vector<Net> &nets){
    Net net(start,end);
    //Need to fix net 35 bug 
    //1.correct start/end point 2.correct the net35 Netsegments
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
void InputFile_ExtendBoundaryToOutterRect_result(std::ifstream &file,std::vector<Segment> &AllBoundarySegments,std::vector<Segment> &AllnetsSegments,std::vector<Net> &nets,std::vector<Boundary> &boundaries,Rectangle &InnerRect){
    std::string str,Net_filiter,Boundary_filiter;
    std::stringstream ss;
    std::vector<Segment> one_net;
    std::vector<Segment> one_boundary;
    double start_x,start_y,end_x,end_y;
    bool all_net_info_end= false;
    int cnt = 1,net_num = 0, boundary_num = 0,ID = 0;
    Point start(0,0);
    Point end(0,0);
    Point InnerRect_CenterPoint(0,0);
    InnerRect_CenterPoint.a = (InnerRect.rect_x + InnerRect.rect_w)/2;
    InnerRect_CenterPoint.b = (InnerRect.rect_y + InnerRect.rect_h)/2;
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
            FindStartAndEndPoint(one_net,start,end,InnerRect_CenterPoint,all_net_info_end);
            CreatingNet(one_net,start,end,nets);
            one_net.clear();
            continue;
        } 
         //processing net Info & Creating net 
        if(!all_net_info_end){
            Net_filiter = "Net" + std::to_string(cnt) + ":";
            if(str == Net_filiter){
                cnt++;
                FindStartAndEndPoint(one_net,start,end,InnerRect_CenterPoint,all_net_info_end);
                CreatingNet(one_net,start,end,nets);
                one_net.clear();
                continue;
            }

            ProcessingSegmentInfo(str,start_x,start_y,end_x,end_y);
            Segment net(start_x,start_y,end_x,end_y); 
            AllnetsSegments.emplace_back(net);
            one_net.emplace_back(net);
            }
        //Processing boundary Info 
        else if(all_net_info_end){
            if(boundary_num){
                Boundary_filiter = "boundary" + std::to_string(cnt) + ":";
                if(str == Boundary_filiter){
                    ID = cnt-1;
                    cnt++;
                    FindStartAndEndPoint(one_boundary,start,end,InnerRect_CenterPoint,all_net_info_end);
                    CreatingBoundary(one_boundary,end,start,ID,boundaries);
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
     //processing the last boundary info
    ID = cnt-1;
    FindStartAndEndPoint(one_boundary,start,end,InnerRect_CenterPoint,all_net_info_end);
    CreatingBoundary(one_boundary,end,start,ID,boundaries);
    one_boundary.clear();
}
/*void OutputFile_OctilinearizeBoundary(std::vector<Net> &nets){
    std::ofstream outfile("OctilinearizeBoundary_result.txt");
    for (int i = 0;i < nets.size();i++){
        outfile<<"Net"<<i<<":\n"<<"Net"<<i<<"_startPoint: "<<nets[i].startPoint.a<<" "<<nets[i].startPoint.b<<" Net"<<i<<"_endPoint: "<<nets[i].endPoint.a<<" "<<nets[i].endPoint.b<<"\n";
        outfile<<"Net"<<i<<"_segments_number: "<<nets[i].NetSegments.size()<<"\n";
        outfile<<"Net"<<i<<":\n";
        for(int j = 0;j < nets[i].NetSegments.size();j++){
            outfile<<"Net_segment: "<<nets[i].NetSegments[j].p0.a<<" "<<nets[i].NetSegments[j].p0.b<<" "<<nets[i].NetSegments[j].p1.a<<" "<<nets[i].NetSegments[j].p1.b<<"\n";
        }
        outfile<<"Net's boundaries:\n";
        int nets_BoundaryNumer = nets[i].Boundaries.size() + nets[i].OutterRectBoundarySegments.size() + nets[i].InnerRectBoundarySegments.size();
        outfile<<"Net"<<i<<"_Boundaries_number: "<<nets_BoundaryNumer<<"\n";
        for(int j = 0;j < nets[i].Boundaries.size();i++){
            outfile<<"Boundary_"<<j<<"_startPoint: "<<nets[i].Boundaries[j].startPoint.a<<" "<<nets[i].Boundaries[j].startPoint.b;
            outfile<<" Boundary_"<<j<<"_endPoint: "<<nets[i].Boundaries[j].endPoint.a<<" "<<nets[i].Boundaries[j].endPoint.b<<"\n";
            for(int k = 0;k < nets[i].Boundaries[j].BoundarySegments.size();k++){
                outfile<<"Boundary_"<<j<<"_segment: "<< nets[i].Boundaries[j].BoundarySegments[k].p0.a<<" "<<nets[i].Boundaries[j].BoundarySegments[k].p0.b<<" "<<
                nets[i].Boundaries[j].BoundarySegments[k].p1.a<<" "<<nets[i].Boundaries[j].BoundarySegments[k].p1.b<<"\n";
            }
        }
        for(int j = 0;j < nets[i].InnerRectBoundarySegments.size();i++){
            outfile<<"InnerBoundary_"<<j<<"_startPoint: "<<nets[i].InnerRectBoundarySegments[j].startPoint.a<<" "<<nets[i].InnerRectBoundarySegments[j].startPoint.b;
            outfile<<" InnerBoundary_"<<j<<"_endPoint: "<<nets[i].InnerRectBoundarySegments[j].endPoint.a<<" "<<nets[i].InnerRectBoundarySegments[j].endPoint.b<<"\n";
            for(int k = 0;k < nets[i].InnerRectBoundarySegments[j].BoundarySegments.size();j++){
                outfile<<"InnerBoundary_"<<i<<"_segment: "<< nets[i].InnerRectBoundarySegments[j].BoundarySegments[k].p0.a<<" "<<nets[i].InnerRectBoundarySegments[j].BoundarySegments[k].p0.b<<" "<<
                    nets[i].InnerRectBoundarySegments[j].BoundarySegments[k].p1.a<<" "<<nets[i].InnerRectBoundarySegments[j].BoundarySegments[k].p1.b<<"\n";
            }
        }
        for(int j = 0; j < nets[i].OutterRectBoundarySegments.size();i++){
            outfile<<"OutterBoundary_"<<j<<"_startPoint: "<<nets[i].OutterRectBoundarySegments[j].startPoint.a<<" "<<nets[i].OutterRectBoundarySegments[j].startPoint.b;
            outfile<<" OutterBoundary_"<<j<<"_endPoint: "<<nets[i].OutterRectBoundarySegments[j].endPoint.a<<" "<<nets[i].OutterRectBoundarySegments[j].endPoint.b<<"\n";
            for(int k = 0;k < nets[i].OutterRectBoundarySegments[j].BoundarySegments.size();j++){
                outfile<<"OutterBoundary_"<<j<<"_segment: "<< nets[i].OutterRectBoundarySegments[j].BoundarySegments[k].p0.a<<" "<<nets[i].OutterRectBoundarySegments[j].BoundarySegments[k].p0.b<<" "<<
                        nets[i].OutterRectBoundarySegments[j].BoundarySegments[k].p1.a<<" "<<nets[i].OutterRectBoundarySegments[j].BoundarySegments[k].p1.b<<"\n";
            }
        }
    }
}*/
void FindBoundariesForNet(std::vector<Net> &nets,std::vector<Boundary> &boundaries){
    Point Net_startPoint(0,0);
    Point Boundary_startPoint(0,0);
    int Min = std::numeric_limits<int>::max(),Min_Index = 0;
    int SecondMin = std::numeric_limits<int>::max(),SecondMin_Index = 0;
    std::vector<int> DistanceBetweenNetAndBoundary;
    for(int i = 0;i < nets.size();i++){
        Min = std::numeric_limits<int>::max();
        SecondMin = std::numeric_limits<int>::max();
        Min_Index = 0;
        SecondMin_Index = 0;
        Net_startPoint = nets[i].startPoint;
        //for each boundary calculate the distance between net's start point and boundary's start point
        for(int j = 0;j < boundaries.size();j++){
            Boundary_startPoint = boundaries[j].startPoint;
            DistanceBetweenNetAndBoundary.emplace_back(boost::polygon::manhattan_distance(Net_startPoint,Boundary_startPoint));
        }
        //find the min and second min distance between net's start point and boundary's start point
        for(int i = 0;i < DistanceBetweenNetAndBoundary.size();i++){
            if(DistanceBetweenNetAndBoundary[i] < Min){
                SecondMin = Min;
                Min = DistanceBetweenNetAndBoundary[i];
                SecondMin_Index = Min_Index;
                Min_Index = i;
            }
            else if(DistanceBetweenNetAndBoundary[i] < SecondMin && DistanceBetweenNetAndBoundary[i] != Min){
                SecondMin = DistanceBetweenNetAndBoundary[i];
                SecondMin_Index = i;
            }
        }
        //Add the boundary to the net's member 
        nets[i].AddBoundary(boundaries[Min_Index].startPoint,boundaries[Min_Index].endPoint,
                boundaries[Min_Index].BoundaryID,boundaries[Min_Index].BoundarySegments);
        nets[i].AddBoundary(boundaries[SecondMin_Index].startPoint,boundaries[SecondMin_Index].endPoint,
                boundaries[SecondMin_Index].BoundaryID,boundaries[SecondMin_Index].BoundarySegments);
        DistanceBetweenNetAndBoundary.clear();
    } 
}//end of FindBoundariesForNet
 //Find the Nearest two nets for each boundary
void FindNetNearBoundary(std::vector<Net> nets,Net &NetNearBoundary0,Net &NetNearBoundary1,int BoundaryID){
    bool NetNearBoundary0Flag = false;
    for(int i = 0;i < nets.size();i++){
        if(!NetNearBoundary0Flag){
            if(nets[i].Boundaries[0].BoundaryID == BoundaryID){
                NetNearBoundary0 = nets[i];
                NetNearBoundary0Flag = true;
            }
            else if(nets[i].Boundaries[1].BoundaryID == BoundaryID){
                NetNearBoundary0 = nets[i];
                NetNearBoundary0Flag = true;
            }
        }
       else if(NetNearBoundary0Flag){
            if(nets[i].Boundaries[0].BoundaryID == BoundaryID){
                NetNearBoundary1 = nets[i];
                break;
            }
            else if(nets[i].Boundaries[1].BoundaryID == BoundaryID){
                NetNearBoundary1 = nets[i];
                break;
            }
        }
    }
}//end of FindNetNearBoundary
int DirectionBoundaryToExtend(Boundary &boundary,Rectangle &OutterRect){
    // 1 for top ,2 for bottom, 3 for left, 4 for right
    int direction = 0;
    if(boundary.endPoint.b == OutterRect.rect_y + OutterRect.rect_h){
        direction = 1;
    }
    else if (boundary.endPoint.b == OutterRect.rect_y){
        direction = 2;
    }
    else if (boundary.endPoint.a == OutterRect.rect_x){
        direction = 3;
    }
    else if(boundary.endPoint.a == OutterRect.rect_x + OutterRect.rect_w){
        direction = 4;
    }
    return direction;
}
int CalculateSlope(Net &net,Segment &StartSegment){
    // 1 for 45 degree, -1 for 135 degree, 0 for 90 degree, 2 for horizontal 
    int slope = 0;
    Point start(0,0);
    start = net.startPoint;
    //Find the StartSegment
    for(int i = 0;i<net.NetSegments.size();i++){
        if((net.NetSegments[i].p0.a == start.a && net.NetSegments[i].p0.b == start.b)||(net.NetSegments[i].p1.a == start.a && net.NetSegments[i].p1.b == start.b)){
            StartSegment = net.NetSegments[i];
            break;
        }
    }
   //cal StartSegment's slope
   //x is the same -> 90 degree & slope = 0
    if(StartSegment.p0.a == StartSegment.p1.a){
        slope = 0;
    }
    //y is the same -> horizontal & slope = 2
    else if (StartSegment.p0.b == StartSegment.p1.b){
        slope = 2;
    }
    else{
        slope = (StartSegment.p1.b - StartSegment.p0.b) / (StartSegment.p1.a - StartSegment.p0.a);
    }
    return slope;
}
void SegmentWhichIsAbuts(std::vector<Segment> &segments,Segment &Source,Segment &Target){
    for(int i = 1;i < segments.size();i++){
        bool isAbuts = false;
        isAbuts = boost::polygon::abuts(Source,segments[i]);
        if(isAbuts){
            Target = segments[i];
            break;
        }
    }
}//end of SegmentWhichIsAbuts
void ExtendBoundaryToOutterRect(Boundary &boundary,Rectangle &OutterRect,int &direction,Point &temp_endPoint){
    int BoundarySegmentSize = boundary.BoundarySegments.size()-1;
    Segment LastSegment = boundary.BoundarySegments[BoundarySegmentSize];
    Segment temp_segment(LastSegment.p1.a,LastSegment.p1.b,0,0);
    //1 for top, 2 for bottom, 3 for left, 4 for right
    switch (direction){
        case 1:
            temp_segment.p1.a = LastSegment.p1.a;
            temp_segment.p1.b = OutterRect.rect_y + OutterRect.rect_h;
            break;
        case 2:
            temp_segment.p1.a = LastSegment.p1.a;
            temp_segment.p1.b = OutterRect.rect_y;
            break;
        case 3:
            temp_segment.p1.a = OutterRect.rect_x;
            temp_segment.p1.b = LastSegment.p1.b;
            break;
        case 4:
            temp_segment.p1.a = OutterRect.rect_x + OutterRect.rect_w;
            temp_segment.p1.b = LastSegment.p1.b;

    }
    temp_endPoint.a = temp_segment.p1.a;
    temp_endPoint.b = temp_segment.p1.b;
    boundary.BoundarySegments.emplace_back(temp_segment);
}
void OctilinearizeBoundaryCase_BothNetsAreVertical(Boundary &boundary,Net &temp_net,Segment &temp_segment,Point &temp_point){
    int SecondLineSlope = 0;
    int dx = 0,dy = 0;
    Segment FirstLine(0,0,0,0);
    Segment SecondLine(0,0,0,0);
    Segment AbutswithStartSegment(0,0,0,0);
    boundary.BoundarySegments.clear();
    //first line
    temp_point.a = boundary.startPoint.a;
    FirstLine.p0.a = boundary.startPoint.a;
    FirstLine.p0.b = boundary.startPoint.b;
    FirstLine.p1.a = temp_point.a;
    FirstLine.p1.b = temp_point.b;
    boundary.BoundarySegments.emplace_back(FirstLine);
    //Find Segment which abuts with startSegment
    SegmentWhichIsAbuts(temp_net.NetSegments,temp_segment,AbutswithStartSegment);
    dx = AbutswithStartSegment.p0.a - AbutswithStartSegment.p1.a;
    dy = AbutswithStartSegment.p0.b - AbutswithStartSegment.p1.b;
    if(dy != 0){
            SecondLineSlope = dy/dx;
    }
    //second line
    temp_point.b = AbutswithStartSegment.p1.b;
    //temp_point.a, x2 = x1+(y2-y1)/slope
    temp_point.a = FirstLine.p1.a + (temp_point.b-FirstLine.p1.b)/SecondLineSlope;
    SecondLine.p0.a = FirstLine.p1.a;
    SecondLine.p0.b = FirstLine.p1.b;
    SecondLine.p1.a = temp_point.a;
    SecondLine.p1.b = temp_point.b;
    boundary.BoundarySegments.emplace_back(SecondLine);
}//end of OctilinearizeBoundaryCase_BothNetsAreVertical

void OctilinearizeBoundaryCase_BothNetsAreHorizontal(Boundary &boundary,Net &temp_net,Segment &temp_segment,Point &temp_point){
    int SecondLineSlope = 0;
    int dx = 0, dy = 0;
    Segment FirstLine(0,0,0,0);
    Segment SecondLine(0,0,0,0);
    Segment AbutswithStartSegment(0,0,0,0);
    boundary.BoundarySegments.clear();
    //first line
    temp_point.b = boundary.startPoint.b;
    FirstLine.p0.a = boundary.startPoint.a;
    FirstLine.p0.b = boundary.startPoint.b;
    FirstLine.p1.a = temp_point.a;
    FirstLine.p1.b = temp_point.b;
    boundary.BoundarySegments.emplace_back(FirstLine);
    //Find Segment which abuts with startSegment
    SegmentWhichIsAbuts(temp_net.NetSegments,temp_segment,AbutswithStartSegment);
    dx = AbutswithStartSegment.p0.a - AbutswithStartSegment.p1.a;
    dy = AbutswithStartSegment.p0.b - AbutswithStartSegment.p1.b;
    if(dy != 0){
        SecondLineSlope = dy/dx;
        }
    //second line
    temp_point.a = AbutswithStartSegment.p1.a;
    //temp_point.b, y2 = (slope*x2)-(slope*x1)+y1
    temp_point.b = (SecondLineSlope * temp_point.a)-(SecondLineSlope * FirstLine.p1.a) + FirstLine.p1.b;
    SecondLine.p0.a = FirstLine.p1.a;
    SecondLine.p0.b = FirstLine.p1.b;
    SecondLine.p1.a = temp_point.a;
    SecondLine.p1.b = temp_point.b;
    boundary.BoundarySegments.emplace_back(SecondLine);
}
void OctilinearizeBoundaryCase_BothNetsAre45or135Degree(Boundary &boundary,Net &temp_net,Segment &temp_segment,Point &temp_point){
    int temp_netSegmentNumber = temp_net.NetSegments.size();
    Segment FirstLine(0,0,0,0);
    boundary.BoundarySegments.clear();
    
    if(temp_netSegmentNumber > 2){
        int SecondLineSlope = 0;
        bool AbutswithStartSegmentIsHorizontal = false;
        Segment SecondLine(0,0,0,0);
        Segment AbutswithStartSegment(0,0,0,0);
        int slope = 0;
        int dx = temp_segment.p0.a - temp_segment.p1.a;
        int dy = temp_segment.p0.b - temp_segment.p1.b;
        if(dy != 0){
            slope = dy/dx;
        }

        //First Line
        FirstLine.p0.a = boundary.startPoint.a;
        FirstLine.p0.b = boundary.startPoint.b;
        FirstLine.p1.a = boundary.startPoint.a  + (temp_point.b - boundary.startPoint.b) / slope;
        FirstLine.p1.b = temp_point.b;
        //未來可能還要考慮更多條件 例如boundary的startpoint在第一象限,net的startpoint在第三象限 p1.b位移的量等等
        if(slope == 1){
            if(temp_net.startPoint.a < 0 && temp_net.startPoint.b < 0 && boundary.startPoint.a < 0 && boundary.startPoint.b < 0){
                    FirstLine.p1.b = temp_point.b + 2000000;
                    FirstLine.p1.a = boundary.startPoint.a  + (temp_point.b - boundary.startPoint.b) / slope;
            }
            else if(temp_net.startPoint.a > 0 && temp_net.startPoint.b > 0 && boundary.startPoint.a > 0 && boundary.startPoint.b > 0){
                FirstLine.p1.b = temp_point.b - 2000000;
                FirstLine.p1.a = boundary.startPoint.a  + (temp_point.b - boundary.startPoint.b) / slope;
            }
        }    
        boundary.BoundarySegments.emplace_back(FirstLine);
        //Second line
        SegmentWhichIsAbuts(temp_net.NetSegments,temp_segment,AbutswithStartSegment);
        //AbutswithStartSegmnet is vertical
        if (AbutswithStartSegment.p0.a == AbutswithStartSegment.p1.a){
            SecondLineSlope = 0;
        }
        //AbutswithStartSegment is horizontal
        else if(AbutswithStartSegment.p0.b == AbutswithStartSegment.p1.b){
            SecondLineSlope = 0;
            AbutswithStartSegmentIsHorizontal = true;
        }
        else{
            dx = AbutswithStartSegment.p0.a - AbutswithStartSegment.p1.a;
            dy = AbutswithStartSegment.p0.b - AbutswithStartSegment.p1.b;
            //std::cout<<"dx "<<dx<<"dy "<<dy<<std::endl;
            if(dy != 0){
                SecondLineSlope = dy/dx;
            }
        }
        //Find y2 = (slope*x2)-(slope*x1)+y1
        if(AbutswithStartSegmentIsHorizontal){
            
        //std::cout<<"boundaryID"<<boundary.BoundaryID<<"boundarystartpoint: "<<boundary.startPoint.a<<" "<<boundary.startPoint.b<<"slope: "<<slope<<"SecondLineSlope: "<<SecondLineSlope<<
          //"AbutswithStartSegmentIsHorizontal"<<AbutswithStartSegmentIsHorizontal<<std::endl;
            temp_point.a = AbutswithStartSegment.p1.a;
            //temp_point.b, y2 = (slope*x2)-(slope*x1)+y1
            temp_point.b = (SecondLineSlope * temp_point.a)-(SecondLineSlope * FirstLine.p1.a) + FirstLine.p1.b;
            SecondLine.p0.a = FirstLine.p1.a;
            SecondLine.p0.b = FirstLine.p1.b;
            SecondLine.p1.a = temp_point.a ;
            SecondLine.p1.b = temp_point.b ;
        
        boundary.BoundarySegments.emplace_back(SecondLine);
        } 
        else{
            temp_point.b = AbutswithStartSegment.p1.b;
            //temp_point.a, x2 = x1+(y2-y1)/slope
            if(SecondLineSlope == 0)
                temp_point.a = FirstLine.p1.a;
            else
                temp_point.a = FirstLine.p1.a + (temp_point.b-FirstLine.p1.b)/SecondLineSlope;
            SecondLine.p0.a = FirstLine.p1.a;
            SecondLine.p0.b = FirstLine.p1.b;
            SecondLine.p1.a = temp_point.a;
            SecondLine.p1.b = temp_point.b;
            boundary.BoundarySegments.emplace_back(SecondLine); 
        }
    }
    //else first line
    else if(temp_netSegmentNumber <= 2) {
        int slope = 0;
        int dx = temp_segment.p0.a - temp_segment.p1.a;
        int dy = temp_segment.p0.b - temp_segment.p1.b;
        if(dy != 0){
            slope = dy/dx;
        }
        FirstLine.p0.a = boundary.startPoint.a;
        FirstLine.p0.b = boundary.startPoint.b;
        FirstLine.p1.a = boundary.startPoint.a + (temp_point.b - boundary.startPoint.b) / slope;
        FirstLine.p1.b = temp_point.b ;
        boundary.BoundarySegments.emplace_back(FirstLine);
    }
}//end of OctilinearizeBoundaryCase_BothNetsAre45or135Degree

void OctilinearizeBoundaryCase_ReferNetIsVertical(Boundary &boundary,Net &temp_net,Segment &temp_segment,Point &temp_point){
    int SecondLineSlope = 0;
    int dx = 0,dy = 0;
    Segment FirstLine(0,0,0,0);
    Segment SecondLine(0,0,0,0);
    Segment AbutswithStartSegment(0,0,0,0);
    boundary.BoundarySegments.clear();
    //first line
    temp_point.a = boundary.startPoint.a;
    FirstLine.p0.a = boundary.startPoint.a;
    FirstLine.p0.b = boundary.startPoint.b;
    FirstLine.p1.a = temp_point.a;
    FirstLine.p1.b = temp_point.b;
    boundary.BoundarySegments.emplace_back(FirstLine);
    //Find Segment which abuts with startSegment
    if(temp_net.NetSegments.size() <= 2){
        SegmentWhichIsAbuts(temp_net.NetSegments,temp_segment,AbutswithStartSegment);
        dx = AbutswithStartSegment.p0.a - AbutswithStartSegment.p1.a;
        dy = AbutswithStartSegment.p0.b - AbutswithStartSegment.p1.b;
        if(dy != 0){
            SecondLineSlope = dy/dx;
        }
        //second line
        temp_point.b = AbutswithStartSegment.p1.b;
        //temp_point.a, x2 = x1+(y2-y1)/slope
        temp_point.a = FirstLine.p1.a + (temp_point.b-FirstLine.p1.b)/SecondLineSlope;
        SecondLine.p0.a = FirstLine.p1.a;
        SecondLine.p0.b = FirstLine.p1.b;
        SecondLine.p1.a = temp_point.a;
        SecondLine.p1.b = temp_point.b;
        boundary.BoundarySegments.emplace_back(SecondLine);
    
    }
    else if(temp_net.NetSegments.size() > 2){ 
        Segment ThirdLine(0,0,0,0);
        SegmentWhichIsAbuts(temp_net.NetSegments,temp_segment,AbutswithStartSegment);
        dx = AbutswithStartSegment.p0.a - AbutswithStartSegment.p1.a;
        dy = AbutswithStartSegment.p0.b - AbutswithStartSegment.p1.b;
        //std::cout<<"boundaryID: "<<boundary.BoundaryID<<"temp_segment: "<<temp_segment.p0.a<<" "<<temp_segment.p0.b<<" "<<temp_segment.p1.a<<" "<<temp_segment.p1.b<<std::endl;
        //std::cout<<"Net_segments: \n";
        if(dy != 0){
            SecondLineSlope = dy/dx;
        }
        //second line
        if(temp_net.startPoint.a > 0 && temp_net.startPoint.b > 0 && boundary.startPoint.a > 0 && boundary.startPoint.b > 0){
            temp_point.b = AbutswithStartSegment.p1.b - 1000000;
            //temp_point.a, x2 = x1+(y2-y1)/slope
            temp_point.a = FirstLine.p1.a + (temp_point.b-FirstLine.p1.b)/SecondLineSlope;
            SecondLine.p0.a = FirstLine.p1.a;
            SecondLine.p0.b = FirstLine.p1.b;
            SecondLine.p1.a = temp_point.a;
            SecondLine.p1.b = temp_point.b;
            boundary.BoundarySegments.emplace_back(SecondLine);
        }
        else if(temp_net.startPoint.a < 0 && temp_net.startPoint.b < 0 && boundary.startPoint.a < 0 && boundary.startPoint.b < 0){
            temp_point.b = AbutswithStartSegment.p1.b + 1000000;
            //temp_point.a, x2 = x1+(y2-y1)/slope
            temp_point.a = FirstLine.p1.a + (temp_point.b-FirstLine.p1.b)/SecondLineSlope;
            SecondLine.p0.a = FirstLine.p1.a;
            SecondLine.p0.b = FirstLine.p1.b;
            SecondLine.p1.a = temp_point.a;
            SecondLine.p1.b = temp_point.b;
            boundary.BoundarySegments.emplace_back(SecondLine);
        }
        else{
            temp_point.b = AbutswithStartSegment.p1.b - 1000000;
            //temp_point.a, x2 = x1+(y2-y1)/slope
            temp_point.a = FirstLine.p1.a + (temp_point.b-FirstLine.p1.b)/SecondLineSlope;
            SecondLine.p0.a = FirstLine.p1.a;
            SecondLine.p0.b = FirstLine.p1.b;
            SecondLine.p1.a = temp_point.a;
            SecondLine.p1.b = temp_point.b;
            boundary.BoundarySegments.emplace_back(SecondLine);
        }
        
        //Third line
        Segment temp_segment1(0,0,0,0);
        temp_segment1.p0.a = AbutswithStartSegment.p0.a;
        temp_segment1.p0.b = AbutswithStartSegment.p0.b;
        temp_segment1.p1.a = AbutswithStartSegment.p1.a;
        temp_segment1.p1.b = AbutswithStartSegment.p1.b;
        //std::cout<<"Abuts: "<<AbutswithStartSegment.p0.a<<" "<<AbutswithStartSegment.p0.b<<" "<<AbutswithStartSegment.p1.a<<" "<<AbutswithStartSegment.p1.b<<std::endl;
        SegmentWhichIsAbuts(temp_net.NetSegments,temp_segment1,AbutswithStartSegment);
//        std::cout<<"Abuts: "<<AbutswithStartSegment.p0.a<<" "<<AbutswithStartSegment.p0.b<<" "<<AbutswithStartSegment.p1.a<<" "<<AbutswithStartSegment.p1.b<<std::endl;
        int ThirdSlope = 0;
        bool isHorizontal = false;
        if(AbutswithStartSegment.p0.b == AbutswithStartSegment.p1.b){
            ThirdSlope = 0;
            isHorizontal = true;
         }
        else if(AbutswithStartSegment.p0.a == AbutswithStartSegment.p1.a){
            ThirdSlope = 0;
         }
        else{
            dx = AbutswithStartSegment.p0.a - AbutswithStartSegment.p1.a;
            dy = AbutswithStartSegment.p0.b - AbutswithStartSegment.p1.b;
            if(dy != 0){
                ThirdSlope = dy/dx;
            }
        }
        //std::cout<<"ThirdSlope: "<<ThirdSlope<<"isHorizontal: "<<isHorizontal<<std::endl;
        ThirdLine.p0.a = SecondLine.p1.a;
        ThirdLine.p0.b = SecondLine.p1.b;
        if(ThirdSlope == 0 && isHorizontal){
            ThirdLine.p1.b = SecondLine.p1.b;
            ThirdLine.p1.a = AbutswithStartSegment.p1.a;
            if(temp_net.startPoint.a > 0 && temp_net.startPoint.b > 0 && boundary.startPoint.a > 0 && boundary.startPoint.b > 0){
                ThirdLine.p1.a = AbutswithStartSegment.p1.a + 1000000;    
            }
            else if(temp_net.startPoint.a < 0 && temp_net.startPoint.b < 0 && boundary.startPoint.a < 0 && boundary.startPoint.b < 0){
                ThirdLine.p1.a = AbutswithStartSegment.p1.a - 1000000;
            }
        }
        else if (ThirdSlope == 0 && !isHorizontal){
            ThirdLine.p1.a = SecondLine.p1.a;
            ThirdLine.p1.b = AbutswithStartSegment.p1.b;
            if(temp_net.startPoint.a > 0 && temp_net.startPoint.b > 0 && boundary.startPoint.a > 0 && boundary.startPoint.b > 0){
                ThirdLine.p1.b = AbutswithStartSegment.p1.b + 1000000;    
            }
            else if(temp_net.startPoint.a < 0 && temp_net.startPoint.b < 0 && boundary.startPoint.a < 0 && boundary.startPoint.b < 0){
                ThirdLine.p1.b = AbutswithStartSegment.p1.b - 1000000;
            }
        }
        boundary.BoundarySegments.emplace_back(ThirdLine);
    }
}

void OctilinearzeBoundaryCase_ReferNetIsHorizontal(Boundary &boundary,Net &temp_net,Segment &temp_segment,Point &temp_point){ 
    int SecondLineSlope = 0;
    int dx = 0, dy = 0;
    Segment FirstLine(0,0,0,0);
    Segment SecondLine(0,0,0,0);
    Segment AbutswithStartSegment(0,0,0,0);
    boundary.BoundarySegments.clear();
    //first line
    temp_point.b = boundary.startPoint.b;
    FirstLine.p0.a = boundary.startPoint.a;
    FirstLine.p0.b = boundary.startPoint.b;
    FirstLine.p1.a = temp_point.a;
    FirstLine.p1.b = temp_point.b;
    boundary.BoundarySegments.emplace_back(FirstLine);
    //Find Segment which abuts with startSegment
    SegmentWhichIsAbuts(temp_net.NetSegments,temp_segment,AbutswithStartSegment);
    dx = AbutswithStartSegment.p0.a - AbutswithStartSegment.p1.a;
    dy = AbutswithStartSegment.p0.b - AbutswithStartSegment.p1.b;
    if(dy != 0){
        SecondLineSlope = dy/dx;
        }
    //second line
    temp_point.a = AbutswithStartSegment.p1.a;
    //temp_point.b, y2 = (slope*x2)-(slope*x1)+y1
    temp_point.b = (SecondLineSlope * temp_point.a)-(SecondLineSlope * FirstLine.p1.a) + FirstLine.p1.b;
    SecondLine.p0.a = FirstLine.p1.a;
    SecondLine.p0.b = FirstLine.p1.b;
    SecondLine.p1.a = temp_point.a;
    SecondLine.p1.b = temp_point.b;
    boundary.BoundarySegments.emplace_back(SecondLine);
}
//Octilinearize the boundary
void OctilinearizeBoundary(std::vector<Net> &nets,std::vector<Boundary> &boundaries,Rectangle &OutterRect){
    Point start(0,0);
    Point end(0,0);
    Point temp_point(0,0);
    Point temp_endPoint(0,0);
    int direction = 0;
    for(int i = 0;i < boundaries.size();i++){
        int BoundaryID = boundaries[i].BoundaryID;
        int NetNearBoundary0_StartSegmentSlope = 0,NetNearBoundary1_StartSegmentSlope = 0;
        //Boundary that only has one net
        bool SpecialCase = false;
        Net NetNearBoundary0(start,end);
        Net NetNearBoundary1(start,end);
        Net temp_net(start,end);
        Segment NetNearBoundary0_StartSegment(0,0,0,0);
        Segment NetNearBoundary1_StartSegment(0,0,0,0);
        Segment temp_segment(0,0,0,0);
        FindNetNearBoundary(nets,NetNearBoundary0,NetNearBoundary1,BoundaryID);
        // 1 for 45 degree, -1 for 135 degree, 0 for 90 degree, 2 for horizontal 
        NetNearBoundary0_StartSegmentSlope = CalculateSlope(NetNearBoundary0,NetNearBoundary0_StartSegment);
        NetNearBoundary1_StartSegmentSlope = CalculateSlope(NetNearBoundary1,NetNearBoundary1_StartSegment);
        direction = DirectionBoundaryToExtend(boundaries[i],OutterRect);
        int NetNearBoundary0_StartSegmentLength = boost::polygon::euclidean_distance(NetNearBoundary0_StartSegment.p0,NetNearBoundary0_StartSegment.p1);
        int NetNearBoundary1_StartSegmentLength = boost::polygon::euclidean_distance(NetNearBoundary1_StartSegment.p0,NetNearBoundary1_StartSegment.p1);
        //if net0 < net1 then length = 1 ; else length = 2 
        int length = NetNearBoundary0_StartSegmentLength < NetNearBoundary1_StartSegmentLength ? 1 : 2;
        if(length == 1){
            temp_net = NetNearBoundary0;
            temp_segment = NetNearBoundary0_StartSegment;
            temp_point.a = temp_segment.p1.a;
            temp_point.b = temp_segment.p1.b;
        }
        else if(length == 2){
            temp_net = NetNearBoundary1;
            temp_segment = NetNearBoundary1_StartSegment;
            temp_point.a = temp_segment.p1.a;
            temp_point.b = temp_segment.p1.b;
        }
        //Nets near Boundary that  both are 90 degree (vertical line)
        if((NetNearBoundary0_StartSegmentSlope == 0 && NetNearBoundary1_StartSegmentSlope == 0)){
            
            OctilinearizeBoundaryCase_BothNetsAreVertical(boundaries[i],temp_net,temp_segment,temp_point);
            //Third line    
            ExtendBoundaryToOutterRect(boundaries[i],OutterRect,direction,temp_endPoint);
        }//end of nets that both are 90 drgree case
         
        //Nets near Boundary that both are 0 degree (horizontal line)
        else if(NetNearBoundary0_StartSegmentSlope == 2 && NetNearBoundary1_StartSegmentSlope == 2){
            
            OctilinearizeBoundaryCase_BothNetsAreHorizontal(boundaries[i],temp_net,temp_segment,temp_point);
            //Third line
            ExtendBoundaryToOutterRect(boundaries[i],OutterRect,direction,temp_endPoint);
        }//end of nets that both are 0 degree case
        //Net near Boundary that both are 45/135degree select longer ner to be refer
        else if((NetNearBoundary0_StartSegmentSlope == 1 && NetNearBoundary1_StartSegmentSlope == 1)||
                (NetNearBoundary0_StartSegmentSlope == -1 && NetNearBoundary1_StartSegmentSlope == -1)){
            int Net0_SegmentNumber = NetNearBoundary0.NetSegments.size();
            int Net1_SegmentNumber = NetNearBoundary1.NetSegments.size(); 
            //int NetDistance = boost::polygon::manhattan_distance();
            if(Net0_SegmentNumber == Net1_SegmentNumber){
                length = NetNearBoundary0_StartSegmentLength > NetNearBoundary1_StartSegmentLength ? 1 : 2;
                if(length == 1){
                    temp_net = NetNearBoundary0;
                    temp_segment = NetNearBoundary0_StartSegment;
                    temp_point.a = temp_segment.p1.a;
                    temp_point.b = temp_segment.p1.b;
                    }
                else if(length == 2){
                    temp_net = NetNearBoundary1;
                    temp_segment = NetNearBoundary1_StartSegment;
                    temp_point.a = temp_segment.p1.a;
                    temp_point.b = temp_segment.p1.b;
                }
            }
            else if(Net0_SegmentNumber > Net1_SegmentNumber){
                temp_net = NetNearBoundary0;
                temp_segment = NetNearBoundary0_StartSegment;
                temp_point.a = temp_segment.p1.a;
                temp_point.b = temp_segment.p1.b;
            }
            else if(Net0_SegmentNumber < Net1_SegmentNumber){
                temp_net = NetNearBoundary1;
                temp_segment = NetNearBoundary1_StartSegment;
                temp_point.a = temp_segment.p1.a;
                temp_point.b = temp_segment.p1.b;
            }
            OctilinearizeBoundaryCase_BothNetsAre45or135Degree(boundaries[i],temp_net,temp_segment,temp_point);
            ExtendBoundaryToOutterRect(boundaries[i],OutterRect,direction,temp_endPoint);
        }//end of nets that both are 45/135 degree case
        //Net near Boundary that one is 0/90 degree and one is 45/135 degree
        else{
             if(NetNearBoundary0_StartSegmentSlope == 0 || NetNearBoundary0_StartSegmentSlope == 2){
                temp_net = NetNearBoundary0;
                temp_segment = NetNearBoundary0_StartSegment;
                temp_point.a = temp_segment.p1.a;
                temp_point.b = temp_segment.p1.b;
            } 
            else if(NetNearBoundary1_StartSegmentSlope == 0 || NetNearBoundary1_StartSegmentSlope == 2){
                temp_net = NetNearBoundary1;
                temp_segment = NetNearBoundary1_StartSegment;
                temp_point.a = temp_segment.p1.a;
                temp_point.b = temp_segment.p1.b;
            }
            if(NetNearBoundary0_StartSegmentSlope == 0 || NetNearBoundary1_StartSegmentSlope == 0){
                OctilinearizeBoundaryCase_ReferNetIsVertical(boundaries[i],temp_net,temp_segment,temp_point);
            }
            else if(NetNearBoundary0_StartSegmentSlope == 2 || NetNearBoundary1_StartSegmentSlope == 2){
                OctilinearzeBoundaryCase_ReferNetIsHorizontal(boundaries[i],temp_net,temp_segment,temp_point);
            }
             //std::cout<<"BoundaryID: "<<boundaries[i].BoundaryID<<" temp_net.NetSegments.size(): "<<temp_net.NetSegments.size()<<" NetNearBoundary0_StartSegmentSlope: "<<NetNearBoundary0_StartSegmentSlope<<"  NetNearBoundary1_StartSegmentSlope: "<<NetNearBoundary1_StartSegmentSlope<<std::endl;
            ExtendBoundaryToOutterRect(boundaries[i],OutterRect,direction,temp_endPoint);
        }//end of one is 90 degree and one is 45/135 degree case
        
        //Update each boundary endPoint
        boundaries[i].endPoint = temp_endPoint;
    
    }//end of for(boundaries) loop 

}//end of OctilinearizeBoundary

//Update each boundary's info for each net
void UpdateNetBoundaries(std::vector<Net> &nets,std::vector<Boundary> &boundaries){
    for(int i = 0;i < nets.size();i++){
       int boundary0_ID = nets[i].Boundaries[0].BoundaryID; 
       int boundary1_ID = nets[i].Boundaries[1].BoundaryID;
       nets[i].Boundaries.clear();
       //AddBoundary(Point &start,Point &end,const int &ID, const std::vector<Segment> &BoundarySegments){
       nets[i].AddBoundary(boundaries[boundary0_ID].startPoint,boundaries[boundary0_ID].endPoint,boundary0_ID,boundaries[boundary0_ID].BoundarySegments);
       nets[i].AddBoundary(boundaries[boundary1_ID].startPoint,boundaries[boundary1_ID].endPoint,boundary1_ID,boundaries[boundary1_ID].BoundarySegments);
    }
}
//Find inner boundary for each net
void FindInnerBoundaryForNet(std::vector<Net> &nets,Rectangle &InnerRect){
   Point net_startPoint0(0,0);
   Point net_startPoint1(0,0);
   Point temp_point(0,0);
   Point InnerRect_LeftBottom(InnerRect.rect_x,InnerRect.rect_y);
   Point InnerRect_LeftTop(InnerRect.rect_x,InnerRect.rect_y + InnerRect.rect_h);
   Point InnerRect_RightTop(InnerRect.rect_x + InnerRect.rect_w,InnerRect.rect_y + InnerRect.rect_h);
   Point InnerRect_RightBottom(InnerRect.rect_x + InnerRect.rect_w,InnerRect.rect_y);
   Segment InnerBoundary(0,0,0,0);
   std::vector<Segment> InnerBoundarySegments;
   for(int i = 0; i < nets.size();i++){
       net_startPoint0.a = nets[i].Boundaries[0].startPoint.a;
       net_startPoint0.b = nets[i].Boundaries[0].startPoint.b;
       net_startPoint1.a = nets[i].Boundaries[1].startPoint.a;
       net_startPoint1.b = nets[i].Boundaries[1].startPoint.b;
       // if x coordinate/ y coordinate is the same, the inner boundary is composed of the start points of the two boundaries of the net
        if(net_startPoint0.a == net_startPoint1.a || net_startPoint0.b == net_startPoint1.b){
            InnerBoundary.p0.a = net_startPoint0.a;
            InnerBoundary.p0.b = net_startPoint0.b;
            InnerBoundary.p1.a = net_startPoint1.a;
            InnerBoundary.p1.b = net_startPoint1.b;
            InnerBoundarySegments.emplace_back(InnerBoundary);
            nets[i].AddInnerRectBoundarySegments(net_startPoint0,net_startPoint1,0,InnerBoundarySegments);
            InnerBoundarySegments.clear();
        }
        //Exception: the inner boundary of the net include one of the corners of the inner bounding box
        else {
            Segment InnerBoundary1(0,0,0,0);
            //Check which corner of the bounding box is closest to the net, and divide the boundary into two segments
            int distance_BetweenLeftBottom = boost::polygon::manhattan_distance(net_startPoint0,InnerRect_LeftBottom);
            int distance_BetweenLeftTop = boost::polygon::manhattan_distance(net_startPoint0,InnerRect_LeftTop);
            int distance_BetweenRightTop = boost::polygon::manhattan_distance(net_startPoint0,InnerRect_RightTop);
            int distance_BetweenRightBottom = boost::polygon::manhattan_distance(net_startPoint0,InnerRect_RightBottom);
            std::vector<int> distance = {distance_BetweenLeftBottom,distance_BetweenLeftTop,distance_BetweenRightTop,distance_BetweenRightBottom};
            int min = *std::min_element(distance.begin(),distance.end());
            if(min == distance_BetweenLeftBottom){
                InnerBoundary.p0.a = net_startPoint0.a;
                InnerBoundary.p0.b = net_startPoint0.b;
                InnerBoundary.p1.a = InnerRect_LeftBottom.a;
                InnerBoundary.p1.b = InnerRect_LeftBottom.b;
                //InnerBoundary1
                InnerBoundary1.p0.a = InnerRect_LeftBottom.a;
                InnerBoundary1.p0.b = InnerRect_LeftBottom.b;
                InnerBoundary1.p1.a = net_startPoint1.a;
                InnerBoundary1.p1.b = net_startPoint1.b;
                temp_point.a = InnerRect_LeftBottom.a;
                temp_point.b = InnerRect_LeftBottom.b;
            }
            else if(min == distance_BetweenLeftTop){
                InnerBoundary.p0.a = net_startPoint0.a;
                InnerBoundary.p0.b = net_startPoint0.b;
                InnerBoundary.p1.a = InnerRect_LeftTop.a;
                InnerBoundary.p1.b = InnerRect_LeftTop.b;
                //InnerBoundary1
                InnerBoundary1.p0.a = InnerRect_LeftTop.a;
                InnerBoundary1.p0.b = InnerRect_LeftTop.b;
                InnerBoundary1.p1.a = net_startPoint1.a;
                InnerBoundary1.p1.b = net_startPoint1.b;
                temp_point.a = InnerRect_LeftTop.a;
                temp_point.b = InnerRect_LeftTop.b;
            }
            else if(min == distance_BetweenRightTop){
                InnerBoundary.p0.a = net_startPoint0.a;
                InnerBoundary.p0.b = net_startPoint0.b;
                InnerBoundary.p1.a = InnerRect_RightTop.a;
                InnerBoundary.p1.b = InnerRect_RightTop.b;
                //InnerBoundary1
                InnerBoundary1.p0.a = InnerRect_RightTop.a;
                InnerBoundary1.p0.b = InnerRect_RightTop.b;
                InnerBoundary1.p1.a = net_startPoint1.a;
                InnerBoundary1.p1.b = net_startPoint1.b;
                temp_point.a = InnerRect_RightTop.a;
                temp_point.b = InnerRect_RightTop.b;
            }
            else if(min == distance_BetweenRightBottom){
                InnerBoundary.p0.a = net_startPoint0.a;
                InnerBoundary.p0.b = net_startPoint0.b;
                InnerBoundary.p1.a = InnerRect_RightBottom.a;
                InnerBoundary.p1.b = InnerRect_RightBottom.b;
                //InnerBoundary1
                InnerBoundary1.p0.a = InnerRect_RightBottom.a;
                InnerBoundary1.p0.b = InnerRect_RightBottom.b;
                InnerBoundary1.p1.a = net_startPoint1.a;
                InnerBoundary1.p1.b = net_startPoint1.b;
                temp_point.a = InnerRect_RightBottom.a;
                temp_point.b = InnerRect_RightBottom.b;
            }
            InnerBoundarySegments.emplace_back(InnerBoundary);
            nets[i].AddInnerRectBoundarySegments(net_startPoint0,temp_point,0,InnerBoundarySegments);
            InnerBoundarySegments.clear();
            InnerBoundarySegments.emplace_back(InnerBoundary1);
            nets[i].AddInnerRectBoundarySegments(temp_point,net_startPoint1,1,InnerBoundarySegments);
            InnerBoundarySegments.clear();
            distance.clear();
        }//end of exception
   }
}//end of FindInnerBoundaryForNet
void FindOutterBoundaryForNet(std::vector<Net> &nets,Rectangle &OutterRect){ 
   Point net_endPoint0(0,0);
   Point net_endPoint1(0,0);
   Point temp_point(0,0);
   Point OutterRect_LeftBottom(OutterRect.rect_x,OutterRect.rect_y);
   Point OutterRect_LeftTop(OutterRect.rect_x,OutterRect.rect_y + OutterRect.rect_h);
   Point OutterRect_RightTop(OutterRect.rect_x + OutterRect.rect_w,OutterRect.rect_y + OutterRect.rect_h);
   Point OutterRect_RightBottom(OutterRect.rect_x + OutterRect.rect_w,OutterRect.rect_y);
   Segment OutterBoundary(0,0,0,0);
   std::vector<Segment> OutterBoundarySegments;
   for(int i = 0; i < nets.size();i++){
       net_endPoint0.a = nets[i].Boundaries[0].endPoint.a;
       net_endPoint0.b = nets[i].Boundaries[0].endPoint.b;
       net_endPoint1.a = nets[i].Boundaries[1].endPoint.a;
       net_endPoint1.b = nets[i].Boundaries[1].endPoint.b;
       // if x coordinate/ y coordinate is the same, the inner boundary is composed of the end points of the two boundaries of the net
        if(net_endPoint0.a == net_endPoint1.a || net_endPoint0.b == net_endPoint1.b){
            OutterBoundary.p0.a = net_endPoint0.a;
            OutterBoundary.p0.b = net_endPoint0.b;
            OutterBoundary.p1.a = net_endPoint1.a;
            OutterBoundary.p1.b = net_endPoint1.b;
            OutterBoundarySegments.emplace_back(OutterBoundary);
            nets[i].AddOutterRectBoundarySegments(net_endPoint0,net_endPoint1,0,OutterBoundarySegments);
            OutterBoundarySegments.clear();
        }
        //Exception: the inner boundary of the net include one of the corners of the outter bounding box
        else {
            Segment OutterBoundary1(0,0,0,0);
            //Check which corner of the bounding box is closest to the net, and divide the boundary into two segments
            int distance_BetweenLeftBottom = boost::polygon::manhattan_distance(net_endPoint0,OutterRect_LeftBottom);
            int distance_BetweenLeftTop = boost::polygon::manhattan_distance(net_endPoint0,OutterRect_LeftTop);
            int distance_BetweenRightTop = boost::polygon::manhattan_distance(net_endPoint0,OutterRect_RightTop);
            int distance_BetweenRightBottom = boost::polygon::manhattan_distance(net_endPoint0,OutterRect_RightBottom);
            std::vector<int> distance = {distance_BetweenLeftBottom,distance_BetweenLeftTop,distance_BetweenRightTop,distance_BetweenRightBottom};
            int min = *std::min_element(distance.begin(),distance.end());
            if(min == distance_BetweenLeftBottom){
                OutterBoundary.p0.a = net_endPoint0.a;
                OutterBoundary.p0.b = net_endPoint0.b;
                OutterBoundary.p1.a = OutterRect_LeftBottom.a;
                OutterBoundary.p1.b = OutterRect_LeftBottom.b;
                //OutterBoundary1
                OutterBoundary1.p0.a = OutterRect_LeftBottom.a;
                OutterBoundary1.p0.b = OutterRect_LeftBottom.b;
                OutterBoundary1.p1.a = net_endPoint1.a;
                OutterBoundary1.p1.b = net_endPoint1.b;
                temp_point.a = OutterRect_LeftBottom.a;
                temp_point.b = OutterRect_LeftBottom.b;
            }
            else if(min == distance_BetweenLeftTop){
                OutterBoundary.p0.a = net_endPoint0.a;
                OutterBoundary.p0.b = net_endPoint0.b;
                OutterBoundary.p1.a = OutterRect_LeftTop.a;
                OutterBoundary.p1.b = OutterRect_LeftTop.b;
                //OutterBoundary1
                OutterBoundary1.p0.a = OutterRect_LeftTop.a;
                OutterBoundary1.p0.b = OutterRect_LeftTop.b;
                OutterBoundary1.p1.a = net_endPoint1.a;
                OutterBoundary1.p1.b = net_endPoint1.b;
                temp_point.a = OutterRect_LeftTop.a;
                temp_point.b = OutterRect_LeftTop.b;
            }
            else if(min == distance_BetweenRightTop){
                OutterBoundary.p0.a = net_endPoint0.a;
                OutterBoundary.p0.b = net_endPoint0.b;
                OutterBoundary.p1.a = OutterRect_RightTop.a;
                OutterBoundary.p1.b = OutterRect_RightTop.b;
                //OutterBoundary1
                OutterBoundary1.p0.a = OutterRect_RightTop.a;
                OutterBoundary1.p0.b = OutterRect_RightTop.b;
                OutterBoundary1.p1.a = net_endPoint1.a;
                OutterBoundary1.p1.b = net_endPoint1.b;
                temp_point.a = OutterRect_RightTop.a;
                temp_point.b = OutterRect_RightTop.b;
            }
            else if(min == distance_BetweenRightBottom){
                OutterBoundary.p0.a = net_endPoint0.a;
                OutterBoundary.p0.b = net_endPoint0.b;
                OutterBoundary.p1.a = OutterRect_RightBottom.a;
                OutterBoundary.p1.b = OutterRect_RightBottom.b;
                //OutterBoundary1
                OutterBoundary1.p0.a = OutterRect_RightBottom.a;
                OutterBoundary1.p0.b = OutterRect_RightBottom.b;
                OutterBoundary1.p1.a = net_endPoint1.a;
                OutterBoundary1.p1.b = net_endPoint1.b;
                temp_point.a = OutterRect_RightBottom.a;
                temp_point.b = OutterRect_RightBottom.b;
            }
            OutterBoundarySegments.emplace_back(OutterBoundary);
            nets[i].AddOutterRectBoundarySegments(net_endPoint0,temp_point,0,OutterBoundarySegments);
            OutterBoundarySegments.clear();
            OutterBoundarySegments.emplace_back(OutterBoundary1);
            nets[i].AddOutterRectBoundarySegments(temp_point,net_endPoint1,1,OutterBoundarySegments);
            OutterBoundarySegments.clear();
            distance.clear();
        }//end of exception
   }
}//end of FindOutterBoundaryForNet

//Check if the innerboundary segment is contained in the boundary
bool isSegmentContained(Segment_g &contained,Segment_g &container){
    Point_g p3 = contained.first;
    Point_g p4 = contained.second;
    //check if start point and end point of the inner boundary are contained in the net's boundary
    bool start_on_container = boost::geometry::covered_by(p3,container);
    bool end_on_container = boost::geometry::covered_by(p4,container);
    return start_on_container && end_on_container;
}
//Eliminate overlapping between inner boundary and net's boundary
void EliminateOverlappingBetweenInnerBoundary(std::vector<Net> &nets){
    Segment_g boundary0_startsegment(Point_g(0,0),Point_g(0,0));
    Segment_g boundary1_startsegment(Point_g(0,0),Point_g(0,0));
    Segment_g InnerBoundary0(Point_g(0,0),Point_g(0,0));
    Segment_g InnerBoundary1(Point_g(0,0),Point_g(0,0));
    Boundary temp_boundary = nets[0].Boundaries[0];
    int needUpdateboundaryID = 0;
    bool needUpdate = false;
    for(int i = 0;i < nets.size();i++){
        needUpdate = false;
        if(nets[i].InnerRectBoundarySegments.size() >1 ){
            //InnerBoundary info
            InnerBoundary0.first.x(nets[i].InnerRectBoundarySegments[0].BoundarySegments[0].p0.a);
            InnerBoundary0.first.y(nets[i].InnerRectBoundarySegments[0].BoundarySegments[0].p0.b);
            InnerBoundary0.second.x(nets[i].InnerRectBoundarySegments[0].BoundarySegments[0].p1.a);
            InnerBoundary0.second.y(nets[i].InnerRectBoundarySegments[0].BoundarySegments[0].p1.b);
            InnerBoundary1.first.x(nets[i].InnerRectBoundarySegments[1].BoundarySegments[0].p0.a);
            InnerBoundary1.first.y(nets[i].InnerRectBoundarySegments[1].BoundarySegments[0].p0.b);
            InnerBoundary1.second.x(nets[i].InnerRectBoundarySegments[1].BoundarySegments[0].p1.a);
            InnerBoundary1.second.y(nets[i].InnerRectBoundarySegments[1].BoundarySegments[0].p1.b);
            //net's boundary info
            boundary0_startsegment.first.x(nets[i].Boundaries[0].BoundarySegments[0].p0.a);
            boundary0_startsegment.first.y(nets[i].Boundaries[0].BoundarySegments[0].p0.b);
            boundary0_startsegment.second.x(nets[i].Boundaries[0].BoundarySegments[0].p1.a);
            boundary0_startsegment.second.y(nets[i].Boundaries[0].BoundarySegments[0].p1.b);
            boundary1_startsegment.first.x(nets[i].Boundaries[1].BoundarySegments[0].p0.a);
            boundary1_startsegment.first.y(nets[i].Boundaries[1].BoundarySegments[0].p0.b);
            boundary1_startsegment.second.x(nets[i].Boundaries[1].BoundarySegments[0].p1.a);
            boundary1_startsegment.second.y(nets[i].Boundaries[1].BoundarySegments[0].p1.b);
            if(isSegmentContained(InnerBoundary0,boundary0_startsegment)){
                std::cout<<"InnerBoundary0 is contained in boundary0_startsegment,net id"<<i<<std::endl;
                needUpdate = true;
                needUpdateboundaryID = nets[i].Boundaries[0].BoundaryID; 
                if(boundary0_startsegment.first.x() == InnerBoundary0.first.x() && boundary0_startsegment.first.y() == InnerBoundary0.first.y()){
                    nets[i].Boundaries[0].BoundarySegments[0].p0.a = InnerBoundary0.second.x();
                    nets[i].Boundaries[0].BoundarySegments[0].p0.b = InnerBoundary0.second.y();
                    nets[i].Boundaries[0].startPoint.a = InnerBoundary0.second.x();
                    nets[i].Boundaries[0].startPoint.b = InnerBoundary0.second.y();
                }
                else if(boundary0_startsegment.first.x() == InnerBoundary0.second.x() && boundary0_startsegment.first.y() == InnerBoundary0.second.y()){
                    nets[i].Boundaries[0].BoundarySegments[0].p0.a = InnerBoundary0.first.x();
                    nets[i].Boundaries[0].BoundarySegments[0].p0.b = InnerBoundary0.first.y();
                    nets[i].Boundaries[0].startPoint.a = InnerBoundary0.first.x();
                    nets[i].Boundaries[0].startPoint.b = InnerBoundary0.first.y();
                }
                temp_boundary = nets[i].Boundaries[0];
            }
            else if(isSegmentContained(InnerBoundary0,boundary1_startsegment)){
                std::cout<<"InnerBoundary0 is contained in boundary1_startsegment,net id"<<i<<std::endl;
                needUpdate = true;
                needUpdateboundaryID = nets[i].Boundaries[1].BoundaryID;
                if(boundary1_startsegment.first.x() == InnerBoundary0.first.x() && boundary1_startsegment.first.y() == InnerBoundary0.first.y()){
                    nets[i].Boundaries[1].BoundarySegments[0].p0.a = InnerBoundary0.second.x();
                    nets[i].Boundaries[1].BoundarySegments[0].p0.b = InnerBoundary0.second.y();
                    nets[i].Boundaries[1].startPoint.a = InnerBoundary0.second.x();
                    nets[i].Boundaries[1].startPoint.b = InnerBoundary0.second.y();
                }
                else if(boundary1_startsegment.first.x() == InnerBoundary0.second.x() && boundary1_startsegment.first.y() == InnerBoundary0.second.y()){
                    nets[i].Boundaries[1].BoundarySegments[0].p0.a = InnerBoundary0.first.x();
                    nets[i].Boundaries[1].BoundarySegments[0].p0.b = InnerBoundary0.first.y();
                    nets[i].Boundaries[1].startPoint.a = InnerBoundary0.first.x();
                    nets[i].Boundaries[1].startPoint.b = InnerBoundary0.first.y();
                }
                temp_boundary = nets[i].Boundaries[1];
            }
            else if(isSegmentContained(InnerBoundary1,boundary0_startsegment)){
                std::cout<<"InnerBoundary1 is contained in boundary0_startsegment,net id"<<i<<std::endl;
                needUpdate = true;
                needUpdateboundaryID = nets[i].Boundaries[0].BoundaryID;
                if(boundary0_startsegment.first.x() == InnerBoundary1.first.x() && boundary0_startsegment.first.y() == InnerBoundary1.first.y()){
                    nets[i].Boundaries[0].BoundarySegments[0].p0.a = InnerBoundary1.second.x();
                    nets[i].Boundaries[0].BoundarySegments[0].p0.b = InnerBoundary1.second.y();
                    nets[i].Boundaries[0].startPoint.a = InnerBoundary1.second.x();
                    nets[i].Boundaries[0].startPoint.b = InnerBoundary1.second.y();
                }
                else if(boundary0_startsegment.first.x() == InnerBoundary1.second.x() && boundary0_startsegment.first.y() == InnerBoundary1.second.y()){
                    nets[i].Boundaries[0].BoundarySegments[0].p0.a = InnerBoundary1.first.x();
                    nets[i].Boundaries[0].BoundarySegments[0].p0.b = InnerBoundary1.first.y();
                    nets[i].Boundaries[0].startPoint.a = InnerBoundary1.first.x();
                    nets[i].Boundaries[0].startPoint.b = InnerBoundary1.first.y();
                }
                temp_boundary = nets[i].Boundaries[0];
            }
            else if (isSegmentContained(InnerBoundary1,boundary1_startsegment)){
                needUpdate = true;
                needUpdateboundaryID = nets[i].Boundaries[1].BoundaryID;
                    if(boundary1_startsegment.first.x() == InnerBoundary1.first.x() && boundary1_startsegment.first.y() == InnerBoundary1.first.y()){
                    nets[i].Boundaries[1].BoundarySegments[0].p0.a = InnerBoundary1.second.x();
                    nets[i].Boundaries[1].BoundarySegments[0].p0.b = InnerBoundary1.second.y();
                    nets[i].Boundaries[1].startPoint.a = InnerBoundary1.second.x();
                    nets[i].Boundaries[1].startPoint.b = InnerBoundary1.second.y();
                }
                else if(boundary1_startsegment.first.x() == InnerBoundary1.second.x() && boundary1_startsegment.first.y() == InnerBoundary1.second.y()){
                    nets[i].Boundaries[1].BoundarySegments[0].p0.a = InnerBoundary1.first.x();
                    nets[i].Boundaries[1].BoundarySegments[0].p0.b = InnerBoundary1.first.y();
                    nets[i].Boundaries[1].startPoint.a = InnerBoundary1.first.x();
                    nets[i].Boundaries[1].startPoint.b = InnerBoundary1.first.y();
                }
                temp_boundary = nets[i].Boundaries[1];
            }

            //Update the boundary/Innerboundary info
            if(needUpdate){
                nets[i].InnerRectBoundarySegments.pop_back();
                nets[i].InnerRectBoundarySegments[0].startPoint.a = nets[i].Boundaries[0].startPoint.a;
                nets[i].InnerRectBoundarySegments[0].startPoint.b = nets[i].Boundaries[0].startPoint.b;
                nets[i].InnerRectBoundarySegments[0].endPoint.a = nets[i].Boundaries[1].startPoint.a;
                nets[i].InnerRectBoundarySegments[0].endPoint.b = nets[i].Boundaries[1].startPoint.b;
                nets[i].InnerRectBoundarySegments[0].BoundarySegments[0].p0.a = nets[i].Boundaries[0].startPoint.a;
                nets[i].InnerRectBoundarySegments[0].BoundarySegments[0].p0.b = nets[i].Boundaries[0].startPoint.b;
                nets[i].InnerRectBoundarySegments[0].BoundarySegments[0].p1.a = nets[i].Boundaries[1].startPoint.a;
                nets[i].InnerRectBoundarySegments[0].BoundarySegments[0].p1.b = nets[i].Boundaries[1].startPoint.b;
                
                for(int j = 0;j < nets.size();j++){
                    int temp_ID0 = nets[j].Boundaries[0].BoundaryID;
                    int temp_ID1 = nets[j].Boundaries[1].BoundaryID;
                    if(temp_ID0 == needUpdateboundaryID && j!=i ){
                        nets[j].Boundaries[0] = temp_boundary;
                    //nets[j]'s InnerRectBoundarySegments info
                        nets[j].InnerRectBoundarySegments[0].startPoint.a = nets[j].Boundaries[0].startPoint.a;
                        nets[j].InnerRectBoundarySegments[0].startPoint.b = nets[j].Boundaries[0].startPoint.b;
                        nets[j].InnerRectBoundarySegments[0].endPoint.a = nets[j].Boundaries[1].startPoint.a;
                        nets[j].InnerRectBoundarySegments[0].endPoint.b = nets[j].Boundaries[1].startPoint.b;
                        nets[j].InnerRectBoundarySegments[0].BoundarySegments[0].p0.a = nets[j].Boundaries[0].startPoint.a;
                        nets[j].InnerRectBoundarySegments[0].BoundarySegments[0].p0.b = nets[j].Boundaries[0].startPoint.b;
                        nets[j].InnerRectBoundarySegments[0].BoundarySegments[0].p1.a = nets[j].Boundaries[1].startPoint.a;
                        nets[j].InnerRectBoundarySegments[0].BoundarySegments[0].p1.b = nets[j].Boundaries[1].startPoint.b;
                    }
                    else if(temp_ID1 == needUpdateboundaryID && j!=i){
                        nets[j].Boundaries[1] = temp_boundary;
                        //nets[j]'s InnerRectBoundarySegments info
                        nets[j].InnerRectBoundarySegments[0].startPoint.a = nets[j].Boundaries[0].startPoint.a;
                        nets[j].InnerRectBoundarySegments[0].startPoint.b = nets[j].Boundaries[0].startPoint.b;
                        nets[j].InnerRectBoundarySegments[0].endPoint.a = nets[j].Boundaries[1].startPoint.a;
                        nets[j].InnerRectBoundarySegments[0].endPoint.b = nets[j].Boundaries[1].startPoint.b;
                        nets[j].InnerRectBoundarySegments[0].BoundarySegments[0].p0.a = nets[j].Boundaries[0].startPoint.a;
                        nets[j].InnerRectBoundarySegments[0].BoundarySegments[0].p0.b = nets[j].Boundaries[0].startPoint.b;
                        nets[j].InnerRectBoundarySegments[0].BoundarySegments[0].p1.a = nets[j].Boundaries[1].startPoint.a;
                        nets[j].InnerRectBoundarySegments[0].BoundarySegments[0].p1.b = nets[j].Boundaries[1].startPoint.b;
                    }
                }
            } //end of if(needupdate)
        }
    }
}//end of EliminateOverlappingBetweenInnerBoundary
int main(int argc,char* argv[]){
    std::ifstream file(argv[1]);
    std::vector<Segment> AllBoundarySegments;
    std::vector<Segment> AllnetsSegments;
    std::vector<Net> nets;
    std::vector<Boundary> boundaries;
    std::vector<Polygon_g> Polygons;
    std::vector<double> area_result;
    Rectangle InnerRect = {-5e+07,-5e+07,1e+08,1e+08}; 
    Rectangle OutterRect = {-1.4e+08,-1.4e+08,2.8e+08,2.8e+08};
    InputFile_ExtendBoundaryToOutterRect_result(file,AllBoundarySegments,AllnetsSegments,nets,boundaries,InnerRect);
    FindBoundariesForNet(nets,boundaries);
    OctilinearizeBoundary(nets,boundaries,OutterRect); 
    //After boundaries octilinearize, update every net of elements in net.Boundaries first,note the bounday.ID and update the new boundary info
    UpdateNetBoundaries(nets,boundaries);
    FindInnerBoundaryForNet(nets,InnerRect);
    FindOutterBoundaryForNet(nets,OutterRect);
    EliminateOverlappingBetweenInnerBoundary(nets);
    file.close();
    //OutputFile_OctilinearizeBoundary(nets);
    
    //Output to check the boundary info
    /*for (int i = 0;i < nets.size();i++){ 
        //std::cout<<"Net"<<i<<":\n"<<"Net"<<i<<"_startPoint: "<<nets[i].startPoint.a<<" "<<nets[i].startPoint.b<<" Net"<<i<<"_endPoint: "<<nets[i].endPoint.a<<" "<<nets[i].endPoint.b<<"\n";
        //std::cout<<"Net"<<i<<"_segments_number: "<<nets[i].NetSegments.size()<<"\n";
        //nets[i].PrintNetSegments_drawing(); 
        //int nets_BoundaryNumer = nets[i].Boundaries.size() + nets[i].OutterRectBoundarySegments.size() + nets[i].InnerRectBoundarySegments.size();
        //std::cout<<"Net"<<i<<"_Boundaries_number: "<<nets_BoundaryNumer<<"\n";
        nets[i].PrintBoundarySegments_drawing();
    }*/

    //output to  calculate area of net
    
    /*
    for(int i = 0;i<nets.size();i++){
        std::cout<<"Net's tuple begin:\n";
        nets[i].PrintBoundaryTuple(); 
        std::cout<<"Net's tuple end\n";
    }*/

    //output to calculate area of net
    file.open("InitialOctBoundary_tuple_result.txt");
    Inputfile_CalculateAreaofNet(file,Polygons);
    CalculateAreaofNet(Polygons,area_result);
    //output area & length of each net
    for(int i = 0;i < nets.size();i++){
        int NetLength = 0;
        std::cout<<"Net"<<i<<"_area: ";
        std::cout<<area_result[i]<<"\n";
        std::cout<<"Net"<<i<<"_length: ";
        NetLength = nets[i].CalculateNetLength();
        std::cout<<NetLength<<"\n";
    }
    file.close();
    return 0;
}//end of  main
