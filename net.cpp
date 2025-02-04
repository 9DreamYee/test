#include "net.h"
#include <iostream>

Net::Net(Point start,Point end)
    : startPoint(start),endPoint(end){}

void Net::SetNetArea(const long long &area){
    NetArea = area;
}
double Net::GetNetArea(){
    return NetArea;
}
void Net::AddNetSegment(const Segment &segment){
    NetSegments.emplace_back(segment);
}
void Net::AddBoundary(Point &start,Point &end,const int &ID, const std::vector<Segment> &BoundarySegments){
    Boundaries.emplace_back(Boundary(start,end,ID,BoundarySegments));
}
void Net::AddInnerRectBoundarySegments(Point &start,Point &end,const int &ID,const std::vector<Segment> &BoundarySegments){
    InnerRectBoundarySegments.emplace_back(Boundary(start,end,ID,BoundarySegments));
}
void Net::AddOutterRectBoundarySegments(Point &start,Point &end,const int &ID,const std::vector<Segment> &BoundarySegments){
    OutterRectBoundarySegments.emplace_back(Boundary(start,end,ID,BoundarySegments));
}
int Net::CalculateNetLength(){
    int TotalLength = 0;
    for (auto seg:NetSegments){
        TotalLength += boost::polygon::length(seg);
    }
    return TotalLength;
}
void Net::PrintNetSegments_drawing(){
    for(int i = 0;i < NetSegments.size();i++){
        std::cout<<"_view.drawLine("<<NetSegments[i].p0.a<<","<<NetSegments[i].p0.b<<","<<NetSegments[i].p1.a<<","<<NetSegments[i].p1.b<<");\n";
    }
}
void Net::PrintNetSegments_info(){
    std::cout<<"Initial_route_segment: ";
    for(int i = 0;i < NetSegments.size();i++){
        std::cout<<NetSegments[i].p0.a<<" "<<NetSegments[i].p0.b<<" "<<NetSegments[i].p1.a<<" "<<NetSegments[i].p1.b<<" ";
    }
    std::cout<<"\n";
}
void Net::PrintBoundarySegments_drawing(){
    for(int i = 0;i < Boundaries.size();i++){
        for(auto seg:Boundaries[i].BoundarySegments){
            std::cout<<"_view.drawLine("<<seg.p0.a<<","<<seg.p0.b<<","<<seg.p1.a<<","<<seg.p1.b<<");\n";
        }
    }
    for(int i = 0;i<InnerRectBoundarySegments.size();i++){
        for(auto seg:InnerRectBoundarySegments[i].BoundarySegments){
            std::cout<<"_view.drawLine("<<seg.p0.a<<","<<seg.p0.b<<","<<seg.p1.a<<","<<seg.p1.b<<");\n";
        }
    }
    for(int i = 0 ;i < OutterRectBoundarySegments.size();i++){
        for(auto seg:OutterRectBoundarySegments[i].BoundarySegments){
            std::cout<<"_view.drawLine("<<seg.p0.a<<","<<seg.p0.b<<","<<seg.p1.a<<","<<seg.p1.b<<");\n";
        }
    }
}
void Net::PrintBoundarySegments_info(){
    for(int i = 0;i < Boundaries.size();i++){
        std::cout<<"Boundary_segment_"<<i<<"_info_start:\n";
        std::cout<<"Boundary_"<<i<<"_startPoint: "<<Boundaries[i].startPoint.a<<" "<<Boundaries[i].startPoint.b;
        std::cout<<" Boundary_"<<i<<"_endPoint: "<<Boundaries[i].endPoint.a<<" "<<Boundaries[i].endPoint.b<<"\n";
        std::cout<<"Boundary_"<<i<<"_segment: ";
        for(auto seg: Boundaries[i].BoundarySegments){
            std::cout<<seg.p0.a<<" "<<seg.p0.b<<" "<<seg.p1.a<<" "<<seg.p1.b<<" ";
        }
        std::cout<<"\n";
    }
    
    for(int i = 0;i<InnerRectBoundarySegments.size();i++){
    std::cout<<"InnerBoundary_segment_"<<i<<"_info_start:\n";
    std::cout<<"InnerBoundary_"<<i<<"_startPoint: "<<InnerRectBoundarySegments[i].startPoint.a<<" "<<InnerRectBoundarySegments[i].startPoint.b;
    std::cout<<" InnerBoundary_"<<i<<"_endPoint: "<<InnerRectBoundarySegments[i].endPoint.a<<" "<<InnerRectBoundarySegments[i].endPoint.b<<"\n";
    std::cout<<"InnerBoundary_"<<i<<"_segment: ";
    for(auto seg:InnerRectBoundarySegments[i].BoundarySegments){
        std::cout<<seg.p0.a<<" "<<seg.p0.b<<" "<<seg.p1.a<<" "<<seg.p1.b<<" ";
        }
        std::cout<<"\n";
    }
    for(int i = 0 ;i < OutterRectBoundarySegments.size();i++){
        std::cout<<"OutterBoundary_segment_info_start:\n";
        std::cout<<"OutterBoundary_"<<i<<"_startPoint: "<<OutterRectBoundarySegments[i].startPoint.a<<" "<<OutterRectBoundarySegments[i].startPoint.b;
        std::cout<<" OutterBoundary_"<<i<<"_endPoint: "<<OutterRectBoundarySegments[i].endPoint.a<<" "<<OutterRectBoundarySegments[i].endPoint.b<<"\n";
        std::cout<<"OutterBoundary_"<<i<<"_segment: ";
        for(auto seg:OutterRectBoundarySegments[i].BoundarySegments){
            std::cout<<seg.p0.a<<" "<<seg.p0.b<<" "<<seg.p1.a<<" "<<seg.p1.b<<" ";
        }
        std::cout<<"\n";
    }
}
void Net::PrintBoundaryTuple(){
    //size of InnerRect or OutterRect more than 1 (may cause calculating area error)
    std::ofstream NetFile("Octilinear_tuple.txt",std::ios::app);
    //NetFile.open("Octilinear_tuple.txt");
    //Print Boundary[0] info
    for(auto seg:Boundaries[0].BoundarySegments){
        //std::cout<<"("<<seg.p0.a<<","<<seg.p0.b<<")\n"<<"("<<seg.p1.a<<","<<seg.p1.b<<")"<<"\n";
        NetFile<<"("<<seg.p0.a<<","<<seg.p0.b<<")\n"<<"("<<seg.p1.a<<","<<seg.p1.b<<")"<<"\n";
    }
    //Print extra info for net that OutterRectBoundary > 1
    if(OutterRectBoundarySegments.size() > 1){
        /*
        std::cout<<"("<<OutterRectBoundarySegments[0].startPoint.a<<","<<OutterRectBoundarySegments[0].startPoint.b<<")\n"<<"("<<
            OutterRectBoundarySegments[0].endPoint.a<<","<<OutterRectBoundarySegments[0].endPoint.b<<")"<<"\n";
        std::cout<<"("<<OutterRectBoundarySegments[1].startPoint.a<<","<<OutterRectBoundarySegments[1].startPoint.b<<")\n"<<"("<<
            OutterRectBoundarySegments[1].endPoint.a<<","<<OutterRectBoundarySegments[1].endPoint.b<<")"<<"\n";
        */
        NetFile<<"("<<OutterRectBoundarySegments[0].startPoint.a<<","<<OutterRectBoundarySegments[0].startPoint.b<<")\n"<<"("<<
            OutterRectBoundarySegments[0].endPoint.a<<","<<OutterRectBoundarySegments[0].endPoint.b<<")"<<"\n";
        NetFile<<"("<<OutterRectBoundarySegments[1].startPoint.a<<","<<OutterRectBoundarySegments[1].startPoint.b<<")\n"<<"("<<
            OutterRectBoundarySegments[1].endPoint.a<<","<<OutterRectBoundarySegments[1].endPoint.b<<")"<<"\n";
    }
    //Print Boundary[1] info
    for(int j = Boundaries[1].BoundarySegments.size()-1;j >= 0;j--){
        //std::cout<<"("<<Boundaries[1].BoundarySegments[j].p1.a<<","<<Boundaries[1].BoundarySegments[j].p1.b<<")\n"<<"("<<Boundaries[1].BoundarySegments[j].p0.a<<","<<Boundaries[1].BoundarySegments[j].p0.b<<")"<<"\n";
        NetFile<<"("<<Boundaries[1].BoundarySegments[j].p1.a<<","<<Boundaries[1].BoundarySegments[j].p1.b<<")\n"<<"("<<Boundaries[1].BoundarySegments[j].p0.a<<","<<Boundaries[1].BoundarySegments[j].p0.b<<")"<<"\n";
    }
    if(InnerRectBoundarySegments.size() > 1){
        /*
        std::cout<<"("<<InnerRectBoundarySegments[1].endPoint.a<<","<<InnerRectBoundarySegments[1].endPoint.b<<")\n"<<"("<<
            InnerRectBoundarySegments[1].startPoint.a<<","<<InnerRectBoundarySegments[1].startPoint.b<<")"<<"\n";
        std::cout<<"("<<InnerRectBoundarySegments[0].endPoint.a<<","<<InnerRectBoundarySegments[0].endPoint.b<<")\n"<<"("<<
            InnerRectBoundarySegments[0].startPoint.a<<","<<InnerRectBoundarySegments[0].startPoint.b<<")"<<"\n";
        */
        NetFile<<"("<<InnerRectBoundarySegments[1].endPoint.a<<","<<InnerRectBoundarySegments[1].endPoint.b<<")\n"<<"("<<
            InnerRectBoundarySegments[1].startPoint.a<<","<<InnerRectBoundarySegments[1].startPoint.b<<")"<<"\n";
        NetFile<<"("<<InnerRectBoundarySegments[0].endPoint.a<<","<<InnerRectBoundarySegments[0].endPoint.b<<")\n"<<"("<<
            InnerRectBoundarySegments[0].startPoint.a<<","<<InnerRectBoundarySegments[0].startPoint.b<<")"<<"\n";
    }
    NetFile.close();
}

