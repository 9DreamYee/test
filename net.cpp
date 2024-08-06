#include "net.h"
#include <iostream>
Net::Net(Point start,Point end)
    : startPoint(start),endPoint(end){}
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
        //std::cout<<"Net_segment: "<<NetSegments[i].p0.a<<" "<<NetSegments[i].p0.b<<" "<<NetSegments[i].p1.a<<" "<<NetSegments[i].p1.b<<"\n";
    }
}
void Net::PrintBoundarySegments_drawing(){
    for(int i = 0;i < Boundaries.size();i++){
        for(auto seg:Boundaries[i].BoundarySegments){
            std::cout<<"_view.drawLine("<<seg.p0.a<<","<<seg.p0.b<<","<<seg.p1.a<<","<<seg.p1.b<<");\n";
        }
        /*
        std::cout<<"Boundary_segment_info_start:\n";
        std::cout<<"Boundary_"<<i<<"_startPoint: "<<Boundaries[i].startPoint.a<<" "<<Boundaries[i].startPoint.b;
        std::cout<<" Boundary_"<<i<<"_endPoint: "<<Boundaries[i].endPoint.a<<" "<<Boundaries[i].endPoint.b<<"\n";
        for(auto seg: Boundaries[i].BoundarySegments){
            std::cout<<"Boundary_segment: "<<seg.p0.a<<" "<<seg.p0.b<<" "<<seg.p1.a<<" "<<seg.p1.b<<"\n";
        }*/
    }
    //std::cout<<"Boundary_segment_info_end\n"; 
    for(int i = 0;i<InnerRectBoundarySegments.size();i++){
        for(auto seg:InnerRectBoundarySegments[i].BoundarySegments){
            std::cout<<"_view.drawLine("<<seg.p0.a<<","<<seg.p0.b<<","<<seg.p1.a<<","<<seg.p1.b<<");\n";
        }
    /*
    std::cout<<"InnerBoundary_segment_info_start:\n";
    std::cout<<"InnerBoundary_"<<i<<"_startPoint: "<<InnerRectBoundarySegments[i].startPoint.a<<" "<<InnerRectBoundarySegments[i].startPoint.b;
    std::cout<<" InnerBoundary_"<<i<<"_endPoint: "<<InnerRectBoundarySegments[i].endPoint.a<<" "<<InnerRectBoundarySegments[i].endPoint.b<<"\n";
    for(auto seg:InnerRectBoundarySegments[i].BoundarySegments){
        std::cout<<"InnerBoundary_segment: "<<seg.p0.a<<" "<<seg.p0.b<<" "<<seg.p1.a<<" "<<seg.p1.b<<"\n";
        }*/
    }
    //std::cout<<"InnerBoundary_segment_info_end\n";
    for(int i = 0 ;i < OutterRectBoundarySegments.size();i++){
        for(auto seg:OutterRectBoundarySegments[i].BoundarySegments){
            std::cout<<"_view.drawLine("<<seg.p0.a<<","<<seg.p0.b<<","<<seg.p1.a<<","<<seg.p1.b<<");\n";
        }
        /*
        std::cout<<"OutterBoundary_segment_info_start:\n";
        std::cout<<"OutterBoundary_"<<i<<"_startPoint: "<<OutterRectBoundarySegments[i].startPoint.a<<" "<<OutterRectBoundarySegments[i].startPoint.b;
        std::cout<<" OutterBoundary_"<<i<<"_endPoint: "<<OutterRectBoundarySegments[i].endPoint.a<<" "<<OutterRectBoundarySegments[i].endPoint.b<<"\n";
        for(auto seg:OutterRectBoundarySegments[i].BoundarySegments){
            std::cout<<"OutterBoundary_segment: "<<seg.p0.a<<" "<<seg.p0.b<<" "<<seg.p1.a<<" "<<seg.p1.b<<"\n";
        }*/
    }
    //std::cout<<"OutterBoundary_segment_info_end\n";
}
void Net::PrintBoundaryTuple(){
    for(auto seg:Boundaries[0].BoundarySegments){
        std::cout<<"("<<seg.p0.a<<","<<seg.p0.b<<")\n"<<"("<<seg.p1.a<<","<<seg.p1.b<<")"<<"\n";
    }
    for(int j = Boundaries[1].BoundarySegments.size()-1;j >= 0;j--){
        std::cout<<"("<<Boundaries[1].BoundarySegments[j].p1.a<<","<<Boundaries[1].BoundarySegments[j].p1.b<<")\n"<<"("<<Boundaries[1].BoundarySegments[j].p0.a<<","<<Boundaries[1].BoundarySegments[j].p0.b<<")"<<"\n";
    }
}
