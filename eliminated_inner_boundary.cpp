#include<iostream>
#include <string>
#include<iomanip>
#include<vector>
#include<fstream>
#include<sstream>


using namespace std;
struct Point{
    double x;
    double y;
    Point(double _x = 0.0 , double _y = 0.0) : x(_x), y(_y) {}
};
struct Segment{
    Point start;
    Point end;
    Segment(double x1 = 0.0 , double y1 = 0.0,double x2 = 0.0,double y2 = 0.0) : start(x1,y1), end(x2,y2)  {}
};
struct Rectangle{
    double rect_x;
    double rect_y;
    double rect_w;
    double rect_h;
};
//checking
bool isIntersecting(Segment line,Rectangle rect) {
    // 檢查線段是否在矩形的上方
    if (line.start.y > rect.rect_y + rect.rect_h && line.end.y > rect.rect_y + rect.rect_h) {
        return false;
    }
    // 檢查線段是否在矩形的下方
    if (line.start.y < rect.rect_y && line.start.y < rect.rect_y) {
        return false;
    }
    // 檢查線段是否在矩形的右方
    if (line.start.x > rect.rect_x + rect.rect_w && line.end.x > rect.rect_x + rect.rect_w) {
        return false;
    }
    // 檢查線段是否在矩形的左方
    if (line.start.x < rect.rect_x && line.end.x < rect.rect_x) {
        return false;
    }
    return true;
}
bool doInsideRectangle(Segment line,Point topLeft,Point topRight,Point bottomLeft,Point bottomRight){
    if ((line.start.x >= topLeft.x && line.start.x <= topRight.x && line.start.y <= topLeft.y && line.start.y >= bottomLeft.y) &&
            (line.end.x >= topLeft.x && line.end.x <= topRight.x && line.end.y <= topLeft.y && line.end.y >= bottomLeft.y)){
        return true;
    }
    return false;
}

//Check if the line segment intersect with the rectangle side.
bool doIntersect(Segment line,Point p2,Point q2){
    auto orientation = [](Point p, Point q, Point r) -> int{
        double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        if (val == 0) return 0;  // collinear
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    };
    int o1 = orientation (line.start,line.end,p2);
    int o2 = orientation (line.start,line.end,q2);
    int o3 = orientation (p2,q2,line.start);
    int o4 = orientation (p2,q2,line.end);
    //cout<<"o1: "<< o1 << " o2: "<< o2 <<" o3: "<< o3<<" o4: "<< o4 <<endl;
    // 一般情況
    return (o1 != o2) && (o3 != o4);
}
//rect_1 is bottomLeft, rect_2 is topRight
bool isLineIntersectRectangle(Point rect_1,Point rect_2,Segment line,int &IntersectWithWhcihSide){
    Point topLeft = {rect_1.x,rect_2.y};
    Point topRight = {rect_2.x,rect_2.y};
    Point bottomLeft = {rect_1.x,rect_1.y};
    Point bottomRight = {rect_2.x,rect_1.y};
    // IntersectwithWhcihside -> 1 for Top, 2 for Bottom , 3 for Left , 4 for Right 5 for inside Rectangle
    if(doInsideRectangle(line,topLeft,topRight,bottomLeft,bottomRight)){
        IntersectWithWhcihSide = 5;
        return false; 
    }
    // Check the intersect with top side.
    if (doIntersect(line,topLeft,topRight)){
        IntersectWithWhcihSide = 1;
        return true;
    }
    // Check the intersect with bottom side.
    if (doIntersect(line,bottomLeft,bottomRight)){
        IntersectWithWhcihSide = 2;
        return true;
    }
    // Check the intersect with left side.
    if (doIntersect(line,topLeft,bottomLeft)){
        IntersectWithWhcihSide = 3;
        return true;
    }
    // Check the intersect with right side.
    if (doIntersect(line,topRight,bottomRight)){
        IntersectWithWhcihSide = 4;
        return true;
    }
    IntersectWithWhcihSide = 0;
    return false;

}
int main(int argc, char *argv[]){
    //usage: ./eliminated_inner_boundary <input_file>(0521_all_net_coord_info.txt)
    ifstream file(argv[1]);
    ofstream outfile("eliminated_inner_boundary_result.txt");
    string line,str,str_start_x,str_start_y,str_end_x,str_end_y,str_;
    double int_start_x,int_start_y,int_end_x,int_end_y;
    int side = 0; //Decide which side the boundary intersect with. for inner rectangle. 1 for Top, 2 for Bottom , 3 for Left , 4 for Right 5 for inside Rectangle
    stringstream ss;
    vector<Segment> vec_boundary; // Boundary that produced by VD.
    vector<Segment> vec_net_coord; // Original net coordinates.
    vector<Segment> remain_boundary_coord; // Atfer eliminated redundant boundaries, the remain boundaries.
    vector<Segment> insideInnerRectSegment; //segments that in inner rect or intersect with inner rect which need to be eliminated.
    vector<Segment> outsideInnerRectSegment; //segments that outside inner rect which need to be remain.
    vector<Segment> eliminated_boundary;
    ss.clear();
    ss.str("");
    outfile << "all_net info start:\n";
    while(getline(file, line)){
        if(line == "all_net_coord_info_start:"){
            while(getline(file, line) ){
                if (line =="all_net_coord_info_end:"){
                    break;
                }
                ss << line;
                ss >> str;
                ss << line;
                ss >> str_start_x; 
                ss << line;
                ss >> str_start_y;
                ss << line;
                ss >>str_end_x;
                ss << line;
                ss >> str_end_y;
                int_start_x = stod(str_start_x);
                int_start_y = stod(str_start_y);
                int_end_x = stod(str_end_x);
                int_end_y = stod(str_end_y);
                vec_net_coord.emplace_back(int_start_x,int_start_y,int_end_x,int_end_y);
                ss.clear();
                ss.str("");
                }
            for (const auto & coords: vec_net_coord){
                outfile<<"net: "<<scientific<<coords.start.x<<" "<<coords.start.y<<" "<<coords.end.x<<" "<<coords.end.y<<endl;
                //outfile<<"_view.drawLine("<<scientific<<coords.start.x<<","<<coords.start.y<<","<<coords.end.x<<","<<coords.end.y<<");"<<endl;
            }
            outfile << "all_net info end\n";
            int_start_x = 0;
            int_start_y = 0;
            int_end_x = 0;
            int_end_y = 0;
        }
        // Filtering the boundaries that need to be eliminated.
        if(line == "using vertex iterator result start: "){
            while(getline(file, line)){
                if(line == "end of vertex iterator result")
                    break;
                ss << line;
                ss >> str;
                ss << line;
                ss >> str_start_x;
                ss << line;
                ss >> str_start_y;
                ss <<line;
                ss >> str_end_x;
                ss << line;
                ss >> str_end_y;
                int_start_x = stod(str_start_x);
                int_start_y = stod(str_start_y);
                int_end_x = stod(str_end_x);
                int_end_y = stod(str_end_y);
                bool boundaries_additional_check_start_type = false;
                bool boundaries_additional_check_end_type = false;
                vec_boundary.emplace_back(int_start_x,int_start_y,int_end_x,int_end_y);
                for(auto & coords: vec_boundary){
                    for(auto & net_coords: vec_net_coord){
                        if ((coords.start.x == net_coords.start.x) && (coords.start.y == net_coords.start.y)){
                            boundaries_additional_check_start_type = true;
                            eliminated_boundary.emplace_back(coords.start.x,coords.start.y,coords.end.x,coords.end.y);
                            break;
                        }
                        else if(coords.end.x == net_coords.end.x && coords.end.y == net_coords.end.y){
                            boundaries_additional_check_end_type = true;
                            eliminated_boundary.emplace_back(coords.start.x,coords.start.y,coords.end.x,coords.end.y);
                            break;
                        }
                        else if(coords.start.x == net_coords.end.x && coords.start.y == net_coords.end.y){
                            boundaries_additional_check_end_type = true;
                            eliminated_boundary.emplace_back(coords.start.x,coords.start.y,coords.end.x,coords.end.y);
                            break;
                        }
                        else if (coords.end.x == net_coords.start.x && coords.end.y == net_coords.start.y){
                            boundaries_additional_check_start_type = true;
                            eliminated_boundary.emplace_back(coords.start.x,coords.start.y,coords.end.x,coords.end.y);
                            break;
                        }
                    }
                }
            ss.clear();
            ss.str("");
            vec_boundary.clear();  
            }
        }
        ss.clear();
        ss.str("");
    }
    // Writnig the remain boundaries to the output file.
    file.clear();
    file.seekg(0,ios::beg);
    //outfile<<"remain boundaies:\n\n";
    while(getline(file, line)){
        if(line == "using vertex iterator result start: "){
            while(getline(file, line)){
                bool need_eliminated = false;
                if(line == "end of vertex iterator result")
                    break;
                ss << line;
                ss >> str;
                ss << line;
                ss >> str_start_x;
                ss << line;
                ss >> str_start_y;
                ss << line;
                ss >> str_end_x;
                ss << line;
                ss >> str_end_y;
                int_start_x = stod(str_start_x);
                int_start_y = stod(str_start_y);
                int_end_x = stod(str_end_x);
                int_end_y = stod(str_end_y);
                for(auto &eliminated_line: eliminated_boundary){
                    need_eliminated = false;
                    if(int_start_x == eliminated_line.start.x && int_start_y == eliminated_line.start.y && int_end_x == eliminated_line.end.x && int_end_y == eliminated_line.end.y){
                        need_eliminated = true;
                        break;
                    }
                }
                if(!need_eliminated){
                   // outfile<<"_view.drawLine("<<scientific<<int_start_x<<","<<int_start_y<<","<<int_end_x<<","<<int_end_y<<");"<<endl;
                    remain_boundary_coord.emplace_back(int_start_x,int_start_y,int_end_x,int_end_y);
                }
                ss.clear();
                ss.str("");
            }
        }
    }

    //filtering the edges that inside the inner rectangle or not.
    file.clear();
    file.seekg(0,ios::beg);
    //rect = {rect_Bottomleft.x,rect_Bottomleft.y,rect_Width.rect_Height}.
    Rectangle rect = {-5e+07,-5e+07,1e+08,1e+08};
    for (auto & coords : remain_boundary_coord){
        if(!isIntersecting(coords,rect)){
            outsideInnerRectSegment.emplace_back(coords.start.x,coords.start.y,coords.end.x,coords.end.y);
            //cout<<"_view.drawLine("<<scientific<<coords.start.x<<","<<coords.start.y<<","<<coords.end.x<<","<<coords.end.y<<");"<<endl;
        }
        else
            insideInnerRectSegment.emplace_back(coords.start.x,coords.start.y,coords.end.x,coords.end.y);
    }
    // Check the segments that inside inner rectangle need to be truncated or need to be eliminated.
    Point rect_bottomLeft = {rect.rect_x,rect.rect_y}; //rect_1
    Point rect_topRight = {rect.rect_x + rect.rect_w,rect.rect_y + rect.rect_h}; //rect_2
    Point rect_topLeft = {rect.rect_x,rect.rect_y + rect.rect_h};
    Point rect_bottomRight = {rect.rect_x + rect.rect_w,rect.rect_y};
    for (auto & coords : insideInnerRectSegment){
        side = 0;
        if(isLineIntersectRectangle(rect_bottomLeft,rect_topRight,coords,side)){
            switch(side){
                //case1 : Intersect with Top side
                case 1:
                    if (coords.start.y < rect_topRight.y){
                        coords.start.y = rect_topRight.y;
                    }
                    else if(coords.end.y < rect_topRight.y){
                        coords.end.y = rect_topRight.y;
                    }
                    break;
                //case 2 : Intersect with Bottom side
                case 2:
                    if(coords.start.y > rect_bottomRight.y){
                        coords.start.y = rect_bottomRight.y;
                    }
                    else if (coords.end.y > rect_bottomRight.y){
                        coords.end.y = rect_bottomRight.y;
                    }
                    break;
                //case 3 : Intersect with Left side
                case 3:
                    if(coords.start.x > rect_bottomLeft.x){
                        coords.start.x = rect_bottomLeft.x;
                    }
                    else if (coords.end.x > rect_bottomLeft.x){
                        coords.end.x = rect_bottomLeft.x;
                    }
                    break;
                //case 4 : Intersect with Right side
                case 4:
                    if(coords.start.x < rect_bottomRight.x){
                        coords.start.x = rect_bottomRight.x;
                    }
                    else if (coords.end.x < rect_bottomRight.x){
                        coords.end.x = rect_bottomRight.x;
                    }
                    break;
               // case 5:
               //     cout<<"Inside the rectangle"<<endl;
             //       break;
            }
            //NeedBeExtendedBoundaries.emplace_back(coords.start.x,coords.start.y,coords.end.x,coords.end.y);
            outfile<<"boundary_segment: "<<scientific<<coords.start.x<<" "<<coords.start.y<<" "<<coords.end.x<<" "<<coords.end.y<<endl;
            //outfile<<"_view.drawLine("<<scientific<<coords.start.x<<","<<coords.start.y<<","<<coords.end.x<<","<<coords.end.y<<");"<<endl;
        }
        /*else 
            cout<<"No intersect with the rectangle"<<endl;*/
    }

    for (auto & coords : outsideInnerRectSegment){
        side = 0;
        if(isLineIntersectRectangle(rect_bottomLeft,rect_topRight,coords,side)){
            switch(side){
                //case1 : Intersect with Top side
                case 1:
                    if (coords.start.y < rect_topRight.y){
                        coords.start.y = rect_topRight.y;
                    }
                    else if(coords.end.y < rect_topRight.y){
                        coords.end.y = rect_topRight.y;
                    }
                    break;
                //case 2 : Intersect with Bottom side
                case 2:
                    if(coords.start.y > rect_bottomRight.y){
                        coords.start.y = rect_bottomRight.y;
                    }
                    else if (coords.end.y > rect_bottomRight.y){
                        coords.end.y = rect_bottomRight.y;
                    }
                    break;
                //case 3 : Intersect with Left side
                case 3:
                    if(coords.start.x > rect_bottomLeft.x){
                        coords.start.x = rect_bottomLeft.x;
                    }
                    else if (coords.end.x > rect_bottomLeft.x){
                        coords.end.x = rect_bottomLeft.x;
                    }
                    break;
                //case 4 : Intersect with Right side
                case 4:
                    if(coords.start.x < rect_bottomRight.x){
                        coords.start.x = rect_bottomRight.x;
                    }
                    else if (coords.end.x < rect_bottomRight.x){
                        coords.end.x = rect_bottomRight.x;
                    }
                    break;
               // case 5:
               //     cout<<"Inside the rectangle"<<endl;
             //       break;
            }
        }
        //NeedBeExtendedBoundaries.emplace_back(coords.start.x,coords.start.y,coords.end.x,coords.end.y);
        outfile<<"boundary_segment: "<<scientific<<coords.start.x<<" "<<coords.start.y<<" "<<coords.end.x<<" "<<coords.end.y<<endl;
        //outfile<<"_view.drawLine("<<scientific<<coords.start.x<<","<<coords.start.y<<","<<coords.end.x<<","<<coords.end.y<<");"<<endl;
    }
    eliminated_boundary.clear();
    remain_boundary_coord.clear();
    insideInnerRectSegment.clear();
    outsideInnerRectSegment.clear();
    vec_net_coord.clear();
    vec_boundary.clear();
    file.close();
    outfile.close();
    return 0;
}

