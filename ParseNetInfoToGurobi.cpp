#include "ParseNetInfoToGurobi.h"
Rectangle::Rectangle():
    rect_x(-5e+07),rect_y(-5e+07),rect_w(1e+08),rect_h(1e+08)
    {}

netInfo::netInfo():
    netID(-1),
    pad_x(0.0), pad_y(0.0),
    ball_x(0.0), ball_y(0.0),
    boundary0ID(-1),boundary1ID(-1),
    area(0.0),areaInitial(0.0)
    {}

commonBoundary::commonBoundary(): 
    boundaryID(-1),
    netA(-1),netB(-1),
    alpha(0.0),alpha_corner(0.0),boundary_move_direction(-1),
    initial_route_alpha(0.0),
    shiftAmount(0),shiftMin(0),shiftMax(0),shiftMax_corner(0),cornerArea(0.0)
    {}
void cal_corner_area(commonBoundary &CB, line_t &corner_line,commonBoundary &temp_CB, const double &original_area, const double &unit){
    /*
    //處理initial route要平移到corner net狀況
    double offset = 0.0;
    if(corner_point.x() == temp_line2.front().x()){
        offset = corner_point.y() - temp_line2.front().y();
        std::cout<<"offset = "<<offset<<std::endl;
        for(auto &point : temp_line2){
            point.y(point.y() + offset);
        }

    }
    else if(corner_point.y() == temp_line2.front().y()){
        offset = corner_point.x() - temp_line2.front().x();
        std::cout<<"offset = "<<offset<<std::endl;
        for(auto &point : temp_line2){
            point.x(point.x() + offset);
        }
    }
    */
    double  slash_area = 0;
    polygon_t poly, slash_poly;
    std::string reason;
    line_t temp_line2 = corner_line;
    point_t outter_corner_point = temp_line2.back();
    point_t inner_corner_point = corner_line.front();
    //原邊界線 原面積
    /*
    bg::append(poly, inner_corner_point);
    for(const auto &point: temp_CB.boundarySegment){
        bg::append(poly, point);
    }
    bg::append(poly, outter_corner_point);
    for(auto it = CB.boundarySegment.rbegin(); it != CB.boundarySegment.rend(); it++){
        bg::append(poly, *it);
    }
    bg::unique(poly);
    bg::correct(poly);
    std::cout<<"poly WKT: "<<bg::wkt(poly)<<std::endl;
    if (!bg::is_valid(poly, reason)) {
        std::cout << "Original_Polygon is invalid .: " << reason << std::endl;
    }
    */

    //slash_corner
    bg::append(slash_poly.outer(),inner_corner_point);
    for(const auto &point : temp_CB.boundarySegment){
        bg::append(slash_poly.outer(), point);
    }
    bg::append(slash_poly.outer(),outter_corner_point);
    for(auto it = temp_line2.rbegin(); it != temp_line2.rend(); it++){
        bg::append(slash_poly.outer(), *it);
    }
    bg::unique(slash_poly);
    bg::correct(slash_poly);
    std::cout<<"slash_poly WKT: "<<bg::wkt(slash_poly)<<std::endl;
    if (!bg::is_valid(slash_poly, reason)) {
        std::cout << "Slash_Polygon is invalid: " << reason << std::endl;
    }
    //計算面積
    //original_area = bg::area(poly)*1e-08;
    slash_area = bg::area(slash_poly)*1e-08;
    CB.cornerArea = original_area - slash_area;
    std::cout<<"original_area = "<<original_area<<std::endl;
    std::cout<<"slash_area = "<<slash_area<<std::endl;
    std::cout<<"corner_area = "<<CB.cornerArea<<std::endl;
   /* 
    if(line1.back().x() != line2.back().x() && line1.back().y() != line2.back().y()){
        point_t outterPoint(0.0,0.0),temp_point(0.0,0.0),temp_point2(0.0,0.0);
        temp_point = line1.back();
        temp_point2 = line2.back();
        if(abs(temp_point.x()) > abs(temp_point2.x())){
            outterPoint.x(temp_point.x());
        }
        else{
            outterPoint.x(temp_point2.x());
        }
        if(abs(temp_point.y()) > abs(temp_point2.y())){
            outterPoint.y(temp_point.y());
        }
        else{
            outterPoint.y(temp_point2.y());
        }
        bg::append(poly, outterPoint);
    }
    //corner_line
    for(auto it = temp_line2.rbegin(); it != temp_line2.rend(); it++){
        bg::append(poly, *it);
    }
    bg::correct(poly);
    */

    //alpha
    //startPoint與corner net同點, 直接以corner line替代
    if(CB.startPoint.x() == corner_line.front().x() && CB.startPoint.y() == corner_line.front().y()){
        CB.alpha = 0.0;
    }
    //在y軸仍有平移的空間
    else if(CB.startPoint.x() == corner_line.front().x()){
        CB.alpha = cal_shifted_area(CB.boundarySegment,unit,1); 
    }
    //在x軸仍有平移的空間
    else if(CB.startPoint.y() == corner_line.front().y()){
        CB.alpha = cal_shifted_area(CB.boundarySegment,unit,0);
    }
    //boundary_move_direction, alpha_corner
    if(CB.shiftAmount > 0){
        CB.boundary_move_direction = 0;
        //代表corner _line在y座標平移
        if(corner_line.front().x() == CB.netA_pad.x()){
            CB.alpha_corner = cal_shifted_area(corner_line,unit,1);
        }
        //代表corner_line在x座標平移
        else if(corner_line.front().y() == CB.netA_pad.y()){
            CB.alpha_corner = cal_shifted_area(corner_line,unit,0);
        }
    }
    else if(CB.shiftAmount < 0){
        CB.boundary_move_direction = 1;
        if(corner_line.front().x() == CB.netB_pad.x()){
            CB.alpha_corner = cal_shifted_area(corner_line,unit,1);
        }
        else if(corner_line.front().y() == CB.netB_pad.y()){
            CB.alpha_corner = cal_shifted_area(corner_line,unit,0);
        }
    }
}
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
    for(const auto &point : line){
        bg::append(poly, point);
    }
    for(auto it = shifted_line.rbegin(); it != shifted_line.rend(); it++){
        bg::append(poly, *it);
    }
    bg::unique(poly);
    bg::correct(poly);
    
    std::string reason;
    if (!bg::is_valid(poly, reason)) {
        //std::cout << "Polygon is valid" << std::endl;
        std::cout<<"Poly WKT: "<<bg::wkt(poly)<<std::endl;
        std::cout << "Polygon is invalid: " << reason << std::endl;
    }
    
    area = bg::area(poly)*1e-08;
    return area;

}
std::pair<int, double> getNetEdgeKey(const commonBoundary &CB, const Rectangle &r) {
    // 先計算四條邊的世界座標
    double leftX   = r.rect_x;
    double rightX  = r.rect_x + r.rect_w;
    double bottomY = r.rect_y;
    double topY    = r.rect_y + r.rect_h;

    double px = CB.startPoint.x() ;
    double py = CB.startPoint.y();

    // 由於浮點運算易有誤差，故用 EPS 來判斷「幾乎相等」
    const double EPS = 1e-9;

    // 上邊 (編號 0):    y = topY   ，x 從小到大
    if (std::fabs(py - topY) < EPS) {
        return {0, px};
    }
    // 右邊 (編號 1):    x = rightX ，y 從大到小 => key = -y
    else if (std::fabs(px - rightX) < EPS) {
        return {1, -py};
    }
    // 下邊 (編號 2):    y = bottomY，x 從大到小 => key = -x
    else if (std::fabs(py - bottomY) < EPS) {
        return {2, -px};
    }
    // 左邊 (編號 3):    x = leftX  ，y 從小到大 => key = y
    else {
        return {3, py};
    }
}

// 對 commonBoundaries 進行順時針排序的函式
void sortBoundaryClockwise(std::vector<commonBoundary> &commonBoundaries, const Rectangle &InnerRect) {
    // 如果沒有 net，直接 return
    if (commonBoundaries.empty()) return;

    // 以順時針「上邊→右邊→下邊→左邊」為基準進行排序
    std::sort(commonBoundaries.begin(), commonBoundaries.end(), [&](const commonBoundary &a, const commonBoundary &b) {
        auto keyA = getNetEdgeKey(a, InnerRect);
        auto keyB = getNetEdgeKey(b, InnerRect);

        // 1. 先比邊編號
        if (keyA.first != keyB.first) {
            return keyA.first < keyB.first;
        }
        // 2. 若邊相同，再比此邊上的排序鍵
        return keyA.second < keyB.second;
    });
}
//Update CommonBounary info: alpha,shift_Direction,shiftAmount netInfo.boundary0ID, boundary1ID
void UpdateCommonBoundaryInfo(std::vector<commonBoundary> &commonBoundaries, std::vector<netInfo> &nets){
    std::ifstream file("Gurobi_area_assignment_result.txt");
    std::string line;
    int rows = commonBoundaries.size(), cols = 2;
    int temp_value = 0,row = 0,col = 0;
    double unit = 1;
    //std::vector<std::vector<int>> matrix(rows,std::vector<int> (cols));
    std::vector<std::vector<int>> matrix(rows,std::vector<int>(2));
    //特例處理 commonBoundary Boundary ID 0
    
    int temp_id = 0, temp_BoundaryID = -1;
    point_t temp_point;
    temp_id = commonBoundaries[0].netA;
    commonBoundaries[0].netA = commonBoundaries[0].netB;
    commonBoundaries[0].netB = temp_id;

    temp_point = commonBoundaries[0].netA_pad;
    commonBoundaries[0].netA_pad = commonBoundaries[0].netB_pad;
    commonBoundaries[0].netB_pad = temp_point;
    
    commonBoundaries[0].boundaryID = 0;
    commonBoundaries[1].boundaryID = 1;
    
    auto modIdx = [&] (int idx){
        return (idx+rows) % rows;
    };
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
    //shift_Direction shiftMax,shiftMin,boundary_move_direction
    for (auto &commonBoundary:commonBoundaries){
        //在x座標平移
        if(commonBoundary.netA_pad.y() == commonBoundary.netB_pad.y()){
            commonBoundary.shift_Direction = 0;
            commonBoundary.shiftMax_corner = 0;
            //shiftMin = netA_pad.x - startPoint.x 
            //shiftMax = netB_pad.x - startPoint.x
            //commonBoundary.shiftMin = commonBoundary.netA_pad.x() - commonBoundary.startPoint.x();
            //commonBoundary.shiftMax = commonBoundary.netB_pad.x() - commonBoundary.startPoint.x();
            commonBoundary.shiftMin = 0;
            // boundary_move_direction根據shiftAmount的正負來決定要將面積調整至哪個pad的方向
	        //distance model預設為euclidean model(直線距離)
            if(commonBoundary.shiftAmount > 0){
                commonBoundary.shiftMax = bg::distance(commonBoundary.netA_pad,commonBoundary.startPoint);
		        commonBoundary.boundary_move_direction = 0;
            }
            else {
            	commonBoundary.shiftMax = bg::distance(commonBoundary.netB_pad,commonBoundary.startPoint);
		        commonBoundary.boundary_move_direction = 1;
            }
        }
        //在y座標平移
        else if (commonBoundary.netA_pad.x() == commonBoundary.netB_pad.x()){
            commonBoundary.shift_Direction = 1;
            commonBoundary.shiftMax_corner = 0;
            //commonBoundary.shiftMin = commonBoundary.netA_pad.y() - commonBoundary.startPoint.y();
            //commonBoundary.shiftMax = commonBoundary.netB_pad.y() - commonBoundary.startPoint.y();
            commonBoundary.shiftMin = 0;
            if(commonBoundary.shiftAmount > 0){
                commonBoundary.shiftMax = bg::distance(commonBoundary.netA_pad,commonBoundary.startPoint);
		        commonBoundary.boundary_move_direction = 0;
            }
            else {
            	commonBoundary.shiftMax = bg::distance(commonBoundary.netB_pad,commonBoundary.startPoint);
		        commonBoundary.boundary_move_direction = 1;
            }
        }
        //corner net 在這更新initialRouteSegment
        //根據shiftamount 正負值來決定initialRouteSegment要用哪個net的
        //shiftAmount > 0 採用 netA的ExtendedInitialRoute, shiftAmount < 0 採用 netB的ExtendedInitialRoute 
        else{
            commonBoundary.shift_Direction = 2;
            point_t outter_corner_point(1.4e+08,1.4e+08);
            //先排除掉pad沒有對齊inner boundary的情況
            //紀錄哪條netInfo需要更新
            int netID_needUpdate = -1;
            //紀錄net更新時要參照commonBoundary的netA或netB的pad 0 -> netA, 1 -> netB
            int referenceNet = 0;
            //代表Boundary的某個net pad 座標需移動到inner boundary上
            if((commonBoundary.netA_pad.x() != commonBoundary.netB_pad.y()) || (commonBoundary.netA_pad.y() != commonBoundary.netB_pad.y())){
                //代表netB pad需移動
                if((commonBoundary.startPoint.x() == commonBoundary.netA_pad.x()) || (commonBoundary.startPoint.y() == commonBoundary.netA_pad.y())){
                    netID_needUpdate = commonBoundary.netB;
                    referenceNet = 1;
                    //直接以inner boundary的最大座標值移動
                    double netB_xOffset = 5e+07 - abs( commonBoundary.netB_pad.x() );
                    double netB_yOffset = 5e+07 - abs( commonBoundary.netB_pad.y() );
                    //代表net B pad應該往y座標平移至innerBoundary比較近
                    if(netB_xOffset > netB_yOffset){
                        if(commonBoundary.netB_pad.y() < 0){
                            netB_yOffset = 0 - netB_yOffset;
                        }
                        commonBoundary.netB_pad.y(commonBoundary.netB_pad.y() + netB_yOffset) ;
                    }
                    else if(netB_xOffset < netB_yOffset){
                        if(commonBoundary.netB_pad.x() < 0){
                            netB_xOffset = 0 - netB_xOffset;
                        }
                        commonBoundary.netB_pad.x(commonBoundary.netB_pad.x() + netB_xOffset) ;
                    }
                }
                //代表netA pad需移動
                else if (commonBoundary.startPoint.x() == commonBoundary.netB_pad.x() || commonBoundary.startPoint.y() == commonBoundary.netB_pad.y()){
                    netID_needUpdate = commonBoundary.netA;
                    referenceNet = 0;
                    //直接以inner boundary的最大座標值移動
                    double netA_xOffset = 5e+7 - abs( commonBoundary.netA_pad.x());
                    double netA_yOffset = 5e+7 - abs( commonBoundary.netA_pad.y());
                    if(netA_xOffset > netA_yOffset){
                        if(commonBoundary.netA_pad.y() < 0){
                            netA_yOffset = 0 - netA_yOffset;
                        }
                        commonBoundary.netA_pad.y(commonBoundary.netA_pad.y() + netA_yOffset) ;
                    }
                    else if(netA_xOffset < netA_yOffset){
                        if(commonBoundary.netA_pad.x() < 0){
                            netA_xOffset = 0 - netA_xOffset;
                        }
                        commonBoundary.netA_pad.x(commonBoundary.netA_pad.x() + netA_xOffset) ;
                    }
                }
                //更新受影響的Boundary: shift_Direction,shiftMax,boundary_move_direction
                if(commonBoundary.netA_pad.y() == commonBoundary.netB_pad.y()){
                    commonBoundary.shift_Direction = 0;
                    commonBoundary.shiftMin = 0;
                    if(commonBoundary.shiftAmount > 0){
                        commonBoundary.shiftMax = bg::distance(commonBoundary.netA_pad,commonBoundary.startPoint);
		                commonBoundary.boundary_move_direction = 0;
                    }
                    else {
            	        commonBoundary.shiftMax = bg::distance(commonBoundary.netB_pad,commonBoundary.startPoint);
		                commonBoundary.boundary_move_direction = 1;
                    }
                }
                //在y座標平移
                else if (commonBoundary.netA_pad.x() == commonBoundary.netB_pad.x()){
                    commonBoundary.shift_Direction = 1;
                    commonBoundary.shiftMin = 0;
                    if(commonBoundary.shiftAmount > 0){
                        commonBoundary.shiftMax = bg::distance(commonBoundary.netA_pad,commonBoundary.startPoint);
		                commonBoundary.boundary_move_direction = 0;
                    }
                    else {
            	        commonBoundary.shiftMax = bg::distance(commonBoundary.netB_pad,commonBoundary.startPoint);
		                commonBoundary.boundary_move_direction = 1;
                    }
                }
                //更新受影響的net: pad,pad_x,pad_y
                for(auto &net:nets){
                    if(net.netID == netID_needUpdate){
                        if(referenceNet == 0){
                            net.pad = commonBoundary.netA_pad;
                            net.pad_x = net.pad.x();
                            net.pad_y = net.pad.y();
                        }
                        else if(referenceNet == 1){
                            net.pad = commonBoundary.netB_pad;
                            net.pad_x = net.pad.x();
                            net.pad_y = net.pad.y();
                             }
                    }
                }
            }//end of pad移動
           //移動pad過後再次檢查哪些net是corner net並排除掉那些不是corner的 
            if((commonBoundary.netA_pad.y() == commonBoundary.netB_pad.y())||(commonBoundary.netA_pad.x() == commonBoundary.netB_pad.x())){
                commonBoundary.shiftAmount = abs(commonBoundary.shiftAmount);
                continue;
            }
            //找出每個corner net的轉角座標
            point_t corner_point;
            //corner_point
            if(abs(commonBoundary.netA_pad.x()) > abs(commonBoundary.netB_pad.x())){
                corner_point.x(commonBoundary.netA_pad.x());
            }
            else{
                corner_point.x(commonBoundary.netB_pad.x());
            }
            if (abs(commonBoundary.netA_pad.y()) > abs(commonBoundary.netB_pad.y())){
                corner_point.y(commonBoundary.netA_pad.y());
            }
            else{
                corner_point.y(commonBoundary.netB_pad.y());
            }
            //outter_corner_point
            //若coner net不須轉角則同一般邊界線處理無須額外計算
            //更新corner net的alpha,shift max (起始點到轉角點的距離)
            //代表邊界線要往net A移動
            if(commonBoundary.shiftAmount > 0){
                //表示邊界線與目標pad在同一條線上不須考慮轉角 shift_min = 0,shift_max = 目標pad
                //start point與net A y座標相同, 代表在x座標上平移
                if((commonBoundary.startPoint.y() == commonBoundary.netA_pad.y()) && 
                        ((commonBoundary.startPoint.x() != corner_point.x()) && (commonBoundary.startPoint.y() != corner_point.y()))){
                    commonBoundary.shift_Direction = 0;
                    commonBoundary.shiftMin = 0;
                    commonBoundary.shiftMax = commonBoundary.netA_pad.x() - commonBoundary.startPoint.x();
                }
                //start point與net A x座標相同, 代表在y座標上平移
                else if((commonBoundary.startPoint.x() == commonBoundary.netA_pad.x()) &&
                        ((commonBoundary.startPoint.x() != corner_point.x()) && (commonBoundary.startPoint.y() != corner_point.y()))){
                    commonBoundary.shift_Direction = 1;
                    commonBoundary.shiftMin = 0;
                    commonBoundary.shiftMax = commonBoundary.netA_pad.y() - commonBoundary.startPoint.y();
                }
                //否則要考慮轉角
                else{
                    //std::cout<<"BoundaryID: "<<commonBoundary.boundaryID<<std::endl;   
                    commonBoundary.shift_Direction = 2;
                    line_t corner_line;
                    if(corner_point.x() > 0 && corner_point.y() < 0){
                        outter_corner_point.y(0-outter_corner_point.y());

                    }
                    else if (corner_point.x() < 0 && corner_point.y() > 0){
                        outter_corner_point.x(0-outter_corner_point.x());
                    }
                    else if (corner_point.x() < 0 && corner_point.y() < 0){
                        outter_corner_point.y(0-outter_corner_point.y());
                        outter_corner_point.x(0-outter_corner_point.x());
                    }
                    corner_line.push_back(corner_point);
                    corner_line.push_back(outter_corner_point);
                    commonBoundary.shiftMin = bg::distance(commonBoundary.startPoint,corner_point);
                    commonBoundary.shiftMax = bg::distance(corner_point,commonBoundary.netA_pad);
                    double original_area = nets[commonBoundary.netA].areaInitial;
                    auto temp_CB = commonBoundaries[modIdx(commonBoundary.boundaryID - 1)];
                    //commonBoundary.initialRouteSegment = nets[commonBoundary.netA].ExtendedInitialRoute;
                    cal_corner_area(commonBoundary,corner_line,temp_CB,original_area,unit);
                }
            }
            
            
            //代表邊界線要往net B移動
            else if(commonBoundary.shiftAmount < 0){
                //表示邊界線與目標pad在同一條線上不須考慮轉角 shift_min = 0,shift_max = 目標pad
                if((commonBoundary.startPoint.y() == commonBoundary.netB_pad.y())&&
                        ((commonBoundary.startPoint.x() != corner_point.x() && commonBoundary.startPoint.y() != corner_point.y()))){
                    commonBoundary.shift_Direction = 0;
                    commonBoundary.shiftMin = 0;
                    commonBoundary.shiftMax = commonBoundary.netB_pad.x() - commonBoundary.startPoint.x();
                }
                else if((commonBoundary.startPoint.x() == commonBoundary.netB_pad.x()) &&
                        ((commonBoundary.startPoint.x() != corner_point.x()) && (commonBoundary.startPoint.y() != corner_point.y()))){
                    commonBoundary.shift_Direction = 1;
                    commonBoundary.shiftMin = 0;
                    commonBoundary.shiftMax = commonBoundary.netB_pad.y() - commonBoundary.startPoint.y();
                }
                else{
                    //std::cout<<"BoundaryID: "<<commonBoundary.boundaryID<<std::endl;   
                    commonBoundary.shift_Direction = 2;
                    line_t corner_line;
                    if(corner_point.x() > 0 && corner_point.y() < 0){
                        outter_corner_point.y(0-outter_corner_point.y());

                    }
                    else if (corner_point.x() < 0 && corner_point.y() > 0){
                        outter_corner_point.x(0-outter_corner_point.x());
                    }
                    else if (corner_point.x() < 0 && corner_point.y() < 0){
                        outter_corner_point.y(0-outter_corner_point.y());
                        outter_corner_point.x(0-outter_corner_point.x());
                    }
                    corner_line.push_back(corner_point);
                    corner_line.push_back(outter_corner_point);
                    commonBoundary.shiftMin = bg::distance(commonBoundary.startPoint,corner_point);
                    commonBoundary.shiftMax = bg::distance(corner_point,commonBoundary.netB_pad);
                    auto temp_CB = commonBoundaries[modIdx(commonBoundary.boundaryID + 1)];
                    //commonBoundary.initialRouteSegment = nets[commonBoundary.netB].ExtendedInitialRoute;
                    double original_area = nets[commonBoundary.netB].areaInitial;
                    cal_corner_area(commonBoundary,corner_line,temp_CB,original_area,unit);
                }
            }
        }//end of shift_Direction = 2
	//為了後續LP_phase2計算方便,將所有shiftAmount採用絕對值
        commonBoundary.shiftAmount = abs(commonBoundary.shiftAmount);
    }
    //alpha,initial_route_alpha 
    for (auto &commonBoundary:commonBoundaries){
        //commonBoundary.shift_Direction = 0 or 1
        if(commonBoundary.shift_Direction == 0 || commonBoundary.shift_Direction == 1){
            commonBoundary.alpha = cal_shifted_area(commonBoundary.boundarySegment,unit,commonBoundary.shift_Direction);
            commonBoundary.alpha_corner = 0;
        }
    }
    //Update netInfo.boundary0ID, boundary1ID
    std::vector<bool> isBoundaryUsed(nets.size(),false);
    for (int i = 0; i < nets.size();i++){
        for (auto &commonBoundary:commonBoundaries){
            if(!isBoundaryUsed[i]){
                if(nets[i].startPoint0.x() == commonBoundary.startPoint.x() && nets[i].startPoint0.y() == commonBoundary.startPoint.y()){
                    nets[i].boundary0ID = commonBoundary.boundaryID;
                }
                else if(nets[i].startPoint1.x() == commonBoundary.startPoint.x() && nets[i].startPoint1.y() == commonBoundary.startPoint.y()){
                    nets[i].boundary1ID = commonBoundary.boundaryID;
                }
            }    
        }
        if(nets[i].boundary0ID!=-1 && nets[i].boundary1ID!=-1){
            isBoundaryUsed[i] = true;
        }
    }
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
        outfile<<"InitialRouteSegment: ";
        for(auto point:commonBoundary.initialRouteSegment){
            outfile<<bg::get<0>(point)<<" "<<bg::get<1>(point)<<" ";
        }
        outfile<<std::endl;
        outfile<<"NetA_pad: "<<bg::get<0>(commonBoundary.netA_pad)<<" "<<bg::get<1>(commonBoundary.netA_pad)<<std::endl;
        outfile<<"NetB_pad: "<<bg::get<0>(commonBoundary.netB_pad)<<" "<<bg::get<1>(commonBoundary.netB_pad)<<std::endl;
        outfile<<"Alpha: "<<commonBoundary.alpha<<std::endl;
        outfile<<"Alpha_corner: "<<commonBoundary.alpha_corner<<std::endl;
        outfile<<"corner_area: "<<commonBoundary.cornerArea<<std::endl;
        outfile<<"InitialRouteAlpha: "<<commonBoundary.initial_route_alpha<<std::endl;
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
        outfile<<"boundary0ID: "<<net.boundary0ID<<std::endl;
        outfile<<"boundary1ID: "<<net.boundary1ID<<std::endl;
        outfile<<"boundarySegment.size() = "<<net.boundarySegments.size()<<std::endl;
        outfile<<"startPoint0:"<<bg::get<0>(net.startPoint0)<<" "<<bg::get<1>(net.startPoint0)<<std::endl;
        outfile<<"startPoint1:"<<bg::get<0>(net.startPoint1)<<" "<<bg::get<1>(net.startPoint1)<<std::endl;
        outfile<<"innerBoundarySegment.size() = "<<net.innerBoundarySegments.size()<<std::endl;
        outfile<<"innerBoundarySegment: ";
        for(int i = 0;i < net.innerBoundarySegments.size();i++){
            for(auto point:net.innerBoundarySegments[i]){
                outfile<<point.x()<<" "<<point.y()<<" ";
            }
        } 
        outfile<<std::endl;
        outfile<<"outterBoundarySegment.size() = "<<net.outterBoundarySegments.size()<<std::endl;
        outfile<<"outterBoundarySegment: ";
        for(int i = 0;i < net.outterBoundarySegments.size();i++){
            for(auto point:net.outterBoundarySegments[i]){
                outfile<<point.x()<<" "<<point.y()<<" ";
            }
        }
        outfile<<std::endl;
        outfile<<"Area Initial: "<<net.areaInitial<<std::endl;
        outfile<<"Area: "<<net.area<<std::endl;
        outfile<<"actualArea: "<<net.areaInitial - net.area<<std::endl;
        outfile<<std::endl;
    }
}
void outputCommonBoundaries_drawing(std::vector<commonBoundary> &commonBoundaries){
    for(auto &boundary:commonBoundaries){
        for(int i = 0;i < boundary.boundarySegment.size(); i+=2){
            std::cout<<"_view.drawLine("<<boundary.boundarySegment[i].x()<<","<<boundary.boundarySegment[i].y()<<","<<
                boundary.boundarySegment[i+1].x()<<","<<boundary.boundarySegment[i+1].y()<<");\n";
        }
    }
}
void outputNetsInfo_drawing(std::vector<netInfo> &nets){
    for(auto &net:nets){
        //boundarySegment[0]
        for(int i = 0;i < net.boundarySegments[0].size(); i+=2){
            std::cout<<"_view.drawLine("<<net.boundarySegments[0][i].x()<<","<<net.boundarySegments[0][i].y()<<","<<
                net.boundarySegments[0][i+1].x()<<","<<net.boundarySegments[0][i+1].y()<<");\n";
        }
        //outterBoundarySegment
        for(int i = 0; i < net.outterBoundarySegments[0].size();i+=2){
            std::cout<<"_view.drawLine("<<net.outterBoundarySegments[0][i].x()<<","<<net.outterBoundarySegments[0][i].y()<<","<<
                net.outterBoundarySegments[0][i+1].x()<<","<<net.outterBoundarySegments[0][i+1].y()<<");\n";
        }
        if(net.outterBoundarySegments.size() == 2){
            for(int i = 0; i < net.outterBoundarySegments[1].size();i+=2){
                std::cout<<"_view.drawLine("<<net.outterBoundarySegments[1][i].x()<<","<<net.outterBoundarySegments[1][i].y()<<","<<
                    net.outterBoundarySegments[1][i+1].x()<<","<<net.outterBoundarySegments[1][i+1].y()<<");\n";
            }
        }
        //boundarySegment[1]
        for(int i = 0; i < net.boundarySegments[1].size();i+=2){
            std::cout<<"_view.drawLine("<<net.boundarySegments[1][i].x()<<","<<net.boundarySegments[1][i].y()<<","<<
                net.boundarySegments[1][i+1].x()<<","<<net.boundarySegments[1][i+1].y()<<");\n";
        }
        //innerBoundarySegment
        for(int i = 0; i < net.innerBoundarySegments[0].size(); i+=2){
            std::cout<<"_view.drawLine("<<net.innerBoundarySegments[0][i].x()<<","<<net.innerBoundarySegments[0][i].y()<<","<<
                net.innerBoundarySegments[0][i+1].x()<<","<<net.innerBoundarySegments[0][i+1].y()<<");\n";
        }
        if(net.innerBoundarySegments.size() == 2){
            for(int i = 0; i < net.innerBoundarySegments[1].size();i+=2){
                std::cout<<"_view.drawLine("<<net.innerBoundarySegments[1][i].x()<<","<<net.innerBoundarySegments[1][i].y()<<","<<
                    net.innerBoundarySegments[1][i+1].x()<<","<<net.innerBoundarySegments[1][i+1].y()<<");\n";
            }
        }
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
            currentNet.pad = point_t(currentNet.pad_x,currentNet.pad_y);
            continue;
        }
        //ball_location
        if(line.rfind("ball_location",0) == 0 && line.find(":")!=std::string::npos){
            std::stringstream ss (line.substr(14));
            ss>>currentNet.ball_x>>currentNet.ball_y;
            currentNet.ball = point_t(currentNet.ball_x,currentNet.ball_y);
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

void Phase2UpdateAllInfo_normal_nets(std::vector<double> &deltaVector,std::vector<commonBoundary> &commonBoundaries,std::vector<netInfo> &nets){
    std::vector<bool> isNetChanged(nets.size(),false);
    //更新所有commonBoundaries的boundarySegments, startPoint(邊界線本身)
    for(int i = 0;i < commonBoundaries.size();i++){
        int deltaBias_netA_x = 0, deltaBias_netB_x = 0;
        int deltaBias_netA_y = 0, deltaBias_netB_y = 0;
        deltaBias_netA_x =  commonBoundaries[i].startPoint.x() - commonBoundaries[i].netA_pad.x(); 
        deltaBias_netB_x =  commonBoundaries[i].startPoint.x() - commonBoundaries[i].netB_pad.x();
        deltaBias_netA_y = commonBoundaries[i].startPoint.y() - commonBoundaries[i].netA_pad.y();
        deltaBias_netB_y = commonBoundaries[i].startPoint.y() - commonBoundaries[i].netB_pad.y();
        //邊界線往netA移動
        if(commonBoundaries[i].boundary_move_direction == 0){
            if(deltaBias_netA_x > 0){
                deltaVector[i] = 0 - deltaVector[i];
            }
            if(deltaBias_netA_y > 0){
                deltaVector[i] = 0 - deltaVector[i];
            }
        }
        //邊界線往netB移動
        else if(commonBoundaries[i].boundary_move_direction == 1){
            if(deltaBias_netB_x > 0){
                deltaVector[i] = 0 - deltaVector[i];
            }
            if(deltaBias_netB_y > 0){
                deltaVector[i] = 0 - deltaVector[i];
            }
        }
        //在x座標上平移
        if(commonBoundaries[i].shift_Direction == 0){
            commonBoundaries[i].startPoint.x(commonBoundaries[i].startPoint.x() + deltaVector[i]);
            for(auto &point:commonBoundaries[i].boundarySegment){
                point.x(point.x() + deltaVector[i]);
            }
        }
        //在y座標上平移
        else if(commonBoundaries[i].shift_Direction == 1){
            commonBoundaries[i].startPoint.y(commonBoundaries[i].startPoint.y() + deltaVector[i]);
            for(auto &point:commonBoundaries[i].boundarySegment){
                point.y(point.y() + deltaVector[i]);
            }
        }
    //更新netInfo內容 1.startPoint 2.boundarySegments 3. innerBoundarySegment 4. outterBoundarySegment 5. arae initial
        for(int j = 0;j < nets.size(); j++){
            if(commonBoundaries[i].boundaryID == nets[j].boundary0ID){
                //start Point
                nets[j].startPoint0 = commonBoundaries[i].startPoint; 
                //boundarySegments
                nets[j].boundarySegments[0] = commonBoundaries[i].boundarySegment;
                isNetChanged[j] = true;
            }
            else if (commonBoundaries[i].boundaryID == nets[j].boundary1ID){
               nets[j].startPoint1 = commonBoundaries[i].startPoint; 
               nets[j].boundarySegments[1] = commonBoundaries[i].boundarySegment;
                isNetChanged[j] = true;
            }
            //inner & outterBoundarySegment
            if(isNetChanged[j]){
                //innerBoundarySegment
                if(nets[j].innerBoundarySegments.size() == 1){
                    nets[j].innerBoundarySegments[0].clear();
                    bg::append(nets[j].innerBoundarySegments[0],point_t(nets[j].startPoint0.x(),nets[j].startPoint0.y()));
                    bg::append(nets[j].innerBoundarySegments[0],point_t(nets[j].startPoint1.x(),nets[j].startPoint1.y()));
                }
                else if(nets[j].innerBoundarySegments.size() == 2){
                    point_t inner_CornerPoint(nets[j].innerBoundarySegments[0].back().x(),nets[j].innerBoundarySegments[0].back().y());
                    nets[j].innerBoundarySegments[0].clear();
                    nets[j].innerBoundarySegments[1].clear();
                    //innerSegments[0]
                    bg::append(nets[j].innerBoundarySegments[0],point_t(nets[j].startPoint0.x(),nets[j].startPoint0.y()));
                    bg::append(nets[j].innerBoundarySegments[0],inner_CornerPoint);
                    //innerBoundarySegment[1]
                    bg::append(nets[j].innerBoundarySegments[1],inner_CornerPoint);
                    bg::append(nets[j].innerBoundarySegments[1],point_t(nets[j].startPoint1.x(),nets[j].startPoint1.y()));
                }
                //outterBoundarySegment
                if(nets[j].outterBoundarySegments.size() ==1){
                    nets[j].outterBoundarySegments[0].clear();
                    bg::append(nets[j].outterBoundarySegments[0],point_t(nets[j].boundarySegments[0].back().x(),nets[j].boundarySegments[0].back().y()));
                    bg::append(nets[j].outterBoundarySegments[0],point_t(nets[j].boundarySegments[1].back().x(),nets[j].boundarySegments[1].back().y()));
                }
                else if(nets[j].outterBoundarySegments.size() == 2){
                    point_t outter_CornerPoint(nets[j].outterBoundarySegments[0].back().x(),nets[j].outterBoundarySegments[0].back().y());
                    nets[j].outterBoundarySegments[0].clear();
                    nets[j].outterBoundarySegments[1].clear();
                    //outterBoundarySegment[0]
                    bg::append(nets[j].outterBoundarySegments[0],point_t(nets[j].boundarySegments[0].back().x(),nets[j].boundarySegments[0].back().y()));
                    bg::append(nets[j].outterBoundarySegments[0],outter_CornerPoint);
                    //outterBoundarySegment[1]
                    bg::append(nets[j].outterBoundarySegments[1],outter_CornerPoint);
                    bg::append(nets[j].outterBoundarySegments[1],point_t(nets[j].boundarySegments[1].back().x(),nets[j].boundarySegments[1].back().y()));
                }
            isNetChanged[j] = false;
            }
        }
    }
    //更新所有nets的area
    for(int i = 0; i < nets.size(); i++){
        polygon_t poly;
        //boundary 0
        for(auto &point:nets[i].boundarySegments[0])
            bg::append(poly,point);
        //outter boundary
        if(nets[i].outterBoundarySegments.size() == 2){
            for(auto &point:nets[i].outterBoundarySegments[0]){
                bg::append(poly,point);
            }
            for(auto &point:nets[i].outterBoundarySegments[1]){
                bg::append(poly,point);
            }
        }
        //boundary 1
        for(auto it = nets[i].boundarySegments[1].rbegin(); it != nets[i].boundarySegments[1].rend(); it++)
            bg::append(poly,*it);
        if(nets[i].innerBoundarySegments.size() == 2){
            for(auto it = nets[i].innerBoundarySegments[1].rbegin(); it != nets[i].innerBoundarySegments[1].rend(); it++)
                bg::append(poly,*it);
            for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                bg::append(poly,*it);
        }
        bg::correct(poly);
        nets[i].area = bg::area(poly)*1e-08;
    }
}//end of Phase2UpdateAllInfo_normal_nets

/*
int main(){
return 0;
}
*/


int main(int argc, char* argv[]){
    std::ifstream file(argv[1]);
    std::vector<double> deltaVector ={7.8292e+06,0,1.00015e+07,3.1842e+06,0,1.21262e+07,0,1.12867e+07,3.23181e+06,0,5.2865e+06,1.49785e+07,9.959515e+06,1.06103e+07,0,2.31958e+07  } ;
    auto nets = parseNetsInfo(file);
    auto commonBoundaries = buildCommonBoundaries(nets);
    Rectangle innerRect;
    sortBoundaryClockwise(commonBoundaries,innerRect);
    //同時特例處利Boundary 1
    UpdateCommonBoundaryInfo(commonBoundaries,nets);
    //Phase2UpdateAllInfo_normal_nets(deltaVector,commonBoundaries,nets);
    
    outputNetsInfo(nets);
    outputCommonBoundaries(commonBoundaries);
    //outputCommonBoundaries_drawing(commonBoundaries);
    //outputNetsInfo_drawing(nets);
    file.close();
    return 0;
}


