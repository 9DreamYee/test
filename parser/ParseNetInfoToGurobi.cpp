#include "ParseNetInfoToGurobi.h"
Rectangle::Rectangle():
    //rect_x(-5e+07),rect_y(-5e+07),rect_w(1e+08),rect_h(1e+08)
    rect_x(0),rect_y(0),rect_w(0),rect_h(0)
    {}
Rectangle::Rectangle(double x, double y, double w, double h) :
    rect_x(x), rect_y(y), rect_w(w), rect_h(h) {}
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
    shiftAmount(0),shiftMin(0),shiftMax(0),shiftMax_corner(0),cornerArea(0.0),isSlash(0),
    phase2_deviation(-1.0),startPointDirection(-1),is_Intersect(false), in_Stage3(false)
    {}
//協助判斷特例處理邊界線起點與終點, 不在同個方向上平移的狀況
bool getCornerBoundaryDirection(const commonBoundary &CB, const Rectangle &r, const Rectangle &outterRect) {
    // 先計算四條邊的世界座標
    double leftX   = r.rect_x;
    double rightX  = r.rect_x + r.rect_w;
    double bottomY = r.rect_y;
    double topY    = r.rect_y + r.rect_h;
    double outter_leftX = outterRect.rect_x;
    double outter_rightX = outterRect.rect_x + outterRect.rect_w;
    double outter_bottomY = outterRect.rect_y;
    double outter_topY = outterRect.rect_y + outterRect.rect_h;

    double px = CB.startPoint.x();
    double py = CB.startPoint.y();
    double endx = CB.boundarySegment.back().x();
    double endy = CB.boundarySegment.back().y();
    // 0 -> 上邊, 1 -> 右邊, 2 -> 下邊, 3 -> 左邊
    int startPoint_Direction = -1, endPoint_Direction = -1;
    // 由於浮點運算易有誤差，故用 EPS 來判斷「幾乎相等」
    const double EPS = 1e-9;
    //判斷起點位在innerRect的哪條邊    
    // 上邊 (編號 0):    y = topY   ，x 從小到大
    if (std::fabs(py - topY) < EPS) {
        startPoint_Direction = 0;
    }
    // 右邊 (編號 1):    x = rightX ，y 從大到小 => key = -y
    else if (std::fabs(px - rightX) < EPS) {
        startPoint_Direction = 1;
    }
    // 下邊 (編號 2):    y = bottomY，x 從大到小 => key = -x
    else if (std::fabs(py - bottomY) < EPS) {
        startPoint_Direction = 2;
    }
    // 左邊 (編號 3):    x = leftX  ，y 從小到大 => key = y
    else {
        startPoint_Direction = 3;
    }
    //判斷終點位在outterRect的哪條邊
    if( std::fabs(endy - outter_topY) < EPS ){
        endPoint_Direction = 0;
    }
    else if (std::fabs(endx - outter_rightX) < EPS){
        endPoint_Direction = 1;
    }
    else if (std::fabs(endy - outter_bottomY) < EPS){
        endPoint_Direction = 2;
    }
    else {
        endPoint_Direction = 3;
    }
    //若起點與終點不在同個方向上 即為特例
    if(startPoint_Direction != endPoint_Direction){
        return true;
    }
    else{
        return false;
    } 
}
void cal_corner_area(commonBoundary &CB, line_t &corner_line,commonBoundary &temp_CB, const double &original_area, const double &unit,
       const Rectangle &innerRect, const Rectangle &outterRect ){
    //處理initial route要平移到corner point狀況
    double  slash_area = 0, ExtendedInitialRoute_area = 0;
    polygon_t poly, slash_poly;
    std::string reason;
    line_t temp_line2 = corner_line;
    point_t outter_corner_point = temp_line2.back();
    point_t inner_corner_point = corner_line.front();
    CB.cornerPoint = inner_corner_point;
    CB.cornerLine = corner_line;
    //將ExtendedInitialRoute平移到corner point
    double offset = 0.0;
    if(inner_corner_point.x() == CB.initialRouteSegment.front().x()){
        offset = abs(inner_corner_point.y()) - abs(CB.initialRouteSegment.front().y());
        for(auto &point : CB.initialRouteSegment){
            if(inner_corner_point.y() < CB.initialRouteSegment.front().y()){
                offset = 0 - offset;
            }
            if(abs(point.y() + offset) > abs(outter_corner_point.y())){
                point.y(outter_corner_point.y());
            }
            else{
                point.y(point.y() + offset);
            }
        }
    }
    else if(inner_corner_point.y() == CB.initialRouteSegment.front().y()){
        offset = abs(inner_corner_point.x()) - abs(CB.initialRouteSegment.front().x());
        for(auto &point : CB.initialRouteSegment){
            if(inner_corner_point.x() < CB.initialRouteSegment.front().x()){
                offset = 0 - offset;
            }
            if(abs(point.x() + offset) > abs(outter_corner_point.x())){
                point.x(outter_corner_point.x());
            }
            else{
                point.x(point.x() + offset);
            }
        }
    }
    //計算ExtendedInitialRoute所形成的面積
    //
    bg::append(poly.outer(),inner_corner_point);
    for(const auto &point : temp_CB.boundarySegment){
        bg::append(poly.outer(), point);
    }
    //bg::append(poly.outer(),outter_corner_point);
    for(auto it = CB.initialRouteSegment.rbegin(); it != CB.initialRouteSegment.rend(); it++){
        bg::append(poly.outer(), *it);
    }
    bg::unique(poly);
    bg::correct(poly);
    /*
    std::cout<<"poly WKT: "<<bg::wkt(poly)<<std::endl;
    if (!bg::is_valid(poly, reason)) {
        std::cout << "Original_Polygon is invalid .: " << reason << std::endl;
    }
    */
    ExtendedInitialRoute_area = bg::area(poly)*1e-08;
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
    /*
    std::cout<<"slash_poly WKT: "<<bg::wkt(slash_poly)<<std::endl;
    if (!bg::is_valid(slash_poly, reason)) {
        std::cout << "Slash_Polygon is invalid: " << reason << std::endl;
    }
    */
    //計算面積
    //original_area = bg::area(poly)*1e-08;
    slash_area = bg::area(slash_poly)*1e-08;
    //選擇最能接近shiftAmount的邊界線(斜線/ExtendedInitialRoute)
    
    double slash_cornerArea = original_area - slash_area;
    double ExtendedInitialRoute_cornerArea = original_area - ExtendedInitialRoute_area;
    double slash_shiftAmount = abs(CB.shiftAmount) - slash_cornerArea;
    double ExtendedInitialRoute_shiftAmount = abs(CB.shiftAmount) - ExtendedInitialRoute_cornerArea;
    //使用最接近shiftAmount的pattern
    if(abs(slash_shiftAmount) < abs(ExtendedInitialRoute_shiftAmount)){
        CB.cornerArea = original_area - slash_area;
        CB.isSlash = true;
    }
    else{
        CB.boundarySegment = CB.initialRouteSegment;
        //CB.cornerLine = CB.initialRouteSegment;
        corner_line = CB.initialRouteSegment;
        CB.cornerArea = original_area - ExtendedInitialRoute_area;
        CB.isSlash = false;
    }
    /*
    //全部統一採用initialRouteSegment
    if(abs(slash_shiftAmount) < abs(ExtendedInitialRoute_shiftAmount)){
        CB.boundarySegment = CB.initialRouteSegment;
        CB.cornerLine = CB.initialRouteSegment;
        //corner_line = CB.initialRouteSegment;
        CB.cornerArea = original_area - ExtendedInitialRoute_area;
        CB.isSlash = false;
    }
    else{
        CB.boundarySegment = CB.initialRouteSegment;
        //CB.cornerLine = CB.initialRouteSegment;
        corner_line = CB.initialRouteSegment;
        CB.cornerArea = original_area - ExtendedInitialRoute_area;
        CB.isSlash = false;
    }
    */
    //CB.cornerArea = original_area - slash_area;

    //alpha
    //特例處理邊界線起點與終點, 不在同個方向上平移的狀況
    bool isSpecialCase = getCornerBoundaryDirection(CB,innerRect,outterRect);
    /*
    if (isSpecialCase){
        std::cout<<"BoundaryID: "<<CB.boundaryID<<std::endl;
    }
    */
    //startPoint與corner net同點, 直接以corner line替代
    if((CB.startPoint.x() == corner_line.front().x()) && (CB.startPoint.y() == corner_line.front().y()) || isSpecialCase){
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
    /*
    std::string reason;
    if (!bg::is_valid(poly, reason)) {
        //std::cout << "Polygon is valid" << std::endl;
        std::cout<<"Poly WKT: "<<bg::wkt(poly)<<std::endl;
        std::cout << "Polygon is invalid: " << reason << std::endl;
    }
    */
    area = bg::area(poly)*1e-08;
    return area;

}
std::pair<int, double> getNetEdgeKey(commonBoundary &CB, const Rectangle &r) {
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
	CB.startPointDirection = 0;
        return {0, px};
    }
    // 右邊 (編號 1):    x = rightX ，y 從大到小 => key = -y
    else if (std::fabs(px - rightX) < EPS) {
	CB.startPointDirection = 1;
        return {1, -py};
    }
    // 下邊 (編號 2):    y = bottomY，x 從大到小 => key = -x
    else if (std::fabs(py - bottomY) < EPS) {
	CB.startPointDirection = 2;
        return {2, -px};
    }
    // 左邊 (編號 3):    x = leftX  ，y 從小到大 => key = y
    else {
	CB.startPointDirection = 3;
        return {3, py};
    }
}

// 對 commonBoundaries 進行順時針排序的函式
void sortBoundaryClockwise(std::vector<commonBoundary> &commonBoundaries, const Rectangle &InnerRect) {
    // 如果沒有 net，直接 return
    if (commonBoundaries.empty()) return;

    // 以順時針「上邊→右邊→下邊→左邊」為基準進行排序
    std::sort(commonBoundaries.begin(), commonBoundaries.end(), [&]( commonBoundary &a, commonBoundary &b) {
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
void UpdateCommonBoundaryInfo(std::vector<commonBoundary> &commonBoundaries, std::vector<netInfo> &nets, Rectangle &innerRect, Rectangle &outterRect){
    std::ifstream file("Gurobi_area_assignment_result.txt");
    std::string line;
    int rows = commonBoundaries.size(), cols = 2;
    int temp_value = 0,row = 0,col = 0;
    double unit = 1;
    //std::vector<std::vector<int>> matrix(rows,std::vector<int> (cols));
    std::vector<std::vector<int>> matrix(rows,std::vector<int>(2));
   
    //-------------特例處理 commonBoundary Boundary ID 0------------
    
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
   // ------------end of 特例處理-------- 
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
		double innerRect_biasX = innerRect.rect_x + innerRect.rect_w;
		double innerRect_biasY = innerRect.rect_y + innerRect.rect_h;
                //代表netB pad需移動
                if((commonBoundary.startPoint.x() == commonBoundary.netA_pad.x()) || (commonBoundary.startPoint.y() == commonBoundary.netA_pad.y())){
                    netID_needUpdate = commonBoundary.netB;
                    referenceNet = 1;
                    //直接以inner boundary的最大座標值移動
                    double netB_xOffset = innerRect_biasX - abs( commonBoundary.netB_pad.x() );
                    double netB_yOffset = innerRect_biasY - abs( commonBoundary.netB_pad.y() );
		    /*
                    double netB_xOffset = 5e+07 - abs( commonBoundary.netB_pad.x() );
                    double netB_yOffset = 5e+07 - abs( commonBoundary.netB_pad.y() );
                    */
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
		    double netA_xOffset = innerRect_biasX - abs( commonBoundary.netA_pad.x());
                    double netA_yOffset = innerRect_biasY - abs( commonBoundary.netA_pad.y());
		    //直接以inner boundary的最大座標值移動
                    /*
		    double netA_xOffset = 5e+07 - abs( commonBoundary.netA_pad.x());
                    double netA_yOffset = 5e+07 - abs( commonBoundary.netA_pad.y());
                    */
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
                    commonBoundary.initialRouteSegment = nets[commonBoundary.netA].ExtendedInitialRoute;
;                  cal_corner_area(commonBoundary,corner_line,temp_CB,original_area,unit,innerRect,outterRect);
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
                    commonBoundary.initialRouteSegment = nets[commonBoundary.netB].ExtendedInitialRoute;
                    double original_area = nets[commonBoundary.netB].areaInitial;
                    cal_corner_area(commonBoundary,corner_line,temp_CB,original_area,unit,innerRect,outterRect);
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
    outfile<<std::fixed<<std::setprecision(0);
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
        outfile<<"boundary_move_direction: "<<commonBoundary.boundary_move_direction<<std::endl;
        //outfile<<"InitialRouteAlpha: "<<commonBoundary.initial_route_alpha<<std::endl;
        outfile<<"ShiftDirection: "<<commonBoundary.shift_Direction<<std::endl;
        outfile<<"ShiftMin: "<<commonBoundary.shiftMin<<std::endl;
        outfile<<"ShiftMax: "<<commonBoundary.shiftMax<<std::endl;
        outfile<<"ShiftAmount: "<<commonBoundary.shiftAmount<<std::endl;
        outfile<<"phase2_deviation: "<<commonBoundary.phase2_deviation<<std::endl;
        outfile<<std::endl;
    }
}
void outputNetsInfo(std::vector<netInfo> &nets){
    std::ofstream outfile("netInfo_toGurobi.txt");
    double totalArea = 0,totalAreaInitial = 0;
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
	totalAreaInitial += net.areaInitial;
        outfile<<"Area: "<<net.area<<std::endl;
	totalArea += net.area;
        outfile<<"actualArea: "<<net.areaInitial - net.area<<std::endl;
        outfile<<std::endl;
    }
    outfile<<"totalAreaInitial: "<<totalAreaInitial<<std::endl;
    outfile<<"totalArea: "<<totalArea<<std::endl;
}
void outputPolyInfo(std::vector<netInfo> &nets){
    std::ofstream outfile("polyInfo_toBro.txt");
    std::ofstream outfile1("padandball_toBro.txt");
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
        else if(nets[i].outterBoundarySegments.size() == 1){
            for(auto &point:nets[i].outterBoundarySegments[0]){
                bg::append(poly,point);
            }
        }
        //boundary 1
        for(auto it = nets[i].boundarySegments[1].rbegin(); it != nets[i].boundarySegments[1].rend(); it++)
            bg::append(poly,*it);
        //inner boundary
        if(nets[i].innerBoundarySegments.size() == 2){
            for(auto it = nets[i].innerBoundarySegments[1].rbegin(); it != nets[i].innerBoundarySegments[1].rend(); it++)
                bg::append(poly,*it);
            for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                bg::append(poly,*it);
        }
        else if (nets[i].innerBoundarySegments.size() == 1 ){
            for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                bg::append(poly,*it);
        }
        bg::unique(poly);
        bg::correct(poly);
        outfile<<"Net "<<i<<": \n"<<boost::geometry::dsv(poly)<<std::endl;
        outfile1<<"("<<nets[i].pad_x<<", "<<nets[i].pad_y<<")"<<std::endl;
        outfile1<<"("<<nets[i].ball_x<<", "<<nets[i].ball_y<<")"<<std::endl;
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
    //pad & ball
    /*
    for(auto net:nets){
        std::cout<<"_view.drawArc("<<net.pad.x()<<","<<net.pad.y()<<",1000000);\n";
        std::cout<<"_view.drawArc("<<net.ball.x()<<","<<net.ball.y()<<",1000000);\n";	
    }
    */
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
std::vector<netInfo> parseNetsInfo(std::ifstream &file, double &resolution, Rectangle &innerRect, Rectangle &outterRect){
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
	// 0) Resolution
	if(line.rfind("Resolution:", 0) == 0){
	    std::istringstream is(line.substr(strlen("Resolution:")));
            is >> resolution;
	    continue;
	}	
	// 1) DieRect header
        if (line.rfind("DieRect:", 0) == 0) {
            std::istringstream is(line.substr(strlen("DieRect:")));
            double xmin, ymin, xmax, ymax;
            is >> xmin >> ymin >> xmax >> ymax;
            innerRect = Rectangle(xmin, ymin, xmax - xmin, ymax - ymin);
            continue;
        }
        // 2) FullRect header
        if (line.rfind("FullRect:", 0) == 0) {
            std::istringstream is(line.substr(strlen("FullRect:")));
            double xmin, ymin, xmax, ymax;
            is >> xmin >> ymin >> xmax >> ymax;
            outterRect = Rectangle(xmin, ymin, xmax - xmin, ymax - ymin);
            continue;
        }
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
void Phase2UpdateAllInfo_normal_nets(std::vector<double> &deltaVector,std::vector<int> &bVector, 
        std::vector<commonBoundary> &commonBoundaries,std::vector<netInfo> &nets, const Rectangle &InnerRect){
    std::vector<bool> isNetChanged(nets.size(),false);
    //更新所有commonBoundaries的boundarySegments, startPoint,startPointDirection(邊界線本身)
    for(int i = 0;i < commonBoundaries.size();i++){
        int deltaBias_netA_x = 0, deltaBias_netB_x = 0;
        int deltaBias_netA_y = 0, deltaBias_netB_y = 0;
        deltaBias_netA_x =  commonBoundaries[i].startPoint.x() - commonBoundaries[i].netA_pad.x(); 
        deltaBias_netB_x =  commonBoundaries[i].startPoint.x() - commonBoundaries[i].netB_pad.x();
        deltaBias_netA_y = commonBoundaries[i].startPoint.y() - commonBoundaries[i].netA_pad.y();
        deltaBias_netB_y = commonBoundaries[i].startPoint.y() - commonBoundaries[i].netB_pad.y();
        if(commonBoundaries[i].shift_Direction == 2){
           //以原邊界線平移
            if(bVector[i] == 0){
                deltaBias_netA_x = commonBoundaries[i].startPoint.x() - commonBoundaries[i].cornerPoint.x();
                deltaBias_netB_x = commonBoundaries[i].startPoint.x() - commonBoundaries[i].cornerPoint.x();
                deltaBias_netA_y = commonBoundaries[i].startPoint.y() - commonBoundaries[i].cornerPoint.y();
                deltaBias_netB_y = commonBoundaries[i].startPoint.y() - commonBoundaries[i].cornerPoint.y();
            }
           //以斜線平移
            else if(bVector[i] == 1){
                deltaBias_netA_x = commonBoundaries[i].cornerPoint.x() - commonBoundaries[i].netA_pad.x();
                deltaBias_netB_x = commonBoundaries[i].cornerPoint.x() - commonBoundaries[i].netB_pad.x();
                deltaBias_netA_y = commonBoundaries[i].cornerPoint.y() - commonBoundaries[i].netA_pad.y();
                deltaBias_netB_y = commonBoundaries[i].cornerPoint.y() - commonBoundaries[i].netB_pad.y();
            }
        }
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
        //corner_net
        else if (commonBoundaries[i].shift_Direction == 2){
           //以原邊界線平移
            if( bVector[i] == 0 ){
                //判斷在x/y軸上平移
                if(commonBoundaries[i].startPoint.x() == commonBoundaries[i].cornerPoint.x()){
                    //在y軸上平移
                    commonBoundaries[i].startPoint.y(commonBoundaries[i].startPoint.y() + deltaVector[i]);
                    for(auto &point:commonBoundaries[i].boundarySegment){
                        point.y(point.y() + deltaVector[i]);
                    }
                }
                else if(commonBoundaries[i].startPoint.y() == commonBoundaries[i].cornerPoint.y()){
                    //在x軸上平移
                    commonBoundaries[i].startPoint.x(commonBoundaries[i].startPoint.x() + deltaVector[i]);
                    for(auto &point:commonBoundaries[i].boundarySegment){
                        point.x(point.x() + deltaVector[i]);
                    }
                }
            }//end of bVector == 0
           //以斜線平移
            else if(bVector[i] == 1 ){
                if(commonBoundaries[i].isSlash){
                    commonBoundaries[i].boundarySegment.clear();
                    bg::append(commonBoundaries[i].boundarySegment,
                    point_t(commonBoundaries[i].cornerLine.front().x(),commonBoundaries[i].cornerLine.front().y()));
                    bg::append(commonBoundaries[i].boundarySegment,
                    point_t(commonBoundaries[i].cornerLine.back().x(),commonBoundaries[i].cornerLine.back().y()));
                }
                if(commonBoundaries[i].boundary_move_direction == 0){
                //判斷往netA平移
                    if(commonBoundaries[i].cornerPoint.x() == commonBoundaries[i].netA_pad.x()){
                        //在y軸上平移
                        commonBoundaries[i].startPoint.x(commonBoundaries[i].cornerPoint.x());
                        commonBoundaries[i].startPoint.y(commonBoundaries[i].cornerPoint.y() + deltaVector[i]);
                        for(auto &point:commonBoundaries[i].boundarySegment){
                            point.y(point.y() + deltaVector[i]);
                        }
                    }
                    else if (commonBoundaries[i].cornerPoint.y() == commonBoundaries[i].netA_pad.y()){
                        //在x軸上平移
                        commonBoundaries[i].startPoint.x(commonBoundaries[i].cornerPoint.x() + deltaVector[i]);
                        commonBoundaries[i].startPoint.y(commonBoundaries[i].cornerPoint.y());
                        for(auto &point:commonBoundaries[i].boundarySegment){
                            point.x(point.x() + deltaVector[i]);
                        }
                    }
                }
		//往netB平移
                else if(commonBoundaries[i].boundary_move_direction == 1){
                    if(commonBoundaries[i].cornerPoint.x() == commonBoundaries[i].netB_pad.x()){
                        //在y軸上平移
                        commonBoundaries[i].startPoint.x(commonBoundaries[i].cornerPoint.x());
                        commonBoundaries[i].startPoint.y(commonBoundaries[i].cornerPoint.y() + deltaVector[i]);
                        for(auto &point:commonBoundaries[i].boundarySegment){
                            point.y(point.y() + deltaVector[i]);
                        }
                    }
                    else if (commonBoundaries[i].cornerPoint.y() == commonBoundaries[i].netB_pad.y()){
                        //在x軸上平移
                        commonBoundaries[i].startPoint.x(commonBoundaries[i].cornerPoint.x() + deltaVector[i]);
                        commonBoundaries[i].startPoint.y(commonBoundaries[i].cornerPoint.y());
                        for(auto &point:commonBoundaries[i].boundarySegment){
                            point.x(point.x() + deltaVector[i]);
                        }
                    }
                }
            }
	    //startPointDirection
	    auto temp_key = getNetEdgeKey(commonBoundaries[i], InnerRect);
        }
    //更新netInfo內容 1.startPoint 2.boundarySegments 3. innerBoundarySegment 4. outterBoundarySegment 5. arae initial
        for(int j = 0;j < nets.size(); j++){
            if(commonBoundaries[i].boundaryID == nets[j].boundary0ID){
                //start Point
                nets[j].startPoint0 = commonBoundaries[i].startPoint; 
                //boundarySegments
		nets[j].boundarySegments[0].clear();
                nets[j].boundarySegments[0] = commonBoundaries[i].boundarySegment;
                isNetChanged[j] = true;
            }
            else if (commonBoundaries[i].boundaryID == nets[j].boundary1ID){
               nets[j].startPoint1 = commonBoundaries[i].startPoint;
	       nets[j].boundarySegments[1].clear(); 
               nets[j].boundarySegments[1] = commonBoundaries[i].boundarySegment;
                isNetChanged[j] = true;
            }
            //inner & outterBoundarySegment
            if(isNetChanged[j]){
                //corner net以斜線更新至兩條相關的net
                if(commonBoundaries[i].shift_Direction == 2 && bVector[i] == 1){
                    line_t tempLine;
                    if(nets[j].innerBoundarySegments.size() == 2){
                        nets[j].innerBoundarySegments.pop_back();
                    }
                    if(nets[j].outterBoundarySegments.size() == 2){
                        nets[j].outterBoundarySegments.pop_back();
                    }
		    //改用斜線後 兩條邊界線在不同軸
                    if((nets[j].startPoint0.x() != nets[j].startPoint1.x()) && (nets[j].startPoint0.y() != nets[j].startPoint1.y())){
                        //innerBoundarySegment
                        nets[j].innerBoundarySegments[0].clear();
                        bg::append(tempLine, point_t(commonBoundaries[i].cornerPoint.x(), commonBoundaries[i].cornerPoint.y()));
                        bg::append(tempLine, point_t(nets[j].startPoint1.x(), nets[j].startPoint1.y()));

                        bg::append(nets[j].innerBoundarySegments[0],point_t(nets[j].startPoint0.x(),nets[j].startPoint0.y()));
                        bg::append(nets[j].innerBoundarySegments[0],point_t(commonBoundaries[i].cornerPoint.x(), commonBoundaries[i].cornerPoint.y()));
                        nets[j].innerBoundarySegments.emplace_back(tempLine);
                        //outterBoundarySegment
                        nets[j].outterBoundarySegments[0].clear();
                        tempLine.clear();
                        bg::append(tempLine, point_t(commonBoundaries[i].cornerLine.back().x(), commonBoundaries[i].cornerLine.back().y()));
                        bg::append(tempLine, point_t(nets[j].boundarySegments[1].back().x(), nets[j].boundarySegments[1].back().y()));

                        bg::append(nets[j].outterBoundarySegments[0],point_t(nets[j].boundarySegments[0].back().x(),nets[j].boundarySegments[0].back().y()));
                        bg::append(nets[j].outterBoundarySegments[0],point_t(commonBoundaries[i].cornerLine.back().x(), 
                                    commonBoundaries[i].cornerLine.back().y())); 
                        nets[j].outterBoundarySegments.emplace_back(tempLine);
                    }
		    //改用斜線後兩條邊界線在同軸
		    else{
		   	//innerBoundarySegment
			nets[j].innerBoundarySegments[0].clear();
                        bg::append(nets[j].innerBoundarySegments[0],point_t(nets[j].startPoint0.x(),nets[j].startPoint0.y()));
                        bg::append(nets[j].innerBoundarySegments[0],point_t(nets[j].startPoint1.x(),nets[j].startPoint1.y()));
			//outterBoundarySegment 
			nets[j].outterBoundarySegments[0].clear();
                        bg::append(nets[j].outterBoundarySegments[0],point_t(nets[j].boundarySegments[0].back().x(),nets[j].boundarySegments[0].back().y()));
                        bg::append(nets[j].outterBoundarySegments[0],point_t(nets[j].boundarySegments[1].back().x(),nets[j].boundarySegments[1].back().y()));

		    }
                }
                //以normal net處理
                else{
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
        else if(nets[i].outterBoundarySegments.size() == 1){
            for(auto &point:nets[i].outterBoundarySegments[0]){
                bg::append(poly,point);
            }
        }
        //boundary 1
            for(auto it = nets[i].boundarySegments[1].rbegin(); it != nets[i].boundarySegments[1].rend(); it++)
            bg::append(poly,*it);
        //inner boundary
        if(nets[i].innerBoundarySegments.size() == 2){
            for(auto it = nets[i].innerBoundarySegments[1].rbegin(); it != nets[i].innerBoundarySegments[1].rend(); it++)
                bg::append(poly,*it);
            for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                bg::append(poly,*it);
        }
        else if (nets[i].innerBoundarySegments.size() == 1 ){
            for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                bg::append(poly,*it);
        }
        bg::unique(poly);
        bg::correct(poly);
        /*
    std::string reason;
    if (!bg::is_valid(poly, reason)) {
        std::cout<<"NetID: "<<nets[i].netID<<std::endl;
        std::cout<<"Poly WKT: "<<bg::wkt(poly)<<std::endl;
        std::cout << "Polygon is invalid: " << reason << std::endl;
    }
    */
        nets[i].area = bg::area(poly)*1e-08;
    }
}//end of Phase2UpdateAllInfo_normal_nets
 //計算從bendingPoint轉角的面積值
double cal_bendingPoint_area(line_t &temp_line, line_t &last_line, line_t &cornerLine){
    double area = 0.0;
    polygon_t temp_poly;
    for(auto &point:temp_line){
        bg::append(temp_poly,point);
    }
    
    //需考慮outter cornerPoint
    if(abs(temp_line.back().x()) == abs(last_line.back().y()) || abs(temp_line.back().y()) == abs(last_line.back().x())){
        bg::append(temp_poly, point_t(cornerLine[cornerLine.size()-1].x(),cornerLine[cornerLine.size()-1].y()));
    }
    
    for(auto it = last_line.rbegin(); it != last_line.rend(); it++){
        bg::append(temp_poly,*it);
    }
    bg::unique(temp_poly);
    bg::correct(temp_poly);
    std::cout<<"polyWKT: "<<bg::wkt(temp_poly)<<std::endl;
    std::string reason;
    if (!bg::is_valid(temp_poly, reason)) {
        std::cout << "Polygon is invalid: " << reason << std::endl;
    }
    area = bg::area(temp_poly)*1e-08;
    return area;
}
bool isIntersect_with_otherBoundary(const line_t &temp_line2,const commonBoundary &CB, const commonBoundary &adjacent_boundary, const std::vector<netInfo> &nets, std::vector<point_t> &intersectionPoints, point_t &ball){
    bool is_intersect = false;
    /*
    //產生該boundary對應的poly 檢查covered_by
    polygon_t temp_poly;
    for(auto point:CB.boundarySegment){
	bg::append(temp_poly,point);
    }
    for(auto it = adjacent_boundary.boundarySegment.rbegin(); it != adjacent_boundary.boundarySegment.rend(); it++){
        bg::append(temp_poly, *it);
    }
    bg::unique(temp_poly);
    bg::correct(temp_poly); 
    bool ball_isCoveredby = false;
    */
    //判斷要檢查哪一條net的ball
    for(auto &net:nets){
        if((net.boundary0ID == CB.boundaryID) && (net.boundary1ID == adjacent_boundary.boundaryID) ||
        (net.boundary1ID == CB.boundaryID) && (net.boundary0ID == adjacent_boundary.boundaryID)){
	    ball = net.ball;
        }
    }
    //判斷是否與其他邊界線相交和相交的點座標
    if(bg::intersects(temp_line2, adjacent_boundary.boundarySegment)){
        bg::intersection(temp_line2, adjacent_boundary.boundarySegment, intersectionPoints);
        is_intersect = true;
    }
    return is_intersect;
}
 //從bendingPoint轉角45度
void rotate_45degree(line_t &line, line_t &last_line, long double &x_bias, long double &y_bias){
    point_t bendingPoint = last_line.front();
    point_t lastPoint = last_line.back();
    std::cout<<std::setprecision(10);
    if(x_bias != 0.0){
        bg::append(line, point_t(bendingPoint.x(), bendingPoint.y()));
        bg::append(line, point_t(lastPoint.x(), x_bias + lastPoint.y()));
    }else if(y_bias != 0.0){
        bg::append(line, point_t(bendingPoint.x(), bendingPoint.y()));
        bg::append(line, point_t( y_bias+lastPoint.x(),lastPoint.y()));
    }
}
line_t binary_search_trapezoid(const double &target_area, const line_t &last_line, const double &x_offset, const double &y_offset, bool isHorizontal){
    double sign_of_x_offset, sign_of_y_offset;
    double EPS = 1e-05;
    double low = 0.1, high = 0.0;
    double area = 0.0;
    line_t shifted_last_line;
    x_offset < 0 ? sign_of_x_offset = -1 : sign_of_x_offset = 1;
    y_offset < 0 ? sign_of_y_offset = -1 : sign_of_y_offset = 1;
    isHorizontal ? high = abs(y_offset) : high = abs(x_offset);
    while(high - low > EPS) {
        double mid = (low + high) / 2.0;
	std::cout<<std::setprecision(10);
	//std::cout<<"mid: "<<mid<<", sign_of_x_offset: "<<sign_of_x_offset<<", sign_of_y_offset: "<<sign_of_y_offset<<std::endl;
	
	shifted_last_line.clear();
        polygon_t poly;
        if(isHorizontal){
            point_t A(last_line.back().x(), last_line.back().y() + (mid * sign_of_y_offset));
	    point_t B(last_line.front().x() + (mid * sign_of_x_offset), last_line.front().y() + (mid * sign_of_y_offset));
	    bg::append(poly, A);
	    bg::append(poly, B);
	    bg::append(poly, last_line.front());
	    bg::append(poly, last_line.back());
	    bg::append(poly,A);

	    bg::unique(poly);
	    bg::correct(poly);
	    bg::append(shifted_last_line, B);
	    bg::append(shifted_last_line, A);
        }
	else if(!isHorizontal){
	    point_t A(last_line.back().x() + (mid * sign_of_x_offset), last_line.back().y());
	    point_t B(last_line.front().x() + (mid * sign_of_x_offset), last_line.front().y() + (mid * sign_of_y_offset));
	    bg::append(poly, A);
	    bg::append(poly, B);
	    bg::append(poly, last_line.front());
	    bg::append(poly, last_line.back());
	    bg::append(poly,A);

	    bg::unique(poly);
	    bg::correct(poly);
	    bg::append(shifted_last_line, B);
	    bg::append(shifted_last_line, A);
        }
        std::string reason;
    	if (!bg::is_valid(poly, reason)) {
            std::cout << "stage_1-1 Polygon is invalid: " << reason << std::endl;
	}
       area = bg::area(poly) * 1e-08;
       //std::cout<<"area: "<<area<<std::endl;
       /*
       if(std::fabs(area - target_area) / target_area < 1e-05)
	   break;
       */
       if(area > target_area)
           high = mid;
       else 
           low = mid;
       //std::cout<<bg::wkt(poly)<<std::endl;
   }
   //std::cout<<std::setprecision(10)<<"target_area: "<<target_area<<" ,area: "<<area<<std::endl;
   return shifted_last_line;
}

void Phase3(std::vector<commonBoundary> &commonBoundaries, std::vector<netInfo> &nets, std::vector<double> &deltaVector, std::vector<int> &bVector){
    
    int rows = commonBoundaries.size();
    int count = 0; 
    auto modIdx = [&] (int idx){
        return (idx+rows) % rows;
    };

    double pitch = 1e+06;

    std::vector<commonBoundary> ordering_CB = commonBoundaries;
    std::vector<std::pair<int, point_t>> illegal_boundaryID;
    //preprocessing: check every net legalization
    //以net為單位檢查ball是否在自己的poly當中
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
        else if(nets[i].outterBoundarySegments.size() == 1){
            for(auto &point:nets[i].outterBoundarySegments[0]){
                bg::append(poly,point);
            }
        }
        //boundary 1
        for(auto it = nets[i].boundarySegments[1].rbegin(); it != nets[i].boundarySegments[1].rend(); it++)
            bg::append(poly,*it);
        //inner boundary
        if(nets[i].innerBoundarySegments.size() == 2){
            for(auto it = nets[i].innerBoundarySegments[1].rbegin(); it != nets[i].innerBoundarySegments[1].rend(); it++)
                bg::append(poly,*it);
            for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                bg::append(poly,*it);
        }
        else if (nets[i].innerBoundarySegments.size() == 1 ){
            for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                bg::append(poly,*it);
        }
        bg::unique(poly);
        bg::correct(poly);
	point_t ball = nets[i].ball;
	double distance_b0 = bg::distance(nets[i].boundarySegments[0], nets[i].ball);
	double distance_b1 = bg::distance(nets[i].boundarySegments[1], nets[i].ball);
	std::cout<<"NetID: "<<nets[i].netID<<", distance_bo: "<<distance_b0<<", distance_b1: "<<distance_b1<<std::endl;
	//對於沒有covered_by的邊界線進行處理
	if(!bg::covered_by(ball, poly)){
	    std::cout<<"!covered_by NetID: "<<nets[i].netID<<", distance bo: "<<distance_b0<<", distance_b1: "<<distance_b1<<std::endl;

	    distance_b0 < distance_b1 ? illegal_boundaryID.emplace_back(nets[i].boundary0ID, ball) :
		    illegal_boundaryID.emplace_back(nets[i].boundary1ID, ball);
	}
       //對於有covered_by但是within的邊界線進行平移(ball 剛好壓在邊界線上), 經Phase2邊界線跟ball距離不夠者尚未考慮
       if(bg::covered_by(ball, poly) && !bg::within(ball, poly) ){
           int index = -1;
	   double offset = 0.5 * pitch;
	   bool isHorizontal = false;
	   line_t original_CB;
	   polygon_t temp_poly;
	   if(distance_b0 < distance_b1){
	       index = nets[i].boundary0ID;
	   }
	   else {
	       index = nets[i].boundary1ID;
	   }
	   deltaVector[index] < 0 ? offset = offset : offset = 0 - offset;

           distance_b0 < distance_b1 ? index = nets[i].boundary0ID : index = nets[i].boundary1ID;
	   point_t bendingPoint = commonBoundaries[index].boundarySegment[commonBoundaries[index].boundarySegment.size() - 2];
	   point_t lastPoint = commonBoundaries[index].boundarySegment[commonBoundaries[index].boundarySegment.size() - 1];
	   original_CB = commonBoundaries[index].boundarySegment;
	   //若採用斜線改到後續處理 這邊先pass
	   
	   if(bVector[index] == 1 && commonBoundaries[index].isSlash){
	       std::cout<<"isSlash && bVector == 1 continue!"<<std::endl;
	       continue;
	   }
           if(lastPoint.y() == bendingPoint.y()){
               isHorizontal = true;
           }
	   if(isHorizontal){
	       for(auto &point: commonBoundaries[index].boundarySegment){
	           point.y(point.y() + offset);
	       }
	   }
	   else if(!isHorizontal){
	       for(auto &point: commonBoundaries[index].boundarySegment){
	           point.x(point.x() + offset);
	       }
	   }
	   for(auto point: original_CB){
	       bg::append(temp_poly, point);
	   }
	   for(auto it = commonBoundaries[index].boundarySegment.rbegin(); it != commonBoundaries[index].boundarySegment.rend(); it++){
	       bg::append(temp_poly, *it);
	   }
	   bg::unique(temp_poly);
	   bg::correct(temp_poly);
	   double shifted_area = bg::area(temp_poly) * 1e-08;

	   offset > 0 ? commonBoundaries[index].phase2_deviation += shifted_area : commonBoundaries[index].phase2_deviation -= shifted_area;

	   std::cout<<"within ID: "<<commonBoundaries[index].boundaryID<<", dev: "<<commonBoundaries[index].phase2_deviation<<", dist_b0: "<<distance_b0
		   <<", dist_b1: "<<distance_b1<<std::endl;
           ordering_CB = commonBoundaries;
       } 
    }
    //依照phase2_deviation排序 誤差越大者優先處理
    /*
    std::sort(ordering_CB.begin(),ordering_CB.end(),[](const commonBoundary &a, const commonBoundary &b){
        return a.phase2_deviation > b.phase2_deviation;}
        );
    */
    //重新排序ordering_CB
    for(auto it = commonBoundaries.rbegin(); it != commonBoundaries.rend(); it++){
        int boundary0_index = commonBoundaries[0].boundaryID;
	int temp_index = (*it).boundaryID;
	if((*it).phase2_deviation == 0 || (deltaVector[boundary0_index] * deltaVector[temp_index]) < 0){
	    break;
	}
	count++;
    }
    ordering_CB.insert(ordering_CB.begin(), ordering_CB.end() - count, ordering_CB.end());
    ordering_CB.erase(ordering_CB.end() - count, ordering_CB.end());
    int index = 0;
    for(auto &CB:ordering_CB){
	std::cout<<"\nCB.ID: "<<CB.boundaryID<<", CB.direction: "<<CB.startPointDirection<<std::endl;
        if(CB.phase2_deviation == 0){
            continue;
        }
        index = CB.boundaryID;
        point_t bendingPoint = CB.boundarySegment[CB.boundarySegment.size()-2];
        point_t lastPoint = CB.boundarySegment[CB.boundarySegment.size()-1];
	point_t bendingPoint2 = CB.boundarySegment[CB.boundarySegment.size() - 4];
        commonBoundary adjacent_boundary;
	if(CB.boundary_move_direction == 0){
	adjacent_boundary = commonBoundaries[CB.netA];
	    if(CB.netA == CB.boundaryID ){
	    	adjacent_boundary = commonBoundaries[modIdx(index-1)];
	    }
	}else if(CB.boundary_move_direction == 1){
	    adjacent_boundary = commonBoundaries[CB.netB];
	    if(CB.netB == CB.boundaryID){
	    	adjacent_boundary = commonBoundaries[modIdx(index+1)];
	    }
	}
        bool isHorizontal = false;
        //紀錄該條邊界線是否調整過角度
        bool isUpdated = false;
        double bendingPoint_area = 0.0;
        double range_left = 0.0, range_right = 0.0;
	double shifted_min = 0.0, shifted_max = 0.0, EPS = 1e-04;
        double left = 0.0, right = 0.0;
	bool isProjected;
	point_t projectedPoint;
	point_t shifted_bendingPoint;	    
	point_t net_ball;
	point_t C, D;
        //轉角度後的最後一條線段與原邊界線的最後一條線段
        line_t temp_line, last_line, last_line2, rotate_45degree_line;
	//last_line
        bg::append(last_line, point_t(bendingPoint.x(), bendingPoint.y()));
        bg::append(last_line, point_t(lastPoint.x(), lastPoint.y()));
	//last_line2
	bg::append(last_line2, bendingPoint);
	bg::append(last_line2, bendingPoint2);
        if(lastPoint.y() == bendingPoint.y()){
            isHorizontal = true;
        }
	auto check_vertical_projection = [](const point_t &A, const line_t &last_line2) -> std::pair<point_t, bool>{
	    //last_line2 不是一條折線
	    if(last_line2.size() != 2) return {point_t(), false};
	    std::vector<point_t> out;
	    line_t ray_up{A, point_t(A.x(), A.y()+1e8)};
	    line_t ray_down{A, point_t(A.x(), A.y()-1e8)};

	    bg::intersection(ray_up, last_line2, out);
	    if(!out.empty()) return {out[0], true};

	    out.clear();
	    bg::intersection(ray_down, last_line2, out);
	    if(!out.empty()) return {out[0], true};

	    return {point_t(), false};
	};
	auto check_horizontal_projection = [](const point_t &A, const line_t &last_line2) -> std::pair<point_t, bool>{
	    if(last_line2.size() != 2) return {point_t(), false};
	    std::vector<point_t> out;
	    line_t ray_right{A, point_t(A.x() + 1e8, A.y())};
	    line_t ray_left{A, point_t(A.x() - 1e8, A.y())};

	    bg::intersection(ray_right, last_line2, out);
	    if(!out.empty()) return {out[0],true};
	    
	    out.clear();
	    bg::intersection(ray_left, last_line2, out);
	    if(!out.empty()) return {out[0],true};

	    return {point_t(), false};
	};
	/*
	auto make_poly = [](const netInfo &net) -> polygon_t{
	    
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
            else if(nets[i].outterBoundarySegments.size() == 1){
                for(auto &point:nets[i].outterBoundarySegments[0]){
                    bg::append(poly,point);
                }
            }
            //boundary 1
            for(auto it = nets[i].boundarySegments[1].rbegin(); it != nets[i].boundarySegments[1].rend(); it++)
                bg::append(poly,*it);
            //inner boundary
            if(nets[i].innerBoundarySegments.size() == 2){
                for(auto it = nets[i].innerBoundarySegments[1].rbegin(); it != nets[i].innerBoundarySegments[1].rend(); it++)
                    bg::append(poly,*it);
                for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                    bg::append(poly,*it);
            }
            else if (nets[i].innerBoundarySegments.size() == 1 ){
                for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                    bg::append(poly,*it);
            }
            bg::unique(poly);
            bg::correct(poly);
	
	};
	*/
	for(int i = 0;i < nets.size(); i++){
	    if((CB.boundaryID == nets[i].boundary0ID && adjacent_boundary.boundaryID == nets[i].boundary1ID) || 
			    (adjacent_boundary.boundaryID == nets[i].boundary0ID && CB.boundaryID == nets[i].boundary1ID)){
	        net_ball = nets[i].ball;
	    }
	}
	std::cout<<"before adjusted & ball dist: "<<bg::distance(net_ball,CB.boundarySegment)<<std::endl;
	// preprocessing illegal boundary
	for(auto i: illegal_boundaryID){
	    if(CB.boundaryID == i.first){
		std::cout<<"illegal boundaryID!!!!: "<<CB.boundaryID<<std::endl;
		point_t ball = i.second;
		point_t new_bendingPoint;
		point_t new_lastPoint = lastPoint;
		point_t projectedPoint;
		point_t new_bendingPoint2;
		point_t new_lastPoint2;
		line_t original_last_line = last_line;
		bool isProjected ;
		double sign_of_x_offset;
		double sign_of_y_offset;
		double offset;
		bendingPoint2.x() - bendingPoint.x() > 0 ? sign_of_x_offset = 1 : sign_of_x_offset = -1;
		bendingPoint2.y() - bendingPoint.y() > 0 ? sign_of_y_offset = 1 : sign_of_y_offset = -1;
		//在要調整的邊界線上找ball的映射點
		if(isHorizontal){
		    auto result = check_vertical_projection(ball, last_line);
                    projectedPoint = result.first;
		    isProjected = result.second;
		}
		else if(!isHorizontal){	
		    auto result = check_horizontal_projection(ball, last_line);
                    projectedPoint = result.first;
		    isProjected = result.second;
		}
                //double distance = bg::distance(projectedPoint, ball) + (0.5 * pitch);
                double distance = bg::distance(projectedPoint, ball) + (2.0 * pitch);
		if(projectedPoint.x() == ball.x()){
		    projectedPoint.y() < ball.y() ? distance = distance : distance = 0 - distance;
		    //new_bendingPoint = point_t(projectedPoint.x() + (0.5 * pitch * sign_of_x_offset), projectedPoint.y() + distance);
		    new_bendingPoint = point_t(projectedPoint.x() + (2.0 * pitch * sign_of_x_offset), projectedPoint.y() + distance);
		    auto result = check_vertical_projection(new_bendingPoint, last_line2);
		    new_bendingPoint2 = result.first;
		    new_lastPoint.y(new_lastPoint.y() + distance);
		}else if(projectedPoint.y() == ball.y()){
		    projectedPoint.x() < ball.x() ? distance = distance : distance = 0 - distance;
		    //new_bendingPoint = point_t(projectedPoint.x() + distance, projectedPoint.y() + (0.5 * pitch * sign_of_y_offset));
		    new_bendingPoint = point_t(projectedPoint.x() + distance, projectedPoint.y() + (2.0 * pitch * sign_of_y_offset));
		    auto result = check_horizontal_projection(new_bendingPoint, last_line2);
		    new_bendingPoint2 = result.first;
		    new_lastPoint.x(new_lastPoint.x() + distance);
		}
		//計算調整後的面積
		polygon_t temp_poly;
		bg::append(temp_poly, new_bendingPoint2);
		bg::append(temp_poly, new_bendingPoint);
		bg::append(temp_poly, new_lastPoint);
		bg::append(temp_poly, last_line.back());
		bg::append(temp_poly, last_line.front());
		bg::append(temp_poly, new_bendingPoint2);
                  
   	 	std::cout<<"adjusted illegal polyWKT: "<<bg::wkt(temp_poly)<<std::endl;
		bg::unique(temp_poly);
		bg::correct(temp_poly);

		std::string reason;
    		if (!bg::is_valid(temp_poly, reason)) {
        	    std::cout << "Updated illegal boundary Polygon is invalid: " << reason << std::endl;
    		}
		double area = bg::area(temp_poly) * 1e-08;
		isHorizontal ? offset = bendingPoint2.y() - bendingPoint.y() : offset = bendingPoint2.x() - bendingPoint.x();
		bool isSamesign = (offset * deltaVector[index]) >= 0;
		isSamesign ? CB.phase2_deviation += area : CB.phase2_deviation -= area;
		//更新調整後的邊界後
		CB.boundarySegment.pop_back();
		CB.boundarySegment.pop_back();
		CB.boundarySegment.pop_back();
		CB.boundarySegment.emplace_back(new_bendingPoint2);

		CB.boundarySegment.emplace_back(new_bendingPoint2);
		CB.boundarySegment.emplace_back(new_bendingPoint);

		CB.boundarySegment.emplace_back(new_bendingPoint);
		CB.boundarySegment.emplace_back(new_lastPoint);
		lastPoint = new_lastPoint;
		bendingPoint = new_bendingPoint;
		last_line.clear();
		bg::append(last_line, bendingPoint);
		bg::append(last_line,lastPoint);
		last_line2.clear();
		bg::append(last_line2, new_bendingPoint2);
		bg::append(last_line2, new_bendingPoint);
	    }
	}
        //若是corner net是斜線則調整角度另外處理 找新的轉彎點 
        if(bVector[index] == 1 && CB.isSlash){
	    auto projectPoint = [] (const point_t &A, const point_t &P1, const point_t &P2) -> point_t{
	        double vx = P2.x() - P1.x();
		double vy = P2.y() - P1.y();
		double apx = A.x() - P1.x();
		double apy = A.y() - P1.y();
		double dot = apx * vx + apy * vy;
		double len_sq = vx * vx + vy * vy;
		double t = dot / len_sq;

		return point_t(P1.x() + t * vx, P1.y() + t * vy);
	    };
	    point_t checked_ball1 = nets[index].ball;
	    point_t checked_ball2 = nets[modIdx(index-1)].ball;
	    point_t temp_ball;
	    point_t target_pad;
	    line_t temp_boundarySegment = CB.boundarySegment;
	    line_t temp2_boundarySegment;
	    int net_index = -1;
	    double ball1_distance = bg::distance(checked_ball1, CB.boundarySegment);
	    double ball2_distance = bg::distance(checked_ball2, CB.boundarySegment);
	    double ball_between_pitch_offset;
	    double shifted_slash_area; 
	    double rev_sign_of_startPoint_x, rev_sign_of_startPoint_y;
            bool needed_shift = false;

	    CB.boundarySegment.front().x() > 0 ? rev_sign_of_startPoint_x = -1 : rev_sign_of_startPoint_x = 1;
	    CB.boundarySegment.front().y() > 0 ? rev_sign_of_startPoint_y = -1 : rev_sign_of_startPoint_y = 1;

	    CB.boundary_move_direction == 0 ? target_pad = nets[CB.netA].pad : target_pad = nets[CB.netB].pad;
	    std::cout<<"move_direction: "<<CB.boundary_move_direction<<std::endl;
	    std::cout<<"CB & ball1 dist: "<<bg::distance(checked_ball1, CB.boundarySegment)<<std::endl;
	    std::cout<<"CB & ball2 dist: "<<bg::distance(checked_ball2, CB.boundarySegment)<<std::endl;
	    //找到有distance問題的net後, 平移當前邊界線使其距離足夠0.5 Pitch
	    if(ball1_distance < (0.5 * pitch)){
	        net_index = nets[index].netID;
		temp_ball = nets[index].ball;
		//ball_between_pitch_offset = (0.5 * pitch) - ball1_distance ;
		ball_between_pitch_offset = (((0.5 * pitch) - ball1_distance) * std::sqrt(2));
		needed_shift = true;
	    }
	    else if(ball2_distance < (0.5 * pitch)){
	        net_index = nets[modIdx(index-1)].netID;
		temp_ball = nets[modIdx(index-1)].ball;
		//ball_between_pitch_offset = (0.5 * pitch) - ball2_distance;
		ball_between_pitch_offset = (((0.5 * pitch) - ball2_distance) * std::sqrt(2));
		needed_shift = true;
	    }
	    std::cout<<"ball_between_pitch_offset: "<<ball_between_pitch_offset<<", initial_phase2_dev: "<<CB.phase2_deviation<<std::endl;
	    netInfo temp_net = nets[net_index];
	    //找到目標net的另一條邊界線
	    temp_net.boundary0ID == CB.boundaryID ? temp2_boundarySegment = temp_net.boundarySegments[1] : temp2_boundarySegment = temp_net.boundarySegments[0];
	    //若slash正好在cornerPoint上, 平移x軸或Y軸, 並確定是否ball仍在net中
            if(needed_shift){
		polygon_t temp_poly;
	        //嘗試平移x座標 
                for(auto &point: temp_boundarySegment){
		    point.x(point.x() + (rev_sign_of_startPoint_x * ball_between_pitch_offset));
		    //point.x(point.x() + (rev_sign_of_startPoint_x * 1));
		    bg::append(temp_poly, point);
	        }
                //平移後確認 ball 是否within
		for(auto it = temp2_boundarySegment.rbegin(); it != temp2_boundarySegment.rend(); it++){
		    bg::append(temp_poly, *it);    
		}
		bg::unique(temp_poly);
		bg::correct(temp_poly);
		bool ball_within = bg::within(temp_ball, temp_poly);
		if(ball_within){
		    temp_poly.clear();
		    for(auto &point: temp_boundarySegment){
		        bg::append(temp_poly, point);
		    }

		    bg::append(temp_poly, temp_boundarySegment.back());
		    bg::append(temp_poly, CB.boundarySegment.back());
	            for(auto it = CB.boundarySegment.rbegin(); it != CB.boundarySegment.rend(); it++){
		        bg::append(temp_poly, *it);
		    }

		    bg::append(temp_poly, CB.boundarySegment.front());
		    bg::append(temp_poly, temp_boundarySegment.front());
		    bg::unique(temp_poly);
		    bg::correct(temp_poly);
		    
   	 	    std::cout<<"shifted_polyWKT: "<<bg::wkt(temp_poly)<<std::endl;
    		    std::string reason;
    		    if (!bg::is_valid(temp_poly, reason)) {
        	        std::cout << "shifted_slash Polygon is invalid: " << reason << std::endl;
    		    }
		    shifted_slash_area = bg::area(temp_poly) * 1e-08;

		    //shifted_slash_area = shifted_slash_area * ball_between_pitch_offset;

		    bg::distance(temp_boundarySegment, target_pad) > bg::distance(CB.boundarySegment, target_pad) ?
			CB.phase2_deviation = CB.phase2_deviation + shifted_slash_area  : CB.phase2_deviation = CB.phase2_deviation - shifted_slash_area ;
		    std::cout<<"shifted_slash_area: "<<shifted_slash_area<<", Updated dev: "<<CB.phase2_deviation<<std::endl;
		    CB.boundarySegment = temp_boundarySegment;
		    lastPoint = CB.boundarySegment.back();
		}
		else {
	       //嘗試平移y座標
	       temp_poly.clear();
	       temp_boundarySegment.clear();
	       temp_boundarySegment = CB.boundarySegment;
	       for(auto &point: temp_boundarySegment){
	           point.y(point.y() + (rev_sign_of_startPoint_y * ball_between_pitch_offset ));
	           //point.y(point.y() + (rev_sign_of_startPoint_y * 1));
		   bg::append(temp_poly, point);
	       }
	       for(auto it = temp2_boundarySegment.rbegin(); it != temp2_boundarySegment.rend(); it++){
	           bg::append(temp_poly, *it);
	       }
	       bg::unique(temp_poly);
	       bg::correct(temp_poly);
	       ball_within = bg::within(temp_ball, temp_poly);
	       if(ball_within){
	           temp_poly.clear();
		    for(auto &point: temp_boundarySegment){
                        //point.y(point.y() + (rev_sign_of_startPoint_y * (ball_between_pitch_offset -1)));
		        bg::append(temp_poly, point);
		    }

		    bg::append(temp_poly, temp_boundarySegment.back());
		    bg::append(temp_poly, CB.boundarySegment.back());

	            for(auto it = CB.boundarySegment.rbegin(); it != CB.boundarySegment.rend(); it++){
		        bg::append(temp_poly, *it);
		    }

		    bg::append(temp_poly, CB.boundarySegment.front());
		    bg::append(temp_poly, temp_boundarySegment.front());
		    
		    bg::unique(temp_poly);
		    bg::correct(temp_poly);
   	 	    std::cout<<"shifted_polyWKT: "<<bg::wkt(temp_poly)<<std::endl;
    		    std::string reason;
    		    if (!bg::is_valid(temp_poly, reason)) {
        	        std::cout << "shifted_slash Polygon is invalid: " << reason << std::endl;
    		    }
		    shifted_slash_area = bg::area(temp_poly) * 1e-08;

		    //shifted_slash_area = shifted_slash_area * ball_between_pitch_offset;

		    bg::distance(temp_boundarySegment, target_pad) > bg::distance(CB.boundarySegment, target_pad) ?
			CB.phase2_deviation = CB.phase2_deviation + shifted_slash_area  : CB.phase2_deviation = CB.phase2_deviation - shifted_slash_area ;
		    std::cout<<"shifted_slash_area: "<<shifted_slash_area<<", Updated dev: "<<CB.phase2_deviation<<std::endl;
		    CB.boundarySegment = temp_boundarySegment;
		    lastPoint = CB.boundarySegment.back();
	       }
		} // end of else
	        //std::cout<<"shifted_slash_distance: "<<bg::distance(CB.boundarySegment, temp_ball)<<", shifted_slash_area: "<<shifted_slash_area<<std::endl;
	    }// end of neededshift
	    std::cout<<"adjacent boundaryID: "<<adjacent_boundary.boundaryID<<std::endl;

            //替slash找轉彎點
	    point_t adjacent_boundary_bendingPoint, adjacent_boundary_lastPoint;
	    point_t projectedPoint;
	    ball1_distance < ball2_distance ? temp_ball = checked_ball1 : temp_ball = checked_ball2;
	    //斜線的起點
	    point_t slash_front = CB.boundarySegment[0];
	    //斜線的終點
	    point_t slash_back = CB.boundarySegment[1];
	    point_t temp_lastPoint;
            std::cout<<"slash front: "<<slash_front.x()<<" "<<slash_front.y()<<"slash_back: "<<slash_back.x()<<" "<<slash_back.y()<<std::endl;
	    adjacent_boundary_bendingPoint = adjacent_boundary.boundarySegment[adjacent_boundary.boundarySegment.size() - 2];
	    adjacent_boundary_lastPoint = adjacent_boundary.boundarySegment[adjacent_boundary.boundarySegment.size() - 1];
	    
	    //bendingPoint = projectPoint(adjacent_boundary_bendingPoint, slash_front, slash_back);
            if(adjacent_boundary_lastPoint.y() == adjacent_boundary_bendingPoint.y()){
                isHorizontal = true;
            }

            if(isHorizontal){
	        auto result = check_horizontal_projection(temp_ball, CB.boundarySegment);
	        bendingPoint = result.first;
	    }
	    else if(!isHorizontal){
	        auto result = check_vertical_projection(temp_ball, CB.boundarySegment);
		bendingPoint = result.first;
	    }
	    
	    temp_lastPoint = bendingPoint;
	    isHorizontal ? temp_lastPoint.x(adjacent_boundary_lastPoint.x()) : temp_lastPoint.y(adjacent_boundary_lastPoint.y());
            //last_line
	    last_line.clear();
            bg::append(last_line, bendingPoint);
	    bg::append(last_line, temp_lastPoint);
            //temp_line
	    temp_line.clear();
	    bg::append(temp_line, bendingPoint);
	    bg::append(temp_line, lastPoint);

	    polygon_t temp_poly;
	    bg::append(temp_poly, temp_lastPoint);
	    bg::append(temp_poly, lastPoint);
	    bg::append(temp_poly, bendingPoint);
	    bg::correct(temp_poly);
	    double temp_area = bg::area(temp_poly) * 1e-08;
	    if( temp_area > CB.phase2_deviation ){
	        std::cout<<"new_temp_area can satisfy!\n";
	        lastPoint = temp_lastPoint;
		CB.boundarySegment.pop_back();
		bg::append(CB.boundarySegment, bendingPoint);
		bg::append(CB.boundarySegment, bendingPoint);
		bg::append(CB.boundarySegment, lastPoint);

	        CB.phase2_deviation -= temp_area;
		std::cout<<"temp_area: "<<temp_area<<", CB.dev: "<<CB.phase2_deviation<<std::endl;
	    }
	    for(auto point: temp_line){
	        std::cout<<"temp_line point: "<<point.x()<<", "<<point.y()<<std::endl;
	    }
	    for(auto point: last_line){
	        std::cout<<"last_line point: "<<point.x()<<", "<<point.y()<<std::endl;
	    }

            for(auto point: CB.boundarySegment){
	        std::cout<<"point: "<<point.x()<<", "<<point.y()<<std::endl;
	    }
            

            /*
	    if(isHorizontal){
	        temp_lastPoint.x(adjacent_boundary_lastPoint.x());
	    }else if(!isHorizontal){
	        temp_lastPoint.y(adjacent_boundary_lastPoint.y());
	    }
	    bg::append(temp_line, bendingPoint);
	    bg::append(temp_line, temp_lastPoint);
  	    last_line.clear();
	    bg::append(last_line, bendingPoint);
            bg::append(last_line, lastPoint);
	    */
	    for(auto point:temp_line){
	        std::cout<<"temp_line: "<<point.x()<<" "<<point.y()<<std::endl;
	    }
	    std::cout<<"bendingPoint: "<<bendingPoint.x()<<" "<<bendingPoint.y()<<", lastPoint: "<<lastPoint.x()<<" "<<lastPoint.y()<<", ball: "<<net_ball.x()<<" "<<net_ball.y()<<", isHorizontal: "<<isHorizontal<<std::endl;
	    //continue;
        }

	//是否投影成功到last_line2上
	//stage 0 / stage 1
        //邊界線的最後一段是水平線
	else if((isHorizontal)){
	    auto result = check_horizontal_projection(net_ball, last_line2);
	    projectedPoint = result.first;
	    isProjected = result.second;
	    if(!isProjected){
	        line_t shifted_last_line;
	        double offset = bendingPoint.y() - bendingPoint2.y();
	        bool same_sign = (offset * deltaVector[index]) >= 0;
		if(!same_sign){
	            bg::append(shifted_last_line, bendingPoint2);
	            bg::append(shifted_last_line, point_t(lastPoint.x(), lastPoint.y() - offset)); 
		    
		    polygon_t temp_poly;
		    bg::append(temp_poly, lastPoint);
		    bg::append(temp_poly, bendingPoint);
		    bg::append(temp_poly, bendingPoint2);
		    bg::append(temp_poly, shifted_last_line.back());
		    bg::correct(temp_poly);
    		    std::string reason;
    		    if (!bg::is_valid(temp_poly, reason)) {
        	        std::cout << "stage_0 Polygon is invalid: " << reason << std::endl;
    		    }
		    double stage_0_max_area = bg::area(temp_poly) * 1e-08;
		    std::cout<<bg::wkt(temp_poly)<<std::endl;
		    std::cout<<"stage_0_max_area: "<<stage_0_max_area<<std::endl;
		    if(CB.phase2_deviation <= stage_0_max_area){
			double x_offset = bendingPoint2.x() - bendingPoint.x();
			double y_offset = bendingPoint2.y() - bendingPoint.y();
			shifted_last_line = binary_search_trapezoid(CB.phase2_deviation, last_line, x_offset, y_offset, isHorizontal);
			temp_poly.clear();
		        bg::append(temp_poly, shifted_last_line.back());
		        bg::append(temp_poly, shifted_last_line.front());
		        bg::append(temp_poly, last_line.front());
		        bg::append(temp_poly, last_line.back());
		        bg::append(temp_poly, shifted_last_line.back());
		        
			bg::unique(temp_poly);
		        bg::correct(temp_poly);
		        double stage_0_Area = bg::area(temp_poly) * 1e-08;
			if(std::fabs(stage_0_Area - CB.phase2_deviation) < EPS){
			    CB.boundarySegment.pop_back();
			    CB.boundarySegment.pop_back();
			    CB.boundarySegment.pop_back();
			    CB.boundarySegment.emplace_back(shifted_last_line.front());
			    CB.boundarySegment.emplace_back(shifted_last_line.front());
			    CB.boundarySegment.emplace_back(shifted_last_line.back());
		            std::cout<<"stage_0_Area: "<<stage_0_Area<<std::endl;
			    std::cout<<"Boundary ID: "<<CB.boundaryID<<"at stage_0 solved!\n";
			    std::cout<<"& ball dist: "<<bg::distance(net_ball,CB.boundarySegment)<<std::endl; 
			    continue;
			}
		    }
		} //end of !same_sign
	    }

            long double x_bias = abs(lastPoint.x()) - abs(bendingPoint.x());
            long double y_bias = 0.0;
	    deltaVector[index] < 0 ? x_bias = 0 - x_bias : x_bias = x_bias;
	    rotate_45degree(temp_line,last_line, x_bias, y_bias);
        }
        //邊界線的最後一段是垂直線
        else if((!isHorizontal)){
	    auto result = check_vertical_projection(net_ball, last_line2);
	    projectedPoint = result.first;
	    isProjected = result.second;
            if(!isProjected){
	        line_t shifted_last_line;
		double offset = bendingPoint.x() - bendingPoint2.x();
	        bool same_sign = (offset * deltaVector[index]) >= 0;
		if(!same_sign){
		    bg::append(shifted_last_line, bendingPoint2);
		    bg::append(shifted_last_line, point_t(lastPoint.x() - offset, lastPoint.y()));
		    
		    polygon_t temp_poly;
		    bg::append(temp_poly, lastPoint);
		    bg::append(temp_poly, bendingPoint);
		    bg::append(temp_poly, bendingPoint2);
		    bg::append(temp_poly, shifted_last_line.back());
		    bg::correct(temp_poly);
    		    std::string reason;
    		    if (!bg::is_valid(temp_poly, reason)) {
        	        std::cout << "stage 1-1 Polygon is invalid: " << reason << std::endl;
    		    }
		    double stage_0_max_area = bg::area(temp_poly) * 1e-08;
		    std::cout<<bg::wkt(temp_poly)<<std::endl;
		    std::cout<<"stage_0_max_area: "<<stage_0_max_area<<std::endl;
		    if(CB.phase2_deviation <= stage_0_max_area){
			double x_offset = bendingPoint2.x() - bendingPoint.x();
			double y_offset = bendingPoint2.y() - bendingPoint.y();
			double sign_of_x_offset, sign_of_y_offset;
                        x_offset < 0 ? sign_of_x_offset = -1 : sign_of_x_offset = 1;
			y_offset < 0 ? sign_of_y_offset = -1 : sign_of_y_offset = 1;
			shifted_last_line = binary_search_trapezoid(CB.phase2_deviation, last_line, x_offset, y_offset, isHorizontal);
			temp_poly.clear();
		        bg::append(temp_poly, shifted_last_line.back());
		        bg::append(temp_poly, shifted_last_line.front());
		        bg::append(temp_poly, last_line.front());
		        bg::append(temp_poly, last_line.back());
		        bg::append(temp_poly, shifted_last_line.back());
		        
			bg::unique(temp_poly);
		        bg::correct(temp_poly);
		        double stage_0_Area = bg::area(temp_poly) * 1e-08;
			if(std::fabs(stage_0_Area - CB.phase2_deviation) < EPS){
			    CB.boundarySegment.pop_back();
			    CB.boundarySegment.pop_back();
			    CB.boundarySegment.pop_back();
			    CB.boundarySegment.emplace_back(shifted_last_line.front());
			    CB.boundarySegment.emplace_back(shifted_last_line.front());
			    CB.boundarySegment.emplace_back(shifted_last_line.back());
		            std::cout<<"stage_0_Area: "<<stage_0_Area<<std::endl;
			    std::cout<<"Boundary ID: "<<CB.boundaryID<<"at stage_0 solved!\n";
			    std::cout<<"& ball dist: "<<bg::distance(net_ball,CB.boundarySegment)<<std::endl; 
			    continue;
			}
		    }
		} // end of !same_sign
	    }
            long double y_bias = abs(lastPoint.y()) - abs(bendingPoint.y());
            long double x_bias = 0.0;
            deltaVector[index] < 0 ? y_bias = 0 - y_bias : y_bias = y_bias;
            rotate_45degree(temp_line,last_line, x_bias, y_bias);
	     
        }
	std::cout<<"netID: "<<CB.boundaryID<<", isProjected: "<<isProjected<<", projectdePoint: "<<projectedPoint.x()<<" "<<projectedPoint.y()<<", bendingPoint: "
		<<bendingPoint.x()<<" "<<bendingPoint.y()<<", ball: "<<net_ball.x()<<" "<<net_ball.y()<<", delta: "<<deltaVector[index]<<std::endl;

        //試算從bendingPoint轉45度後的面積
        bendingPoint_area = cal_bendingPoint_area(temp_line,last_line,CB.cornerLine);

	double stage_1_max_area = bendingPoint_area;
	std::cout<<"45 degree: \n"<<"Boundary ID: "<<CB.boundaryID<<", deltaVector: "<<deltaVector[index]<<", Initial deviation: "<<abs(CB.phase2_deviation)<<", After stage_1 area: "<< abs(CB.phase2_deviation) - bendingPoint_area<<std::endl;

        if(abs(CB.phase2_deviation) <= bendingPoint_area){
            //找精確座標值
	    auto calculateTargetLength = [](double area) -> double{
	        return std::sqrt(2 * area);
	    };
	    double target_length = calculateTargetLength(abs(CB.phase2_deviation) * 1e+08);
	    double stage_1_Area = 0.0;
	    std::cout<<"stage_1_target_length: "<<target_length<<std::endl;

	    line_t target_45degree_line, new_last_line;
	    point_t new_bendingPoint = bendingPoint;
	    point_t new_lastPoint = lastPoint;
	    double x_bias = 0.0, y_bias = 0.0;

	    if(bVector[index] == 1 && CB.isSlash){
		if(CB.phase2_deviation < 0){
			std::cout<<"CB.dev = neg!\n";
		    if(isHorizontal){
			x_bias = abs(target_length);
		        adjacent_boundary.startPointDirection == 1 ? new_bendingPoint.x(lastPoint.x() - x_bias) : 
			    new_bendingPoint.x(lastPoint.x() + x_bias);
	    	        deltaVector[adjacent_boundary.boundaryID] < 0 ? x_bias = 0 - x_bias : x_bias = x_bias;
		    }
		    else if(!isHorizontal){
			y_bias = abs(target_length);
		        adjacent_boundary.startPointDirection == 0 ? new_bendingPoint.y(lastPoint.y() - y_bias) :
			    new_bendingPoint.y(lastPoint.y() + y_bias);
                        deltaVector[adjacent_boundary.boundaryID] < 0 ? y_bias = 0 - y_bias : y_bias = y_bias;
		    }

		    CB.phase2_deviation = abs(CB.phase2_deviation);
		}
		//lastPoint.x() > 0 ? x_bias = 0 - abs(target_length) : x_bias = abs(target_length);
		//lastPoint.y() > 0 ? y_bias = 0 - abs(target_length) : y_bias = abs(target_length);
	        //new_bendingPoint.y(lastPoint.y() + y_bias);
	        //new_bendingPoint.x(lastPoint.x() + x_bias_hypotenuse);
	        //new_bendingPoint.y(lastPoint.y() + y_bias_hypotenuse);
	        //isHorizontal ? new_lastPoint.y(lastPoint.y() + y_bias) : new_lastPoint.x(lastPoint.x() + x_bias);
	        std::cout<<"new_bendingPoint: "<<new_bendingPoint.x()<<" "<<new_bendingPoint.y()<<", new_lastPoint: "<<new_lastPoint.x()<<" "<<new_lastPoint.y()<<std::endl;	
	    }
	    else if((isHorizontal)){
                x_bias = abs(target_length);
                y_bias = 0.0;
		CB.startPointDirection == 1 ? new_bendingPoint.x(lastPoint.x() - x_bias) : 
			new_bendingPoint.x(lastPoint.x() + x_bias);
		//std::cout<<"lastPoint.x(): "<<lastPoint.x()<<", sum: "<<new_bendingPoint.x()<<std::endl;
	    	deltaVector[index] < 0 ? x_bias = 0 - x_bias : x_bias = x_bias;
            }else if (!isHorizontal){
	    	x_bias = 0.0;
		y_bias = abs(target_length);
		CB.startPointDirection == 0 ? new_bendingPoint.y(lastPoint.y() - y_bias) :
			new_bendingPoint.y(lastPoint.y() + y_bias);
                deltaVector[index] < 0 ? y_bias = 0 - y_bias : y_bias = y_bias;
	    }
            //target_45degree_line.front()
            bg::append(target_45degree_line, new_bendingPoint);

	    //target_45degree_line.back()
	    //轉45度
	    if(x_bias != 0.0){
        	bg::append(target_45degree_line, point_t(lastPoint.x(), x_bias + lastPoint.y()));
    	    }else if(y_bias != 0.0){
        	bg::append(target_45degree_line, point_t(y_bias + lastPoint.x(),lastPoint.y()));
    	    }

	    /*
	    if(bVector[index] == 1 && CB.isSlash){
	        bg::append(target_45degree_line, new_lastPoint);
	    }else if(x_bias != 0.0){
        	bg::append(target_45degree_line, point_t(lastPoint.x(), x_bias + lastPoint.y()));
    	    }else if(y_bias != 0.0){
        	bg::append(target_45degree_line, point_t(y_bias + lastPoint.x(),lastPoint.y()));
    	    }
            */
	    new_last_line = last_line;
	    new_last_line.front() = target_45degree_line.front();
	    //new_last_line.front().x(target_45degree_line.front().x());
	    //new_last_line.front().y(target_45degree_line.front().y());

            stage_1_Area = cal_bendingPoint_area(target_45degree_line, new_last_line, CB.cornerLine);

	    std::cout<<"stage_1_Area: "<<stage_1_Area<<std::endl;
	    CB.boundarySegment.pop_back();
	    CB.boundarySegment.emplace_back(target_45degree_line.front());
  	    for(auto &point : target_45degree_line){
	        CB.boundarySegment.emplace_back(point);
	    }
	    if(std::fabs(stage_1_Area - CB.phase2_deviation) < EPS ){
	    	std::cout<<"Boundary ID: "<<CB.boundaryID<<" at stage_1 solved!\n";
	        std::cout<<"& ball dist: "<<bg::distance(net_ball,CB.boundarySegment)<<std::endl; 
	    }
	    isUpdated = true;
            continue;
        }
	//stage_2
        //從bendingPoint轉90度 或檢查從第二個折線形變的可能(ball在多邊形內部很遠的位置
        else if(CB.phase2_deviation > stage_1_max_area){
            //先檢查轉90度是否會與其他邊界線相交 找出相交的點並設定平移bendingPoint極限值
            line_t temp_line2, shifted_45degree_line, temp_line2_downto_outterRect;
	    line_t t_line, t_45degree_line;
            //commonBoundary adjacent_boundary;
            std::vector<point_t> intersectionPoints;
            point_t intersectionPoint, ball;
            double shifted_bendingPoint_area = 0.0;
	    double shifted_offset = 2.0 * pitch;
            bool is_intersect = false;
	    int adjacent_boundaryID = -1;
            //找出轉角度時可能會發生相交的邊界線
	    if(CB.boundary_move_direction == 0){
	    	adjacent_boundary = commonBoundaries[CB.netA];
		if(CB.netA == CB.boundaryID ){
	    	    adjacent_boundary = commonBoundaries[modIdx(index-1)];
		}
	    }else if(CB.boundary_move_direction == 1){
	    	adjacent_boundary = commonBoundaries[CB.netB];
		
		if(CB.netB == CB.boundaryID){
	    	    adjacent_boundary = commonBoundaries[modIdx(index+1)];
		}
		
	    }
   	    //產生temp_line2_downto_outterRect
            if(isHorizontal){
                bendingPoint.x() - lastPoint.x() > 0 ? shifted_offset = 0 - shifted_offset : shifted_offset = shifted_offset; 
		shifted_bendingPoint = point_t(bendingPoint.x() + shifted_offset, bendingPoint.y());
	        bg::append(temp_line2_downto_outterRect, shifted_bendingPoint);
                if(deltaVector[index] > 0){
                    bg::append(temp_line2_downto_outterRect, point_t(shifted_bendingPoint.x(), abs(lastPoint.x())));
                }else if(deltaVector[index] < 0){
                    bg::append(temp_line2_downto_outterRect, point_t(shifted_bendingPoint.x(), 0 - abs(lastPoint.x())));
                }
            }
            else if(!isHorizontal){
		bendingPoint.y() - lastPoint.y() > 0 ? shifted_offset = 0 - shifted_offset : shifted_offset = shifted_offset;
		shifted_bendingPoint = point_t(bendingPoint.x(), bendingPoint.y() + shifted_offset);
	        bg::append(temp_line2_downto_outterRect, shifted_bendingPoint);
                if(deltaVector[index] > 0){
                    bg::append(temp_line2_downto_outterRect, point_t(abs(lastPoint.y()), shifted_bendingPoint.y()));
                }else if(deltaVector[index] < 0){
                    bg::append(temp_line2_downto_outterRect, point_t(0 - abs(lastPoint.y()), shifted_bendingPoint.y()));
                }
            }
            //判斷是否與其他邊界線/ball相交, 並找出相交的點座標
            //intersect 2種情況: true: 碰到其他邊界線, false:碰到外邊界線
            //根據回傳的交點決定bendingPoint可以平移的範圍
            is_intersect = isIntersect_with_otherBoundary(temp_line2_downto_outterRect, CB, adjacent_boundary, nets, intersectionPoints, ball);
	    CB.is_Intersect = is_intersect;
	    std::cout<<"ID: "<<CB.boundaryID<<" ,is_intersect: "<<is_intersect<<std::endl;
	    //stage_2
            if(is_intersect){
		std::cout<<"stage_2 is intersect  ID:"<<CB.boundaryID<<std::endl;
                intersectionPoint = intersectionPoints[0];
		/*
		for(auto point:intersectionPoints){
			std::cout<<"intersectionPoint: "<<point.x()<<", "<<point.y()<<std::endl;
		}
		*/
		//shifted_bendingPoint = bendingPoint;
                temp_line2.clear();
                bg::append(temp_line2, shifted_bendingPoint);
                bg::append(temp_line2, intersectionPoint);

		bg::append(t_line, shifted_bendingPoint);
		bg::append(t_line, lastPoint);
                //在平移後bendingPoint平移的45度線
                if(isHorizontal){
		   long double x_bias = bg::distance(lastPoint, shifted_bendingPoint);
		   long double y_bias = 0.0;
		   deltaVector[index] < 0 ? x_bias = 0 - x_bias : x_bias = x_bias;
		   rotate_45degree(t_45degree_line, t_line, x_bias, y_bias);
                   
                   double shifted_offset = abs(abs(intersectionPoint.y()) - abs(shifted_bendingPoint.y()));
		   deltaVector[index] > 0 ? shifted_offset = shifted_offset : shifted_offset = 0 - shifted_offset;
                   bg::append(shifted_45degree_line, intersectionPoint);
                   bg::append(shifted_45degree_line, point_t(temp_line.back().x(), temp_line.back().y() + shifted_offset));
                }else if(!isHorizontal){
                    long double x_bias = 0; 
                    long double y_bias = bg::distance(lastPoint, shifted_bendingPoint);
		    deltaVector[index] < 0 ? y_bias = 0 - y_bias : y_bias = y_bias;
		    rotate_45degree(t_45degree_line, t_line, x_bias, y_bias);

                    double shifted_offset = abs(abs(intersectionPoint.x())- abs(shifted_bendingPoint.x()));
                    deltaVector[index] > 0 ? shifted_offset = shifted_offset : shifted_offset = 0 - shifted_offset;
                    bg::append(shifted_45degree_line, intersectionPoint);
                    bg::append(shifted_45degree_line, point_t(temp_line.back().x() + shifted_offset, temp_line.back().y()));
                }
                bg::append(temp_line2, shifted_45degree_line.front());
                bg::append(temp_line2, shifted_45degree_line.back());
		//協助計算stage_2_max_area
                shifted_bendingPoint_area = cal_bendingPoint_area(temp_line2, t_line, adjacent_boundary.cornerLine);
            }
	    //丟到stage_3處理 temp_line2_downto_outterRect沒有碰到其他邊界線而是直接碰到外邊界
	    else {
                //情況3. 碰到外邊界線
                //shifted_bendingPoint_area = cal_bendingPoint_area(temp_line2, temp_line,adjacent_boundary.cornerLine);
            }
	    
	    //確認是否能在stage_2解決
	    double stage_2_max_area;
	    is_intersect ? stage_2_max_area = shifted_bendingPoint_area : stage_2_max_area = -1 ;
	    std::cout<<"stage_2_max_area: "<<stage_2_max_area<<std::endl;
	    if(stage_2_max_area >= CB.phase2_deviation){
		polygon_t t_poly;
	        bg::append(t_poly, t_45degree_line.back());
	        bg::append(t_poly, t_45degree_line.front());
	        bg::append(t_poly, t_line.back());
		bg::append(t_poly, t_45degree_line.back());
	        bg::correct(t_poly);
	        double t_poly_area = bg::area(t_poly) * 1e-08;
		double stage_2_TargetArea = (CB.phase2_deviation - t_poly_area)* 1e+08;
		//double stage_2_TargetArea = (CB.phase2_deviation - bendingPoint_area) * 1e+08;
		double stage_2_Area = 0.0;
		//sin 45 情況1/2
		if(is_intersect){
		    //計算平行四邊形的短邊
		    auto calShortEdge45 = [](double TargetArea, double L){
		        return (TargetArea) / L;
		        //return (std::sqrt(2) * TargetArea) / L;
		    };
		    //長邊
		    double t_line_length = bg::distance(t_line.front(), t_line.back()); 
		    //短邊
		    double shifted_offset = calShortEdge45(stage_2_TargetArea, t_line_length);
		    std::cout<<"TargetArea: "<<stage_2_TargetArea<<", t_line_length: "<< t_line_length<<std::endl;
		std::cout<<"CB.dev: "<<CB.phase2_deviation<<", t_poly_area: "<<t_poly_area<<", shifted_offset: "<<shifted_offset<<std::endl;
		    shifted_45degree_line.clear();
		    //組合平行四邊形
                    if(isHorizontal){
                        deltaVector[index] > 0 ? shifted_offset = shifted_offset : shifted_offset = 0 - shifted_offset;
		        bg::append(shifted_45degree_line, point_t(t_45degree_line.front().x(), t_45degree_line.front().y()));
			bg::append(shifted_45degree_line, point_t(t_45degree_line.front().x(), t_45degree_line.front().y() + shifted_offset));
                        
			bg::append(shifted_45degree_line, point_t(t_45degree_line.front().x(), t_45degree_line.front().y() + shifted_offset));
                        bg::append(shifted_45degree_line, point_t(t_45degree_line.back().x(), t_45degree_line.back().y() + shifted_offset));
                    }else if(!isHorizontal){
                        deltaVector[index] > 0 ? shifted_offset = shifted_offset : shifted_offset = 0 - shifted_offset;
		        bg::append(shifted_45degree_line, point_t(t_45degree_line.front().x(), t_45degree_line.front().y()));
		        bg::append(shifted_45degree_line, point_t(t_45degree_line.front().x() + shifted_offset,t_45degree_line.front().y()));
                        
			bg::append(shifted_45degree_line, point_t(t_45degree_line.front().x() + shifted_offset, t_45degree_line.front().y()));
                        bg::append(shifted_45degree_line, point_t(t_45degree_line.back().x() + shifted_offset, t_45degree_line.back().y()));
                    }
		    //計算平行四邊形面積
                    stage_2_Area = cal_bendingPoint_area(shifted_45degree_line, t_line, adjacent_boundary.cornerLine);
		    std::cout<<"stage_2_Area: " << stage_2_Area<<std::endl;
		    //檢查該面積是否符合
		    if(std::fabs(stage_2_Area - CB.phase2_deviation) < EPS){
		    	std::cout<<"boundary ID: "<<CB.boundaryID<<" at stage_2 solved!\n";
		        CB.boundarySegment.pop_back();
			bg::append(CB.boundarySegment, shifted_bendingPoint);

		        for(auto &point : shifted_45degree_line){
		    	    bg::append(CB.boundarySegment,point_t(point));
		        }
		        isUpdated = true;
	                std::cout<<"& ball dist: "<<bg::distance(net_ball,CB.boundarySegment)<<std::endl; 
			continue;
		    }
		    /*
		    if(stage_2_Area == (stage_2_TargetArea * 1e-08)){
		    	std::cout<<"boundary ID: "<<CB.boundaryID<<" at stage_2 solved!\n";
		    }
		    */
		}// end of is_intersect in stage_2
	    } // end of stage_2
	    //無法在stage_2解決 到stage_3
	    else if(stage_2_max_area < CB.phase2_deviation || !is_intersect){
	        double stage_3_Area = 0.0;
	        double stage_3_max_area = 0.0;
		bool ball_out_of_boundary = false;
		
		line_t new_last_line;
		line_t temp_line3;
	    	point_t new_bendingPoint = shifted_bendingPoint;
		point_t shifted_lastPoint = temp_line2_downto_outterRect.back();
		//替ball out of boundary準備
                point_t t_shifted_lastPoint = temp_line2_downto_outterRect.back();
		line_t t_line2_downto_outterRect = temp_line2_downto_outterRect;
		double bendingPoint_lastPoint_dist = bg::distance(shifted_bendingPoint, lastPoint);

		std::cout<<"stage_3 ID:"<<CB.boundaryID<<std::endl;
		
		auto calShortEdge90 = [](double TargetArea, double L){
		    return TargetArea / L;
		};
		double L = bg::length(temp_line2_downto_outterRect);
		double target_length = calShortEdge90(CB.phase2_deviation * 1e+08, L);
		if(isHorizontal){
	            if(CB.startPointDirection == 1){
			t_shifted_lastPoint.x(temp_line2_downto_outterRect.back().x() + bendingPoint_lastPoint_dist);
		        new_bendingPoint.x(lastPoint.x() - target_length);
			for(auto &point : temp_line2_downto_outterRect){
			    point.x(lastPoint.x() - target_length);
			}
			shifted_lastPoint.x(temp_line2_downto_outterRect.back().x() + target_length);
		    }
		    else{
			t_shifted_lastPoint.x(temp_line2_downto_outterRect.back().x() - bendingPoint_lastPoint_dist);
			new_bendingPoint.x(lastPoint.x() + target_length);
			for(auto &point : temp_line2_downto_outterRect){
			    point.x(lastPoint.x() + target_length);
			}
			shifted_lastPoint.x(temp_line2_downto_outterRect.back().x() - target_length);
		    }
		}
		else if(!isHorizontal){
		    if(CB.startPointDirection == 0){
			t_shifted_lastPoint.y(temp_line2_downto_outterRect.back().y() + bendingPoint_lastPoint_dist);
		        new_bendingPoint.y(lastPoint.y() - target_length);
			for(auto &point : temp_line2_downto_outterRect){
			    point.y(lastPoint.y() - target_length);
			}
			shifted_lastPoint.y(temp_line2_downto_outterRect.back().y() + target_length);
		    }
		    else{
			t_shifted_lastPoint.y(temp_line2_downto_outterRect.back().y() - bendingPoint_lastPoint_dist);
		        new_bendingPoint.y(lastPoint.y() + target_length);
		        for(auto &point : temp_line2_downto_outterRect){
		            point.y(lastPoint.y() + target_length);
		        }
		        shifted_lastPoint.y(temp_line2_downto_outterRect.back().y() - target_length);
		    }
		}
		bg::append(new_last_line, new_bendingPoint);
		bg::append(new_last_line, last_line.back());
		temp_line3 = temp_line2_downto_outterRect;
		bg::append(temp_line3, shifted_lastPoint);
		//calculate stage_3_Area
		polygon_t temp_poly;
		for(auto &point : temp_line3){
		    bg::append(temp_poly.outer(), point);
		}
		for(auto it = new_last_line.rbegin(); it != new_last_line.rend(); it++){
        	    bg::append(temp_poly.outer(),*it);
		}
                bg::unique(temp_poly);
    		bg::correct(temp_poly);
   	 	std::cout<<"polyWKT: "<<bg::wkt(temp_poly)<<std::endl;
    		std::string reason;
    		if (!bg::is_valid(temp_poly, reason)) {
        	    std::cout << "Polygon is invalid: " << reason << std::endl;
    		}
		//若temp_poly中, 有ball則代表ball在邊界外
		if(bg::within(net_ball, temp_poly)){
	            std::cout<<"ball out of boundary ID!!!!!!: "<<CB.boundaryID<<std::endl;
	            ball_out_of_boundary = true;	    
		}
		else {
    		    stage_3_Area = bg::area(temp_poly)*1e-08;
		    CB.boundarySegment.pop_back();
		    CB.boundarySegment.emplace_back(temp_line2_downto_outterRect.front());
		    for(auto &point : temp_line2_downto_outterRect){
		        CB.boundarySegment.emplace_back(point);
		    }
		    //stage_3_Area = cal_bendingPoint_area(temp_line3, new_last_line, adjacent_boundary.cornerLine);
		    std::cout<<"stage_3_Area: "<<stage_3_Area<<std::endl;
	            if(std::fabs(stage_3_Area - (CB.phase2_deviation)) < EPS && (!ball_out_of_boundary)){
	    	        std::cout<<"Boundary ID: "<<CB.boundaryID<<" at stage_3 solved!\n";
	                std::cout<<"& ball dist: "<<bg::distance(net_ball,CB.boundarySegment)<<std::endl; 
		        continue;
	            }
		}
		//CB.Temp_line2_downto_outterRect = temp_line2_downto_outterRect;

	        //bendingPoint平移 + 平移temp_line的組合面積解決不了
		/*
		for(auto &temp_CB:ordering_CB){
		    if(temp_CB.boundaryID == adjacent_boundary.boundaryID){
		        adjacent_boundary = temp_CB;
		    }
		}
		CB.in_Stage3 = true;
		CB.Temp_line2_downto_outterRect = temp_line2_downto_outterRect;
		adjacent_boundaryID = adjacent_boundary.boundaryID;
	        std::cout<<"adjacent_boundaryID: "<<adjacent_boundary.boundaryID<<std::endl;
		std::cout<<"adjacent_boundary.in_Stage3: "<<adjacent_boundary.in_Stage3<<std::endl;
		*/
		//stage_4-> stage_3仍不夠面積 or ball out of boundary, 基於temp_line2再轉45度
		//stage_4
		if(ball_out_of_boundary){
		    std::cout<<"stage_4 ID: "<<CB.boundaryID<<std::endl;
		    polygon_t temp_poly;
		    bg::append(temp_poly, lastPoint);
		    bg::append(temp_poly, t_shifted_lastPoint);
		    bg::append(temp_poly, t_line2_downto_outterRect.back());
		    bg::append(temp_poly, t_line2_downto_outterRect.front());
		    bg::append(temp_poly, lastPoint);
		    bg::unique(temp_poly);
		    bg::correct(temp_poly);
    		    
		    std::string reason;
    		    if (!bg::is_valid(temp_poly, reason)) {
        	        std::cout << "Polygon is invalid: " << reason << std::endl;
    	  	    }
		    double temp_line2_downto_outterRect_area = bg::area(temp_poly) * 1e-08;
	    	    auto calculateTargetLength = [](double area) -> double{
	        	return std::sqrt(2*area);
	    	    };
	            double target_length = calculateTargetLength((CB.phase2_deviation - temp_line2_downto_outterRect_area) * 1e+08);
                    std::cout<<"t2dtor area: "<< temp_line2_downto_outterRect_area <<std::endl; 
	            line_t target_45degree_line, new_last_line;
	    	    point_t new_bendingPoint = shifted_bendingPoint;
            	    double x_bias = 0.0;
            	    double y_bias = 0.0;
		    double stage_4_Area = 0.0;

		    temp_line2 = t_line2_downto_outterRect;
		    std::cout<<"new_bendingPoint: "<<new_bendingPoint.x()<<" "<<new_bendingPoint.y()<<std::endl;
		    for(auto point: temp_line2){
		        std::cout<<"temp_line2: "<<point.x()<<" "<<point.y()<<std::endl;
		    }
		    //若原邊界線最後一段是水平,則到這裡temp_line2必然會成為垂直
		    if(isHorizontal){
            	        x_bias = abs(target_length);
            	        y_bias = 0.0;
			temp_line2.back().y() > bendingPoint.y() ? new_bendingPoint.y(temp_line2.back().y() - x_bias) : 
			    new_bendingPoint.y(temp_line2.back().y() + x_bias);
            		deltaVector[index] < 0 ? x_bias = 0 - x_bias : x_bias = x_bias;
		    }
		    //若原邊界線最後一段是垂直,則到這裡temp_line2必然會成為水平
		    else if(!isHorizontal){
            		y_bias = abs(target_length);
            		x_bias = 0.0;
		        temp_line2.back().x() > bendingPoint.x() ? new_bendingPoint.x(temp_line2.back().x() - y_bias) :
			    new_bendingPoint.x(temp_line2.back().x() + y_bias);
	    		deltaVector[index] < 0 ? y_bias = 0 - y_bias : y_bias = y_bias;
		    }
            	    bg::append(target_45degree_line, point_t(new_bendingPoint.x(), new_bendingPoint.y()));
	    	    //轉45度
	    	    if(x_bias != 0.0){
        	        bg::append(target_45degree_line, point_t(temp_line2.back().x() + x_bias, temp_line2.back().y()));
    	    	    }else if(y_bias != 0.0){
        		bg::append(target_45degree_line, point_t(temp_line2.back().x(), y_bias + temp_line2.back().y()));
    	            }
		    new_last_line.clear();
	    	    new_last_line = temp_line2;
	    	    new_last_line.front().x(target_45degree_line.front().x());
	   	    new_last_line.front().y(target_45degree_line.front().y());
		    for(auto point: new_last_line){
		        std::cout<<"new_last_line_line: "<<point.x()<<" "<<point.y()<<std::endl;
		    }
		    stage_4_Area = cal_bendingPoint_area(target_45degree_line, new_last_line, adjacent_boundary.cornerLine);

		    CB.boundarySegment.pop_back();
		    CB.boundarySegment.emplace_back(shifted_bendingPoint);
		    
		    CB.boundarySegment.emplace_back(shifted_bendingPoint);
		    CB.boundarySegment.push_back(target_45degree_line.front());
		    for(auto &point:target_45degree_line){
		        CB.boundarySegment.push_back(point);
		    }
		    std::cout<<"stage_4_Area: "<<stage_4_Area<<std::endl;
	            if(std::fabs(stage_4_Area - (CB.phase2_deviation - temp_line2_downto_outterRect_area)) < EPS ){
	    	        std::cout<<"Boundary ID: "<<CB.boundaryID<<" at stage_4 solved!\n";
			std::cout<<" & ball dist: "<<bg::distance(net_ball,CB.boundarySegment)<<std::endl;
			continue;
	            }
		}
		//基於bendingPoint平移 + 平移temp_line的組合, 延伸出temp_line3(與temp_line平行)
		//在temp_line上轉90度往outter Rect連接
	    } // end of stage 4
	    
	    std::cout<<"temp_line: \n";
	    for(auto point:temp_line){
	    	std::cout<<point.x()<<", "<<point.y()<<std::endl;
	    }
	    std::cout<<"temp_line2: \n";
	    for(auto point:temp_line2){
	    	std::cout<<point.x()<<", "<<point.y()<<std::endl;
	    }
        }//end of stage_2 & stage_3
    }//end of for loop of ordering_CB
    commonBoundaries = ordering_CB;
}//end of Phase3


void Phase3UpdateAllInfo(std::vector<commonBoundary> &commonBoundaries, std::vector<netInfo> &nets){
    //需考慮到slash平移後 nets的innerboundary也需要更新
    //更新netInfo boundarySegment, outterboundary
    int rows = nets.size();
    auto modIdx = [&] (int idx){
        return (idx+rows) % rows;
    };

    std::cout<<std::setprecision(15);
    
    int counter = nets.size();
    double pitch = 1e+06;
    double EPS = 1e+04;
    double Golden_area = 0.0;
    for(int i = 0; i < commonBoundaries.size(); i ++){
        for(int j = 0;j < nets.size(); j++){
            if(commonBoundaries[i].boundaryID == nets[j].boundary0ID){
                //boundarySegments
	        nets[j].boundarySegments[0].clear();
                nets[j].boundarySegments[0] = commonBoundaries[i].boundarySegment;
            }
            else if (commonBoundaries[i].boundaryID == nets[j].boundary1ID){
	        nets[j].boundarySegments[1].clear(); 
                nets[j].boundarySegments[1] = commonBoundaries[i].boundarySegment;
            }
        }
    }
    for(auto &net : nets){
	Golden_area += net.areaInitial;
        //outterboundary
	net.outterBoundarySegments.clear();
	line_t temp_line;
	//兩條邊界線在同一條軸上
	if(net.boundarySegments[0].back().x() == net.boundarySegments[1].back().x() || net.boundarySegments[0].back().y() == net.boundarySegments[1].back().y()){
		bg::append(temp_line, point_t(net.boundarySegments[0].back().x(), net.boundarySegments[0].back().y()));
		bg::append(temp_line, point_t(net.boundarySegments[1].back().x(), net.boundarySegments[1].back().y()));
		net.outterBoundarySegments.emplace_back(temp_line);
        }
	//兩條邊界線在不同軸上
	else{
	    double temp_b0 = 0.0, temp_b1 = 0.0;
	    line_t temp_line2;
	    abs(net.boundarySegments[0].back().x()) > abs(net.boundarySegments[0].back().y()) ? temp_b0 = net.boundarySegments[0].back().x() : 
		    temp_b0 = net.boundarySegments[0].back().y();
	    abs(net.boundarySegments[1].back().x()) > abs(net.boundarySegments[1].back().y()) ? temp_b1 = net.boundarySegments[1].back().x() :
		    temp_b1 = net.boundarySegments[1].back().y();
	    point_t corner_point;
	    point_t cp0(0 - abs(temp_b0), abs(temp_b1));
	    point_t cp1(abs(temp_b0), abs(temp_b1));
	    point_t cp2(abs(temp_b0), 0 - abs(temp_b1));
	    point_t cp3(0 - abs(temp_b0), 0 - abs(temp_b1));
	    std::vector<point_t> cpVector;
	    std::vector<double> distVector;
	    cpVector.emplace_back(cp0);
	    cpVector.emplace_back(cp1);
	    cpVector.emplace_back(cp2);
	    cpVector.emplace_back(cp3);
	    for(auto point: cpVector){
	        double distance = bg::distance(net.boundarySegments[0].back(), point);
		distVector.emplace_back(distance);
	    }
            int index = -1;
	    double min_val = std::numeric_limits<double>::max();
	    for(int i = 0; i < distVector.size();i++){
	        if(distVector[i] < min_val){
		    min_val = distVector[i];
		    index = i;
		}
	    }
	    corner_point = cpVector[index];
	    bg::append(temp_line, net.boundarySegments[0].back());
	    bg::append(temp_line, corner_point);
	    bg::append(temp_line2, corner_point);
	    bg::append(temp_line2, net.boundarySegments[1].back());
	    net.outterBoundarySegments.emplace_back(temp_line);
	    net.outterBoundarySegments.emplace_back(temp_line2);
	    std::cout<<"ID: "<<net.netID<<"outter_corner_point: "<<corner_point.x()<<", "<<corner_point.y()<<std::endl;
	}

	//inner boundary
	temp_line.clear();
	net.innerBoundarySegments.clear();
	//兩條邊界線在同軸上
	if(net.boundarySegments[0].front().x() == net.boundarySegments[1].front().x() || net.boundarySegments[0].front().y() == net.boundarySegments[1].front().y()){
		bg::append(temp_line, net.boundarySegments[0].front());
		bg::append(temp_line, net.boundarySegments[1].front());
		net.innerBoundarySegments.emplace_back(temp_line);
        }
	//兩條邊界線在不同軸上
	else{
	    double temp_b0 = 0.0, temp_b1 = 0.0;
	    line_t temp_line2;
	    abs(net.boundarySegments[0].front().x()) > abs(net.boundarySegments[0].front().y()) ? temp_b0 = net.boundarySegments[0].front().x() : 
		    temp_b0 = net.boundarySegments[0].front().y();
	    abs(net.boundarySegments[1].front().x()) > abs(net.boundarySegments[1].front().y()) ? temp_b1 = net.boundarySegments[1].front().x() :
		    temp_b1 = net.boundarySegments[1].front().y();
	    point_t corner_point;
	    point_t cp0(0 - abs(temp_b0), abs(temp_b1));
	    point_t cp1(abs(temp_b0), abs(temp_b1));
	    point_t cp2(abs(temp_b0), 0 - abs(temp_b1));
	    point_t cp3(0 - abs(temp_b0), 0 - abs(temp_b1));
	    std::vector<point_t> cpVector;
	    std::vector<double> distVector;
	    cpVector.emplace_back(cp0);
	    cpVector.emplace_back(cp1);
	    cpVector.emplace_back(cp2);
	    cpVector.emplace_back(cp3);
	    for(auto point: cpVector){
	        double distance = bg::distance(net.boundarySegments[0].front(), point);
		distVector.emplace_back(distance);
	    }
            int index = -1;
	    double min_val = std::numeric_limits<double>::max();
	    for(int i = 0; i < distVector.size();i++){
	        if(distVector[i] < min_val){
		    min_val = distVector[i];
		    index = i;
		}
	    }
	    corner_point = cpVector[index];
	    bg::append(temp_line, net.boundarySegments[0].front());
	    bg::append(temp_line, corner_point);
	    bg::append(temp_line2, corner_point);
	    bg::append(temp_line2, net.boundarySegments[1].front());
	    net.innerBoundarySegments.emplace_back(temp_line);
	    net.innerBoundarySegments.emplace_back(temp_line2);
	    std::cout<<"ID: "<<net.netID<<"inner_corner_point: "<<corner_point.x()<<", "<<corner_point.y()<<std::endl;
	}
    }

    Golden_area = Golden_area / nets.size();

    //更新所有nets的area, 更新polygon
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
        else if(nets[i].outterBoundarySegments.size() == 1){
            for(auto &point:nets[i].outterBoundarySegments[0]){
                bg::append(poly,point);
            }
        }
        //boundary 1
        for(auto it = nets[i].boundarySegments[1].rbegin(); it != nets[i].boundarySegments[1].rend(); it++)
            bg::append(poly,*it);
        //inner boundary
        if(nets[i].innerBoundarySegments.size() == 2){
            for(auto it = nets[i].innerBoundarySegments[1].rbegin(); it != nets[i].innerBoundarySegments[1].rend(); it++)
                bg::append(poly,*it);
            for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                bg::append(poly,*it);
        }
        else if (nets[i].innerBoundarySegments.size() == 1 ){
            for(auto it = nets[i].innerBoundarySegments[0].rbegin(); it != nets[i].innerBoundarySegments[0].rend(); it++)
                bg::append(poly,*it);
        }
        bg::unique(poly);
        bg::correct(poly);
        std::string reason;
	//發生自交
        if (!bg::is_valid(poly, reason)) {
            std::cout<<"NetID: "<<nets[i].netID<<"is self intersects!"<<std::endl;
            std::cout<<"Poly WKT: "<<bg::wkt(poly)<<std::endl;
            std::cout << "Polygon is invalid: " << reason << std::endl;
        }
        nets[i].area = bg::area(poly) * 1e-08;
	nets[i].netPolygon = poly;
    }

    for(int i = 0; i < nets.size() ; i++){
	double distance_b0 = bg::distance(nets[i].ball, nets[i].boundarySegments[0]);
	double distance_b1 = bg::distance(nets[i].ball, nets[i].boundarySegments[1]);
	int ip = modIdx(nets[i].netID + 1);
        int im = modIdx(nets[i].netID - 1);
        //bool ALL_PASS = false;

	std::cout<<"dis_b0: "<<distance_b0<<", dis_b1: "<<distance_b1<<std::endl;
	/*
	//檢查自己與前一個polygon是否相交
	if(bg::intersects(nets[i].netPolygon, nets[ip].netPolygon)){
	    std::cout<<"Intersect with previous net!\n";
	    continue;
	}
	//自己與後一個polygon是否相交
	if(bg::intersects(nets[i].netPolygon, nets[im].netPolygon)){
	    std::cout<<"Intersect with following net!\n";
	    continue;
	}
*/
	//檢查自己 若ball都在多邊形內, 距離符合DRC, 面積符合需求
	if(bg::within(nets[i].ball, nets[i].netPolygon)){
	    if(distance_b0 >= (0.5 * pitch) && distance_b1 >= (0.5 * pitch)){
		if(std::fabs(nets[i].area - Golden_area) < EPS){
	            std::cout<<"ALL Correct Net ID: "<<nets[i].netID<<", final area: "<<nets[i].area<<std::endl;
		    counter--;
		}
	    }
	    else{
	        std::cout<<"distance not enough Net ID: "<<nets[i].netID<<", distance between ball: "<<bg::distance(nets[i].ball, nets[i].netPolygon)<<std::endl;
	    }
	}
	else{
	    if(bg::covered_by(nets[i].ball, nets[i].netPolygon)){
	        std::cout<<"ball not within poly but still coverby Net ID: "<<nets[i].netID<<"final_area: "<<nets[i].area<<std::endl;
	    }
	    std::cout<<"ball not within poly Net ID: "<<nets[i].netID<<std::endl;
	}
    }
    if(counter == 0){
        std::cout<<"ALL NET complete!!!!!"<<std::endl;
    }
}//end of Phase3UpdateAllInfo

/*
int main(){
return 0;
}
*/
/*
int main(int argc, char* argv[]){
    std::ifstream file(argv[1]);
    std::vector<double> deltaVector ={
2.43792e+07,7.8292e+06,1.00015e+07,3.1842e+06,0,1.21262e+07,1.17052e+07,1.12867e+07,3.23181e+06,1.22016e+07,5.2865e+06,1.49785e+07,9.59515e+06,1.06103e+07,2.92293e+07,2.31958e+07
 } ;
    std::vector<int> bVector = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0};
    auto nets = parseNetsInfo(file);
    outputPolyInfo(nets);
    auto commonBoundaries = buildCommonBoundaries(nets);
    Rectangle innerRect;
    Rectangle outterRect =Rectangle(-1.4e+08,-1.4e+08,2.8e+08,2.8e+08);
    sortBoundaryClockwise(commonBoundaries,innerRect);
    //同時特例處利Boundary 0
    UpdateCommonBoundaryInfo(commonBoundaries,nets,innerRect,outterRect);
    Phase2UpdateAllInfo_normal_nets(deltaVector, bVector, commonBoundaries, nets);
    Phase3(commonBoundaries,nets,deltaVector);
    outputNetsInfo(nets);
    outputCommonBoundaries(commonBoundaries);
    //outputCommonBoundaries_drawing(commonBoundaries);
    outputNetsInfo_drawing(nets);
    file.close();
    return 0;
}
*/
