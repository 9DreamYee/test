#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <cmath>
#include <limits>
#include <algorithm>
using namespace std;

// 二維座標點結構
struct Point {
    double x, y;
};

// 將點 p 投影到線段 ab 上
static Point projectToSegment(const Point& p, const Point& a, const Point& b) {
    double vx = b.x - a.x;
    double vy = b.y - a.y;
    double wx = p.x - a.x;
    double wy = p.y - a.y;
    double denom = vx*vx + vy*vy;
    double t = (denom > 0 ? (vx*wx + vy*wy) / denom : 0.0);
    t = max(0.0, min(1.0, t));
    return { a.x + t*vx, a.y + t*vy };
}

// 在 outline 四邊中尋找距離 p 最近的投影點，並返回該點與所在邊索引
static pair<Point,int> findNearestOnOutlineIndexed(const Point& p, const vector<Point>& outline) {
    Point bestPt = p;
    double bestD2 = numeric_limits<double>::infinity();
    int bestIdx = -1;
    int n = outline.size();
    for (int i = 0; i < n; ++i) {
        const Point& A = outline[i];
        const Point& B = outline[(i+1)%n];
        Point Q = projectToSegment(p, A, B);
        double dx = p.x - Q.x;
        double dy = p.y - Q.y;
        double d2 = dx*dx + dy*dy;
        if (d2 < bestD2) {
            bestD2 = d2;
            bestPt = Q;
            bestIdx = i;
        }
    }
    return { bestPt, bestIdx };
}

// 計算能放下所有 ball 的大矩形 outline
static vector<Point> computeBallOutline(double x0, double y0, double w, double h,
                                        const array<int,4>& counts, double ballPitch) {
    double Nbot = counts[0], Nrt = counts[1], Ntop = counts[2], Nlt = counts[3];
    double Wreq = max((Nbot>1?(Nbot-1)*ballPitch:0.0), (Ntop>1?(Ntop-1)*ballPitch:0.0));
    double Hreq = max((Nlt>1?(Nlt-1)*ballPitch:0.0), (Nrt>1?(Nrt-1)*ballPitch:0.0));
    double xmin = x0, ymin = y0;
    double xmax = x0 + w, ymax = y0 + h;
    double cx = 0.5*(xmin+xmax);
    double cy = 0.5*(ymin+ymax);
    double halfW = 0.5*Wreq + ballPitch;
    double halfH = 0.5*Hreq + ballPitch;
    return vector<Point>{
        {cx-halfW, cy-halfH},
        {cx+halfW, cy-halfH},
        {cx+halfW, cy+halfH},
        {cx-halfW, cy+halfH}
    };
}

struct PadEntry { int lineIndex; string indent, pinKw, suffix; Point newPos; int edgeIdx; };

int main(int argc, char* argv[]) {
    if (argc != 9) {
        cerr << "用法: " << argv[0] << " input.dsn output.dsn x y width height padPitch ballPitch\n";
        return 1;
    }
    string inFile  = argv[1];
    string outFile = argv[2];
    double x0      = stod(argv[3]);
    double y0      = stod(argv[4]);
    double w       = stod(argv[5]);
    double h       = stod(argv[6]);
    double padPitch  = stod(argv[7]);
    double ballPitch = stod(argv[8]);

    // DIE1 outline
    vector<Point> outline = {{x0,y0},{x0+w,y0},{x0+w,y0+h},{x0,y0+h}};

    // 讀取檔案
    vector<string> lines;
    ifstream ifs(inFile);
    if (!ifs) { cerr<<"無法開啟輸入檔: "<<inFile<<"\n"; return 1; }
    string line;
    while (getline(ifs, line)) lines.push_back(line);
    ifs.close();
    int total = lines.size();

    // DIE1 pad 區間
    int dieS=-1, dieE=-1;
    for (int i=0;i<total;++i) {
        if (dieS<0 && lines[i].find("(image")!=string::npos && lines[i].find("DIE1")!=string::npos)
            dieS = i;
        if (dieS>=0 && dieE<0 && lines[i].find("(outline")!=string::npos)
            dieE = i;
    }
    if (dieS<0 || dieE<0) return 1;

    // 收集 pads
    vector<PadEntry> pads;
    for (int i=dieS+1;i<dieE;++i) {
        auto &ln = lines[i];
        size_t p = ln.find("(pin");
        if (p==string::npos) continue;
        string indent = ln.substr(0,p);
        istringstream iss(ln.substr(p));
        string pin, name, num, xs, ys;
        if (!(iss>>pin>>name>>num>>xs>>ys)) continue;
        while (!ys.empty() && !isdigit(ys.back()) && ys.back()!='.' && ys.back()!='-') ys.pop_back();
        if (ys.empty()) continue;
        double xv=stod(xs), yv=stod(ys);
        auto pr = findNearestOnOutlineIndexed({xv,yv}, outline);
        size_t yPos = ln.find(ys);
        string suffix = (yPos!=string::npos? ln.substr(yPos+ys.size()):")");
        pads.push_back({i, indent, string("(pin ")+name+" "+num+" ", suffix, pr.first, pr.second});
    }
    int M = pads.size();
    vector<bool> keep(M,false);
    for (int i=0;i<M;++i) {
        bool ok=true;
        for (int j=0;j<i;++j) if (keep[j]) {
            double dx = pads[i].newPos.x - pads[j].newPos.x;
            double dy = pads[i].newPos.y - pads[j].newPos.y;
            if (sqrt(dx*dx+dy*dy) < padPitch) { ok=false; break; }
        }
        keep[i] = ok;
    }
    array<int,4> cnt = {0,0,0,0};
    for (int i=0;i<M;++i) if (keep[i]) cnt[pads[i].edgeIdx]++;

    // 計算 ballOutline 同前略...
    double tx=x0, ty=y0;
    if (x0==0 && y0==0) { tx=-w/2; ty=-h/2; }
    auto ballOutline = computeBallOutline(tx,ty,w,h,cnt,ballPitch);

    // 計算 ballPos, ballKeys 同前略...
    vector<Point> ballPos;
    vector<string> ballKeys;
    const char ec[4]={'B','R','T','L'};
    for (int e=0;e<4;++e) {
        int C=cnt[e]; if(C<=0) continue;
        Point A=ballOutline[e], B=ballOutline[(e+1)%4];
        for(int j=0;j<C;++j) {
            double t = (C>1? double(j)/(C-1):0.5);
            ballPos.push_back({ A.x + t*(B.x-A.x), A.y + t*(B.y-A.y) });
            ballKeys.push_back(string(1,ec[e]) + to_string(j+1));
        }
    }

    // 處理 BGA_BGA block 內容，只更新坐標，保留原 token0~2
    int bgaS=-1,bgaE=-1,depth=0;
    for(int i=0;i<total;++i) {
        if (bgaS<0 && lines[i].find("(image")!=string::npos && lines[i].find("BGA_BGA")!=string::npos) {
            bgaS=i;
            depth = count(lines[i].begin(), lines[i].end(), '(')
                  - count(lines[i].begin(), lines[i].end(), ')');
        } else if (bgaS>=0 && bgaE<0) {
            depth += count(lines[i].begin(), lines[i].end(), '(')
                   - count(lines[i].begin(), lines[i].end(), ')');
            if (depth<=0) bgaE=i;
        }
    }
    if (bgaS>=0 && bgaE> bgaS) {
        vector<int> bl;
        for(int i=bgaS+1;i<bgaE;++i)
            if(lines[i].find("(pin")!=string::npos) bl.push_back(i);
        int K=ballPos.size();
        for(int i=0;i<bl.size();++i) {
            int idx=bl[i]; string orig=lines[idx];
            size_t p2=orig.find("(pin"); if(p2==string::npos) continue;
            string indent=orig.substr(0,p2), t2=orig.substr(p2);
            istringstream iss2(t2); vector<string> tok;
            string w;
            while(iss2>>w) tok.push_back(w);
            if(tok.size()<4) continue;
            if(i<K) {
                // 保留 tok[0] tok[1] tok[2]
                ostringstream oss;
                oss << indent
                    << tok[0] << " "
                    << tok[1] << " "
                    << tok[2] << " "
                    << ballPos[i].x << " " << ballPos[i].y << ")";
                lines[idx] = oss.str();
            } else {
                lines[idx] = "#" + orig;
            }
        }
    }

    // 更新 DIE1 pads
    for(int i=0;i<M;++i) {
        int idx=pads[i].lineIndex;
        if(!keep[i]) lines[idx]="#"+lines[idx];
        else {
            ostringstream oss;
            oss<<pads[i].indent<<pads[i].pinKw
               <<pads[i].newPos.x<<" "<<pads[i].newPos.y<<pads[i].suffix;
            lines[idx]=oss.str();
        }
    }

    // 輸出結果
    ofstream ofs(outFile, ios::out|ios::binary);
    if(!ofs) { cerr<<"無法寫入: "<<outFile<<"\n"; return 1; }
    for(auto &l: lines) ofs<<l<<"\n";
    ofs.close();

    cout<<"parse_dsn 完成。"<<endl;
    return 0;
}
