#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cctype>
using namespace std;

struct Point { double x, y; };
struct PadEntry {
    int lineIndex;
    string indent, pinName, pinNum, suffix;
    Point newPos;
    int edgeIdx;
};

static string trim(const string &s) {
    auto a = s.find_first_not_of(" \t\r\n"),
         b = s.find_last_not_of(" \t\r\n");
    return a==string::npos ? "" : s.substr(a, b-a+1);
}

// 投影 p 到线段 ab
static Point projSeg(const Point &p, const Point &a, const Point &b) {
    double vx=b.x-a.x, vy=b.y-a.y;
    double wx=p.x-a.x, wy=p.y-a.y;
    double d=vx*vx+vy*vy;
    double t = d>0 ? (vx*wx+vy*wy)/d : 0;
    t = max(0.0, min(1.0, t));
    return { a.x + t*vx, a.y + t*vy };
}

// 找到 outline 上离 p 最近的投影点及所在边索引
static pair<Point,int> nearest(const Point &p, const vector<Point> &outline) {
    Point best=p;
    double bestD=1e300;
    int bestI=0, n=outline.size();
    for(int i=0;i<n;++i){
        auto q=projSeg(p, outline[i], outline[(i+1)%n]);
        double d2=(p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y);
        if(d2<bestD){ bestD=d2; best=q; bestI=i; }
    }
    return {best,bestI};
}

// 生成矩形 outline
static vector<Point> rectOutline(double x0,double y0,double w,double h){
    return {{x0,y0},{x0+w,y0},{x0+w,y0+h},{x0,y0+h}};
}

// parse DIE1 pin 并投影到 outline 上，不写回 lines
static array<int,4> parsePads(
    const vector<string> &lines, int s, int e,
    const vector<Point> &outline,
    double padPitch,
    vector<PadEntry> &pads
){
    array<int,4> cnt{0,0,0,0};
    for(int i=s+1;i<e;++i){
        auto pos=lines[i].find("(pin");
        if(pos==string::npos) continue;
        string indent=lines[i].substr(0,pos),
               rest=lines[i].substr(pos);
        istringstream iss(rest);
        string pin,name,num,xs,ys;
        if(!(iss>>pin>>name>>num>>xs>>ys)) continue;
        while(!ys.empty() && !(isdigit(ys.back())||ys.back()=='.'||ys.back()=='-'))
            ys.pop_back();
        Point oldp{stod(xs),stod(ys)};
        auto pr=nearest(oldp, outline);
        size_t yp=rest.find(ys);
        string suf = yp!=string::npos ? rest.substr(yp+ys.size()) : ")";
        pads.push_back({i,indent,name,num,suf,pr.first,pr.second});
        ++cnt[pr.second];
    }
    return cnt;
}

// Pad 消重叠，保证同边至少 padPitch 距离
static void resolveOverlaps(
    vector<PadEntry> &pads,
    double padPitch,
    const vector<Point> &outline
){
    array<vector<PadEntry*>,4> groups;
    for(auto &pe:pads) groups[pe.edgeIdx].push_back(&pe);
    for(int e=0;e<4;++e){
        auto &v=groups[e];
        if(v.size()<2) continue;
        sort(v.begin(), v.end(),[&](auto *A, auto *B){
            if(e==0) return A->newPos.x < B->newPos.x;
            if(e==1) return A->newPos.y < B->newPos.y;
            if(e==2) return A->newPos.x > B->newPos.x;
            return A->newPos.y > B->newPos.y;
        });
        Point A=outline[e], B=outline[(e+1)%4];
        double dx=B.x-A.x, dy=B.y-A.y;
        double L=hypot(dx,dy), ux=dx/L, uy=dy/L;
        double tPrev=((v[0]->newPos.x-A.x)*ux + (v[0]->newPos.y-A.y)*uy);
        for(int i=1;i<v.size();++i){
            double tCur=((v[i]->newPos.x-A.x)*ux + (v[i]->newPos.y-A.y)*uy);
            if(tCur-tPrev < padPitch){
                tCur = min(tPrev+padPitch, L);
                v[i]->newPos = {A.x+ux*tCur, A.y+uy*tCur};
            }
            tPrev=tCur;
        }
    }
}

// 重写 BGA_BGA image block 中的 pin
static void rewriteBGA(
    const vector<Point> &bp,
    const vector<string> &bk,
    vector<string> &lines
){
    int bs=-1,be=-1,depth=0;
    string padName;
    int n=lines.size();
    for(int i=0;i<n;++i){
        if(bs<0 && lines[i].find("(image")!=string::npos
               && lines[i].find("BGA_BGA")!=string::npos){
            bs=i;
            depth = count(lines[i].begin(),lines[i].end(),'(')
                  - count(lines[i].begin(),lines[i].end(),')');
        } else if(bs>=0 && be<0){
            depth += count(lines[i].begin(),lines[i].end(),'(')
                   - count(lines[i].begin(),lines[i].end(),')');
            if(i==bs+1){
                istringstream iss(trim(lines[i]));
                string tok; iss>>tok>>padName;
            }
            if(depth<=0) be=i;
        }
    }
    if(bs<0||be<0) return;
    string indent="   ";
    auto p=lines[bs+1].find("(pin");
    if(p!=string::npos) indent=lines[bs+1].substr(0,p);
    lines.erase(lines.begin()+bs+1, lines.begin()+be);
    vector<string> np;
    for(int i=0;i<bp.size();++i){
        ostringstream os;
        os<<indent<<"(pin "<<padName<<" "<<bk[i]
          <<" "<<bp[i].x<<" "<<bp[i].y<<")";
        np.push_back(os.str());
    }
    lines.insert(lines.begin()+bs+1, np.begin(), np.end());
}

// 解析 signal boundary
static bool parseSignalRect(
    const vector<string> &lines,
    double &x0,double &y0,double &x1,double &y1
){
    for(auto &l:lines){
        auto t=trim(l);
        if(t.rfind("(boundary (rect signal",0)==0){
            istringstream iss(t);
            string b,r,name;
            iss>>b>>r>>name>>x0>>y0>>x1>>y1;
            return true;
        }
    }
    return false;
}

int main(int argc,char*argv[]){
    if(argc!=11){
        cerr<<"Usage: parse_dsn input.dsn output.dsn x0 y0 w h padPitch ballPitch marginPercent boundaryExpand\n";
        return 1;
    }
    string inF=argv[1], outF=argv[2];
    double x0=stod(argv[3]), y0=stod(argv[4]);
    double w=stod(argv[5]), h=stod(argv[6]);
    double padPitch=stod(argv[7]), ballPitch=stod(argv[8]);
    double marginPct=stod(argv[9]), boundaryExpand=stod(argv[10]);

    // 读入所有行
    vector<string> lines;
    { ifstream ifs(inF);
      for(string L;getline(ifs,L);) lines.push_back(L);
    }

    // 1. 调整 rule(clearance,width)
    {
        double rc=padPitch*2.0/3.0, rw=padPitch*1.0/3.0;
        ostringstream osC,osW; osC<<rc; osW<<rw;
        for(auto &l:lines){
            auto t=trim(l);
            if(t.rfind("(rule (clearance",0)==0){
                auto p=l.find("(rule");
                string pre=p!=string::npos?l.substr(0,p):"";
                l=pre+"(rule (clearance "+osC.str()+"))";
            } else if(t.rfind("(rule (width",0)==0){
                auto p=l.find("(rule");
                string pre=p!=string::npos?l.substr(0,p):"";
                l=pre+"(rule (width "+osW.str()+"))";
            }
        }
    }

    // 2. 找 DIE1 block outline
    auto dieOutline=rectOutline(x0,y0,w,h);
    int ds=-1,de=-1, n=lines.size();
    for(int i=0;i<n;++i){
        if(ds<0 && lines[i].find("(image")!=string::npos
               && lines[i].find("DIE1")!=string::npos) ds=i;
        if(ds>=0 && de<0 && lines[i].find("(outline")!=string::npos) de=i;
    }
    if(ds<0||de<0){ cerr<<"DIE1 block not found\n"; return 1; }

    // 3. parsePads + resolveOverlaps
    vector<PadEntry> pads;
    auto padCnt=parsePads(lines, ds, de, dieOutline, padPitch, pads);
    resolveOverlaps(pads, padPitch, dieOutline);

    // 4. 第一代 Ball：线性插值 P_e,i & U_e,i
    double sx0,sy0,sx1,sy1;
    if(!parseSignalRect(lines,sx0,sy0,sx1,sy1)){ cerr<<"signal boundary not found\n"; return 1; }
    double sw=sx1-sx0, sh=sy1-sy0;
    double mx=sw*marginPct/100.0, my=sh*marginPct/100.0;
    double bx0=sx0+mx, by0=sy0+my, bw=sw-2*mx, bh=sh-2*my;
    auto ballOutlineScaled=rectOutline(bx0,by0,bw,bh);

    int totalPads=padCnt[0]+padCnt[1]+padCnt[2]+padCnt[3];
    array<vector<PadEntry*>,4> pg;
    for(auto &pe:pads) pg[pe.edgeIdx].push_back(&pe);
    for(int e=0;e<4;++e){
        sort(pg[e].begin(),pg[e].end(),[&](auto *A,auto *B){
            if(e==0) return A->newPos.x<B->newPos.x;
            if(e==1) return A->newPos.y<B->newPos.y;
            if(e==2) return A->newPos.x>B->newPos.x;
            return A->newPos.y>B->newPos.y;
        });
    }

    vector<Point> P, U;
    vector<string> ballKeys;
    const char EN[4]={'B','R','T','L'};
    for(int e=0;e<4;++e){
        int N=pg[e].size();
        if(N==0) continue;
        Point A=ballOutlineScaled[e], B=ballOutlineScaled[(e+1)%4];
        bool asc=(e==0||e==2);
        double alpha = double(padCnt[e])/double(totalPads);
        for(int i=0;i<N;++i){
            // P_e,i：pad->newPos 投影到 edge
            Point proj = projSeg(pg[e][i]->newPos, A, B);
            P.push_back(proj);
            // U_e,i：均匀分布
            double t = asc ? (N>1? double(i)/(N-1) : 0.5)
                           : double(i+1)/(N+1);
            U.push_back({A.x + t*(B.x-A.x), A.y + t*(B.y-A.y)});
            ballKeys.push_back(string(1,EN[e]) + to_string(i+1));
        }
    }
    vector<Point> firstBallPos;
    firstBallPos.reserve(P.size());
    int idx=0;
    for(int e=0;e<4;++e){
        int N=pg[e].size();
        if(N==0) continue;
        double alpha = double(padCnt[e])/double(totalPads);
		alpha = 1.0;
        for(int i=0;i<N;++i,++idx){
            firstBallPos.push_back({
                P[idx].x*(1-alpha) + U[idx].x*alpha,
                P[idx].y*(1-alpha) + U[idx].y*alpha
            });
        }
    }

    // —— 5. 输出 original_coords.txt，并写入第一代 Ball 的 Rect 信息 —— 
    {
        ofstream fout("original_coords.txt");
    fout << "Ball Rect: "
	     << "xmin="<< bx0
         << " ymin=" << by0
         << " xmax=" << bw+bx0
         << " ymax=" << bh+by0
         << "\n";
	fout << "Full Rect: "
         << "xmin=" << sx0
         << " ymin=" << sy0
         << " xmax=" << sx1
         << " ymax=" << sy1
         << "\n";


        // 按边分组并排序 Pads
        array<vector<PadEntry*>,4> pb{};
        for (auto &pe : pads) pb[pe.edgeIdx].push_back(&pe);
        for (int e = 0; e < 4; ++e) {
            auto &v = pb[e];
            sort(v.begin(), v.end(), [e](PadEntry* A, PadEntry* B){
                if (e == 0) return A->newPos.x < B->newPos.x;
                if (e == 1) return A->newPos.y < B->newPos.y;
                if (e == 2) return A->newPos.x > B->newPos.x;
                return          A->newPos.y > B->newPos.y;
            });
        }

        // 按边分组并排序第一代 Ball
        array<vector<int>,4> bb{};
        for (int i = 0; i < (int)ballKeys.size(); ++i) {
            int e = (ballKeys[i][0]=='B'?0
                  : ballKeys[i][0]=='R'?1
                  : ballKeys[i][0]=='T'?2 : 3);
            bb[e].push_back(i);
        }
        for (int e = 0; e < 4; ++e) {
            auto &v = bb[e];
            sort(v.begin(), v.end(), [&](int i, int j){
                if (e == 0) return firstBallPos[i].x < firstBallPos[j].x;
                if (e == 1) return firstBallPos[i].y < firstBallPos[j].y;
                if (e == 2) return firstBallPos[i].x > firstBallPos[j].x;
                return          firstBallPos[i].y > firstBallPos[j].y;
            });
        }

        // 输出 Net / Pad_location / Ball_location
        int netIdx = 0;
        for (int e = 0; e < 4; ++e) {
            int Pn = pb[e].size();
            int Bn = bb[e].size();
            int M  = min(Pn, Bn);
            for (int i = 0; i < M; ++i) {
                ++netIdx;
                fout << "Net" << netIdx << ":\n";
                fout << "Pad_location: (" 
                     << pb[e][i]->newPos.x << "," 
                     << pb[e][i]->newPos.y << ")\n";
                int bi = bb[e][i];
                fout << "Ball_location: (" 
                     << firstBallPos[bi].x << "," 
                     << firstBallPos[bi].y << ")\n";
            }
        }
    }


    // 6. Pad 中点→最近边投影 替换 pads
    {
        // 已有 pg[e] 和 dieOutline
        vector<PadEntry> newPads;
        for(int e=0;e<4;++e){
            auto &v=pg[e];
            // 相邻两 pad 欧氏中点→投影
            for(int i=0;i+1<v.size();++i){
                auto *A=v[i], *B=v[i+1];
                Point mid{0.5*(A->newPos.x+B->newPos.x),
                          0.5*(A->newPos.y+B->newPos.y)};
                auto pr=nearest(mid, dieOutline);
                PadEntry np=*A;
                np.newPos = pr.first;
                newPads.push_back(np);
            }
            // 邻边角落用同样中点→投影
            int e0=(e+3)%4, e1=e;
            auto &v0=pg[e0], &v1=pg[e1];
            if(!v0.empty() && !v1.empty()){
                auto *A=v0.back(), *B=v1.front();
                Point mid{0.5*(A->newPos.x+B->newPos.x),
                          0.5*(A->newPos.y+B->newPos.y)};
                auto pr=nearest(mid, dieOutline);
                PadEntry np=*A;
                np.newPos=pr.first;
                newPads.push_back(np);
            }
        }
        pads.swap(newPads);
		rotate(pads.begin(), pads.begin() + 1, pads.end());
    }

    // —— 7. 写回 DIE1 pin block （第二代 Pad） —— 
    {
        int pinStart = -1, pinEnd = -1, depth = 0;
        for (int i = ds; i <= de; ++i) {
            if (trim(lines[i]).rfind("(pin", 0) == 0 && pinStart < 0) {
                pinStart = i;
            }
            if (pinStart >= 0 && pinEnd < 0) {
                depth += count(lines[i].begin(), lines[i].end(), '(')
                       - count(lines[i].begin(), lines[i].end(), ')');
                if (depth <= 1 && trim(lines[i]).rfind("(pin", 0) != 0) {
                    pinEnd = i;
                    break;
                }
            }
        }
        if (pinStart < 0) pinStart = ds + 1;
        if (pinEnd   < 0) pinEnd   = de;
        lines.erase(lines.begin() + pinStart, lines.begin() + pinEnd);

        // epsilon for floating‐point comparison
        const double eps = 1e-6;
        vector<string> out;
        for (auto &pe : pads) {
            ostringstream os;
            os << pe.indent
               << "(pin " << pe.pinName << " " << pe.pinNum
               << " " << pe.newPos.x << " " << pe.newPos.y;

            // 坐标判断：真正落在左/右边才旋转
            bool onLeft  = fabs(pe.newPos.x - x0)      < eps;
            bool onRight = fabs(pe.newPos.x - (x0 + w)) < eps;
            if (onLeft || onRight) {
                os << " (rotate 90)";
            }

            os << pe.suffix;
            out.push_back(os.str());
        }
        lines.insert(lines.begin() + pinStart, out.begin(), out.end());
    }


// 8. 第二代 Ball：逐边取中点 + 角落中点，投影到 scaledBallOutline
array<vector<int>,4> ballsByEdge{};
for (int i = 0; i < (int)ballKeys.size(); ++i) {
    int e = (ballKeys[i][0]=='B'?0
          : ballKeys[i][0]=='R'?1
          : ballKeys[i][0]=='T'?2 : 3);
    ballsByEdge[e].push_back(i);
}

vector<Point> newBallPos;
vector<string> newBallKeys;
// 每条边上相邻两球的中点
for (int e = 0; e < 4; ++e) {
    auto &lst = ballsByEdge[e];
    for (int i = 0; i + 1 < (int)lst.size(); ++i) {
        Point A = firstBallPos[lst[i]];
        Point B = firstBallPos[lst[i+1]];
        Point mid{ 0.5*(A.x + B.x), 0.5*(A.y + B.y) };
        // 投影到 scaledBallOutline
        auto pr = nearest(mid, ballOutlineScaled);
        newBallPos.push_back(pr.first);
        newBallKeys.push_back(ballKeys[lst[i+1]]);
    }
}
// 四个角的跨边中点
for (int e = 0; e < 4; ++e) {
    int e0 = (e + 3) % 4;
    auto &v0 = ballsByEdge[e0];
    auto &v1 = ballsByEdge[e];
    if (!v0.empty() && !v1.empty()) {
        Point A = firstBallPos[v0.back()];
        Point B = firstBallPos[v1.front()];
        Point mid{ 0.5*(A.x + B.x), 0.5*(A.y + B.y) };
        auto pr = nearest(mid, ballOutlineScaled);
        newBallPos.push_back(pr.first);
        newBallKeys.push_back(ballKeys[v1.front()]);
    }
}

// 9. 重写 BGA_BGA 区块
rewriteBGA(newBallPos, newBallKeys, lines);


    // 10. 重建 network block — 全局环状配对并 shift=1 —
    {
        // —— a) 按旧逻辑分组／排序后的 pbe, bbe —— 保持不变 —— 
        array<vector<PadEntry>,4> pbe;
        array<vector<string>,4> bbe;
        for (auto &pe : pads)               pbe[pe.edgeIdx].push_back(pe);
        for (int e = 0; e < 4; ++e) {
            sort(pbe[e].begin(), pbe[e].end(), [e](auto &A, auto &B) {
                if      (e==0) return A.newPos.x < B.newPos.x;
                else if (e==1) return A.newPos.y < B.newPos.y;
                else if (e==2) return A.newPos.x > B.newPos.x;
                else            return A.newPos.y > B.newPos.y;
            });
        }
        for (auto &k : newBallKeys) {
            int e = (k[0]=='B'?0 : k[0]=='R'?1 : k[0]=='T'?2 : 3);
            bbe[e].push_back(k);
        }
        for (int e = 0; e < 4; ++e) {
            sort(bbe[e].begin(), bbe[e].end(), [](auto &a, auto &b){
                return stoi(a.substr(1)) < stoi(b.substr(1));
            });
        }

        // —— b) 展平成线性列表 —— 
        vector<PadEntry> padList;
        vector<string>    ballList;
        for (int e = 0; e < 4; ++e) {
            for (auto &pe : pbe[e])    padList.push_back(pe);
            for (auto &bk : bbe[e])    ballList.push_back(bk);
        }
        int N = min(padList.size(), ballList.size());
        int shift = 1;  // 向前偏移 1

        // —— c) 构造 network block —— 
        vector<string> nb;
        nb.push_back("(network");
        for (int i = 0; i < N; ++i) {
            int bi = (i + shift) % N;
            nb.push_back("  (net NET_" + to_string(i+1));
            nb.push_back("    (pins BGA-" + ballList[bi]
                       + " DIE1-" + padList[i].pinNum + "))");
        }
        nb.push_back(")");

        // —— d) 替换原有 network 块 —— 
        int ns=-1, ne=-1, depth=0;
        for (int i = 0; i < (int)lines.size(); ++i) {
            if (ns < 0 && trim(lines[i]).rfind("(network",0) == 0) {
                ns = i;
                depth = count(lines[i].begin(), lines[i].end(), '(')
                      - count(lines[i].begin(), lines[i].end(), ')');
            } else if (ns >= 0 && ne < 0) {
                depth += count(lines[i].begin(), lines[i].end(), '(')
                       - count(lines[i].begin(), lines[i].end(), ')');
                if (depth == 0) { ne = i; break; }
            }
        }
        if (ns >= 0 && ne > ns) {
            vector<string> tmp;
            tmp.insert(tmp.end(), lines.begin(), lines.begin()+ns);
            tmp.insert(tmp.end(), nb.begin(), nb.end());
            tmp.insert(tmp.end(), lines.begin()+ne+1, lines.end());
            lines.swap(tmp);
        }
    }


    // 11. 写出 aligned .dsn
    ofstream ofs(outF);
    for(auto &l:lines) ofs<<l<<"\n";
    return 0;
}
