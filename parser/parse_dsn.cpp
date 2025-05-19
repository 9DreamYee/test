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

static string trim(const string &s) {
    auto a = s.find_first_not_of(" \t\r\n");
    auto b = s.find_last_not_of(" \t\r\n");
    return (a == string::npos ? string() : s.substr(a, b - a + 1));
}

struct Point { double x, y; };
struct PadEntry { int lineIndex; string indent, pinName, pinNum, suffix; Point newPos; int edgeIdx; };
using PPI = pair<Point,int>;

static Point projSeg(const Point& p, const Point& a, const Point& b) {
    double vx=b.x-a.x, vy=b.y-a.y, wx=p.x-a.x, wy=p.y-a.y;
    double d=vx*vx+vy*vy;
    double t = d>0 ? (vx*wx+vy*wy)/d : 0;
    t = max(0.0, min(1.0, t));
    return {a.x + t*vx, a.y + t*vy};
}

static PPI nearest(const Point& p, const vector<Point>& outline) {
    Point best = p; double bestD2 = numeric_limits<double>::infinity(); int bestIdx = -1;
    for (int i = 0; i < (int)outline.size(); ++i) {
        Point q = projSeg(p, outline[i], outline[(i+1)%outline.size()]);
        double dx = p.x - q.x, dy = p.y - q.y, d2 = dx*dx + dy*dy;
        if (d2 < bestD2) { bestD2 = d2; best = q; bestIdx = i; }
    }
    return {best, bestIdx};
}

static vector<Point> dieOutline(double x0,double y0,double w,double h) {
    return {{x0,y0},{x0+w,y0},{x0+w,y0+h},{x0,y0+h}};
}

static array<int,4> parsePads(vector<string>& lines,int s,int e,const vector<Point>& outline,
                              double padPitch, vector<PadEntry>& pads) {
    array<int,4> cnt = {0,0,0,0};
    for (int i = s+1; i < e; ++i) {
        auto pos = lines[i].find("(pin"); if (pos == string::npos) continue;
        string indent = lines[i].substr(0,pos), rest = lines[i].substr(pos);
        istringstream iss(rest); string pin, name, num, xs, ys;
        if (!(iss>>pin>>name>>num>>xs>>ys)) continue;
        while (!ys.empty() && !(isdigit((unsigned char)ys.back())||ys.back()=='.'||ys.back()=='-'))
            ys.pop_back();
        Point oldp{stod(xs), stod(ys)};
        auto pr = nearest(oldp, outline);
        size_t yp = rest.find(ys);
        string suf = yp!=string::npos ? rest.substr(yp+ys.size()) : ")";
        pads.push_back({i, indent, name, num, suf, pr.first, pr.second});
    }
    for (auto &pe : pads) {
        ostringstream os;
        os << pe.indent << "(pin " << pe.pinName << " " << pe.pinNum
           << " " << pe.newPos.x << " " << pe.newPos.y << pe.suffix;
        lines[pe.lineIndex] = os.str();
        ++cnt[pe.edgeIdx];
    }
    return cnt;
}

static vector<Point> ballOutline(double x0,double y0,double w,double h,
                                 const array<int,4>& cnt,double pitch) {
    double N0=cnt[0],N1=cnt[1],N2=cnt[2],N3=cnt[3];
    double W = max((N0>1?(N0-1)*pitch:0.0), (N2>1?(N2-1)*pitch:0.0));
    double H = max((N3>1?(N3-1)*pitch:0.0), (N1>1?(N1-1)*pitch:0.0));
    double cx=x0+w/2, cy=y0+h/2, hw=W/2+pitch, hh=H/2+pitch;
    return {{cx-hw,cy-hh}, {cx+hw,cy-hh}, {cx+hw,cy+hh}, {cx-hw,cy+hh}};
}

static void genBalls(const vector<Point>& bo,const array<int,4>& cnt,double pitch,
                     vector<Point>& bp,vector<string>& ballKeys) {
    const char ec[4] = {'B','R','T','L'};
    for (int e = 0; e < 4; ++e) {
        int N = cnt[e]; if (N <= 0) continue;
        bool inc = (e==0||e==2);
        Point A = bo[e], B = bo[(e+1)%4];
        for (int j = 0; j < N; ++j) {
            double t = inc ? (N>1?double(j)/(N-1):0.5)
                           : double(j+1)/double(N+1);
            bp.push_back({A.x + t*(B.x-A.x), A.y + t*(B.y-A.y)});
            ballKeys.push_back(string(1,ec[e]) + to_string(j+1));
        }
    }
}

static void rewriteBGA(const vector<Point>& bp,const vector<string>& bk,vector<string>& lines) {
    int bs=-1, be=-1, depth=0;
    string padName;
    // 定位 BGA_BGA image block
    for (int i = 0; i < (int)lines.size(); ++i) {
        if (bs<0 && lines[i].find("(image")!=string::npos
                  && lines[i].find("BGA_BGA")!=string::npos) {
            bs = i;
            depth = count(lines[i].begin(),lines[i].end(),'(')
                  - count(lines[i].begin(),lines[i].end(),')');
        } else if (bs>=0 && be<0) {
            depth += count(lines[i].begin(),lines[i].end(),'(')
                   - count(lines[i].begin(),lines[i].end(),')');
            if (bs+1==i) {
                istringstream iss(trim(lines[i]));
                string token; iss>>token>>padName;
            }
            if (depth<=0) be = i;
        }
    }
    if (bs<0||be<0) return;
    // 获取缩进
    string pinIndent = "   ";
    auto p = lines[bs+1].find("(pin");
    if (p!=string::npos) pinIndent = lines[bs+1].substr(0,p);
    // 删除旧的
    lines.erase(lines.begin()+bs+1, lines.begin()+be);
    // 插入新的
    vector<string> newPins;
    for (int k = 0; k < (int)bp.size(); ++k) {
        ostringstream os;
        os << pinIndent << "(pin " << padName
           << " " << bk[k]
           << " " << bp[k].x << " " << bp[k].y << ")";
        newPins.push_back(os.str());
    }
    lines.insert(lines.begin()+bs+1, newPins.begin(), newPins.end());
}

static void debugCounts(const array<int,4>& pc,const vector<string>& bk) {
    array<int,4> bc={0,0,0,0};
    for (auto &k: bk) {
        int e = (k[0]=='B'?0:k[0]=='R'?1:k[0]=='T'?2:3);
        ++bc[e];
    }
    cerr<<"DEBUG Pads:   B="<<pc[0]<<" R="<<pc[1]
        <<" T="<<pc[2]<<" L="<<pc[3]<<"\n";
    cerr<<"DEBUG Balls:  B="<<bc[0]<<" R="<<bc[1]
        <<" T="<<bc[2]<<" L="<<bc[3]<<"\n";
}

int main(int argc,char*argv[]) {
    if (argc!=9) {
        cerr<<"Usage: parse_dsn input.dsn output.dsn x0 y0 w h padPitch ballPitch\n";
        return 1;
    }
    string inF=argv[1], outF=argv[2];
    double x0=stod(argv[3]), y0=stod(argv[4]);
    double w=stod(argv[5]), h=stod(argv[6]);
    double padPitch=stod(argv[7]), ballPitch=stod(argv[8]);

    vector<string> lines;
    ifstream ifs(inF);
    for (string l; getline(ifs,l);) lines.push_back(l);
    ifs.close();

    auto outline = dieOutline(x0,y0,w,h);
    int ds=-1,de=-1;
    for (int i=0;i<(int)lines.size();++i) {
        if (ds<0 && lines[i].find("(image")!=string::npos
                  && lines[i].find("DIE1")!=string::npos) ds=i;
        if (ds>=0 && de<0 && lines[i].find("(outline")!=string::npos) de=i;
    }
    if (ds<0||de<0) { cerr<<"DIE1 block not found\n"; return 1; }

    vector<PadEntry> pads;
    auto padCnt = parsePads(lines, ds, de, outline, padPitch, pads);

    double tx=x0, ty=y0;
    if (x0==0 && y0==0) { tx=-w/2; ty=-h/2; }
    auto bo = ballOutline(tx, ty, w, h, padCnt, ballPitch);

    vector<Point> bp; vector<string> bk;
    genBalls(bo, padCnt, ballPitch, bp, bk);

    rewriteBGA(bp, bk, lines);
    debugCounts(padCnt, bk);

    // 重新生成 network 区块
    array<vector<PadEntry>,4> pbe; array<vector<string>,4> bbe;
    for (auto &pe : pads) pbe[pe.edgeIdx].push_back(pe);
    for (int e=0;e<4;++e) {
        sort(pbe[e].begin(), pbe[e].end(), [e](auto &a, auto &b){
            if(e==0) return a.newPos.x < b.newPos.x;
            if(e==1) return a.newPos.y < b.newPos.y;
            if(e==2) return a.newPos.x > b.newPos.x;
            return a.newPos.y > b.newPos.y;
        });
    }
    for (auto &k: bk) bbe[string("BRTL").find(k[0])].push_back(k);
    for (int e=0;e<4;++e)
        sort(bbe[e].begin(), bbe[e].end(), [](auto &a, auto &b){
            return stoi(a.substr(1)) < stoi(b.substr(1));
        });

    vector<string> netBlock;
    netBlock.push_back("(network");
    int netIdx=1;
    for (int e=0;e<4;++e) {
        int N=min((int)pbe[e].size(), (int)bbe[e].size());
        for (int i=0;i<N;++i) {
            netBlock.push_back("  (net NET_" + to_string(netIdx));
            netBlock.push_back("    (pins BGA-" + bbe[e][i] + " DIE1-" + pbe[e][i].pinNum + "))");
            ++netIdx;
        }
    }
    netBlock.push_back(")");

    // 用深度匹配替换 file 中的第一个顶级 network 区块
    int ns=-1, ne=-1, depth=0;
    for (int i=0; i<(int)lines.size(); ++i) {
        if (ns<0 && trim(lines[i]).rfind("(network",0)==0) {
            ns = i;
            depth = 1; 
            // count extra '(' on same line
            depth += count(lines[i].begin(), lines[i].end(),'(') - 1;
            depth -= count(lines[i].begin(), lines[i].end(),')');
        } else if (ns>=0 && ne<0) {
            depth += count(lines[i].begin(), lines[i].end(),'(')
                   - count(lines[i].begin(), lines[i].end(),')');
            if (depth==0) {
                ne = i;
                break;
            }
        }
    }
    if (ns>=0 && ne>ns) {
        vector<string> out;
        out.insert(out.end(), lines.begin(), lines.begin()+ns);
        out.insert(out.end(), netBlock.begin(), netBlock.end());
        out.insert(out.end(), lines.begin()+ne+1, lines.end());
        lines.swap(out);
    }

    ofstream ofs(outF);
    for (auto &l: lines) ofs<<l<<"\n";
    return 0;
}
