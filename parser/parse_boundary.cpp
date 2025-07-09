// parse_boundary.cpp — 統合新版 applyLift

#include <bits/stdc++.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>

using namespace std;
namespace bg = boost::geometry;

// 基本型別
using Pt      = bg::model::point<double,2,bg::cs::cartesian>;
using LS      = bg::model::linestring<Pt>;
using LSM     = map<string,LS>;
using Seg     = bg::model::segment<Pt>;
using Polygon = bg::model::polygon<Pt>;

struct PB   { Pt pad, ball; };
struct Rect { double xmin,ymin,xmax,ymax; };
enum Side { NONE, LEFT, RIGHT, BOTTOM, TOP };

// 輔助函式
static string trim(const string &s) {
    auto a = s.find_first_not_of(" \t\r\n");
    auto b = s.find_last_not_of(" \t\r\n");
    return (a==string::npos) ? string() : s.substr(a, b-a+1);
}
static int countChar(const string &s, char c) {
    return int(count(s.begin(), s.end(), c));
}

// 判斷點是否在 Rect 邊上
Side whichSide(const Pt &p, const Rect &r) {
    double eps = 1e-6;
    double x = bg::get<0>(p), y = bg::get<1>(p);
    if (fabs(x - r.xmin) <= eps) return LEFT;
    if (fabs(x - r.xmax) <= eps) return RIGHT;
    if (fabs(y - r.ymin) <= eps) return BOTTOM;
    if (fabs(y - r.ymax) <= eps) return TOP;
    return NONE;
}

// 1. 解析 SES header
struct SESInfo { double res=1, offX=0, offY=0; };
SESInfo parseSesInfo(const string &file){
    SESInfo inf;
    ifstream fin(file); if(!fin) throw runtime_error("open SES fail");
    string line;
    while(getline(fin,line)){
        auto t=trim(line);
        if(t.rfind("(resolution",0)==0){
            istringstream is(t);
            string tag, unit;
            is>>tag>>unit>>inf.res;
        } else if(t.rfind("(place",0)==0){
            istringstream is(t);
            string plc, comp; double x,y;
            is>>plc>>comp>>x>>y;
            if(comp=="DIE1"){
                inf.offX=x; inf.offY=y;
            }
        }
    }
    return inf;
}

// 2. 解析 original_coords
void readOriginalCoords(
    const string &file,
    const SESInfo &inf,
    map<string,PB> &pbs,
    Rect &ballRect,
    Rect &fullRect
){
    ifstream fin(file); if(!fin) throw runtime_error("open orig fail");
    string line, cur;
    while(getline(fin,line)){
        auto t=trim(line);
        if(t.rfind("Ball Rect:",0)==0){
            sscanf(t.c_str(),
                   "Ball Rect: xmin=%lf ymin=%lf xmax=%lf ymax=%lf",
                   &ballRect.xmin,&ballRect.ymin,
                   &ballRect.xmax,&ballRect.ymax);
        }
        else if(t.rfind("Full Rect:",0)==0){
            sscanf(t.c_str(),
                   "Full Rect: xmin=%lf ymin=%lf xmax=%lf ymax=%lf",
                   &fullRect.xmin,&fullRect.ymin,
                   &fullRect.xmax,&fullRect.ymax);
        }
        else if(t.rfind("Net",0)==0 && t.find(':')!=string::npos){
            cur = t.substr(0,t.find(':'));
        }
        else if(t.rfind("Pad_location",0)==0){
            double x,y;
            sscanf(line.c_str()+line.find('(')+1,"%lf,%lf",&x,&y);
            PB &b = pbs[cur];
            double px = round(x*inf.res) + inf.offX;
            double py = round(y*inf.res) + inf.offY;
            bg::set<0>(b.pad, px);
            bg::set<1>(b.pad, py);
        }
        else if(t.rfind("Ball_location",0)==0){
            double x,y;
            sscanf(line.c_str()+line.find('(')+1,"%lf,%lf",&x,&y);
            PB &b = pbs[cur];
            bg::set<0>(b.ball, round(x*inf.res));
            bg::set<1>(b.ball, round(y*inf.res));
        }
    }
}

// 3. parse SES paths
LSM parseSesPaths(
    const string &file,
    double minX,double minY,
    double maxX,double maxY
){
    ifstream fin(file); if(!fin) throw runtime_error("open SES paths fail");
    LSM nets;
    string line,name;
    int depth=0;
    vector<Pt> buf;
    const double eps=10.0; // 保持原值，僅型別改為 double

    auto flush=[&](){
        if(name.empty()||buf.size()<2){ buf.clear(); name.clear(); return; }
        vector<Pt> pts;
        for(auto &p:buf)
            if(pts.empty() || bg::distance(p,pts.back())>0)
                pts.push_back(p);
        buf.clear();
        if(pts.size()<2){ name.clear(); return; }

        auto onEdge=[&](const Pt &p){
            return fabs(bg::get<0>(p)-minX)<=eps
                || fabs(bg::get<0>(p)-maxX)<=eps
                || fabs(bg::get<1>(p)-minY)<=eps
                || fabs(bg::get<1>(p)-maxY)<=eps;
        };
        Pt ctr;
        bg::set<0>(ctr,(minX+maxX)/2);
        bg::set<1>(ctr,(minY+maxY)/2);

        auto dist2=[&](int i){
            auto &q=pts[i];
            double dx=bg::get<0>(q)-bg::get<0>(ctr),
                   dy=bg::get<1>(q)-bg::get<1>(ctr);
            return dx*dx+dy*dy;
        };

        vector<int> idx;
        for(int i=0;i<(int)pts.size();++i)
            if(onEdge(pts[i])) idx.push_back(i);

        int st=0;
        if(!idx.empty()){
            st = *min_element(idx.begin(),idx.end(),
                [&](int a,int b){ return dist2(a)<dist2(b); });
        } else {
            vector<int> all(pts.size());
            iota(all.begin(),all.end(),0);
            st = *min_element(all.begin(),all.end(),
                [&](int a,int b){ return dist2(a)<dist2(b); });
        }

        vector<char> used(pts.size());
        vector<Pt> ord;
        ord.reserve(pts.size());
        ord.push_back(pts[st]);
        used[st]=1;
        int cur=st;
        for(size_t k=1;k<pts.size();++k){
            double best=numeric_limits<double>::max();
            int nxt=-1;
            for(int i=0;i<(int)pts.size();++i) if(!used[i]){
                double d = fabs(bg::get<0>(pts[cur])-bg::get<0>(pts[i]))
                         + fabs(bg::get<1>(pts[cur])-bg::get<1>(pts[i]));
                if(d<best){ best=d; nxt=i; }
            }
            cur=nxt;
            ord.push_back(pts[cur]);
            used[cur]=1;
        }
        if(!onEdge(ord.front()) && onEdge(ord.back()))
            reverse(ord.begin(),ord.end());

        nets[name] = LS(ord.begin(),ord.end());
        name.clear();
    };

    while(getline(fin,line)){
        auto t=trim(line);
        if(t.rfind("(net",0)==0){
            flush();
            istringstream is(t); string tmp;
            is>>tmp>>name;
            if(!name.empty()&&name.back()==')') name.pop_back();
        }
        else if(t.rfind("(path",0)==0){
            depth=1;
        }
        else if(depth>0){
            depth += countChar(t,'(')-countChar(t,')');
            istringstream is(t);
            double x,y;
            if(is>>x>>y) buf.emplace_back(x,y);
        }
    }
    flush();
    return nets;
}

// 4. 配對
vector<pair<string,vector<string>>> assignByBallEndNearest(
    const map<string,PB> &pbs,
    const LSM &nets
){
    const int CAP=2;
    unordered_map<string,int> cnt;
    vector<pair<string,vector<string>>> res;
    res.reserve(pbs.size());

    for(auto &kv:pbs){
        string pid=kv.first;
        Pt    bp=kv.second.ball;
        vector<pair<double,string>> ds;
        ds.reserve(nets.size());

        for(auto &nt:nets){
            Pt ed = nt.second.back();
            double dx=bg::get<0>(ed)-bg::get<0>(bp),
                   dy=bg::get<1>(ed)-bg::get<1>(bp);
            ds.emplace_back(dx*dx+dy*dy, nt.first);
        }
        sort(ds.begin(), ds.end());

        vector<string> sel;
        for(auto &pr:ds){
            if(cnt[pr.second]<CAP){
                sel.push_back(pr.second);
                cnt[pr.second]++;
            }
            if(sel.size()==2) break;
        }
        // fallback
        if(sel.size()<2){
            for(auto &pr:ds){
                if(find(sel.begin(),sel.end(),pr.second)!=sel.end()) continue;
                sel.push_back(pr.second);
                if(sel.size()==2) break;
            }
        }
        res.emplace_back(pid,sel);
    }
    return res;
}

// 5. drawScene
void drawScene(
    const LS &l1,
    const LS &l2,
    const PB &pb,
    const Rect &ballRect,
    const Rect &fullRect,
    const SESInfo &inf
){
    Rect br=ballRect, fr=fullRect;
    br.xmin*=inf.res; br.xmax*=inf.res;
    br.ymin*=inf.res; br.ymax*=inf.res;
    fr.xmin*=inf.res; fr.xmax*=inf.res;
    fr.ymin*=inf.res; fr.ymax*=inf.res;
    const double eps=10.0;

    auto project=[&](const LS &ls){
        Pt ed = ls.back();
        double x=bg::get<0>(ed), y=bg::get<1>(ed);
        Pt q = ed;
        if(fabs(x-br.xmin)<=eps)      bg::set<0>(q, fr.xmin);
        else if(fabs(x-br.xmax)<=eps) bg::set<0>(q, fr.xmax);
        else if(fabs(y-br.ymin)<=eps) bg::set<1>(q, fr.ymin);
        else if(fabs(y-br.ymax)<=eps) bg::set<1>(q, fr.ymax);
        else return;
        
		cout<<"_view.drawLine("
            <<x<<","<<y<<","
            <<bg::get<0>(q)<<","<<bg::get<1>(q)<<");\n";
		
    };
    project(l1);
    project(l2);

    cout<<"_view.drawArc("<<bg::get<0>(pb.pad)<<","<<bg::get<1>(pb.pad)<<",100000);\n";
    cout<<"_view.drawArc("<<bg::get<0>(pb.ball)<<","<<bg::get<1>(pb.ball)<<",100000);\n";

    auto drawLines=[&](const LS &ls){
        for(size_t i=0;i+1<ls.size();++i){
            auto &p1=ls[i], &p2=ls[i+1];
            cout<<"_view.drawLine("
                <<bg::get<0>(p1)<<","<<bg::get<1>(p1)<<","
                <<bg::get<0>(p2)<<","<<bg::get<1>(p2)<<");\n";
        }
    };
    drawLines(l1);
    drawLines(l2);
    cout<<"\n";
}

// applyLift：一次可以接多個 pad，只要任一個落在線段上就走 A 分支
void applyLift(LS &ls,
               const vector<Pt> &pads,
               const Rect &dieRect,
               double res)
{
    const double EPS = 1e-6;
    const double K   = 10.0;

    if (ls.size() < 3) return;

    Pt A = ls[0], B = ls[1];
    double Ax = bg::get<0>(A), Ay = bg::get<1>(A);
    double Bx = bg::get<0>(B), By = bg::get<1>(B);

    Side sA = whichSide(A, dieRect);
    if (sA == NONE) return;

    if ((sA==TOP||sA==BOTTOM) && fabs(Ay-By)>EPS) return;
    if ((sA==LEFT||sA==RIGHT)  && fabs(Ax-Bx)>EPS) return;

    // 新：對所有 pads 做 OR，只要有 one covered_by 就算壓 pad
    Seg segAB(A,B);
    bool padOnAB = false;
    for (auto &pad : pads) {
        if (bg::covered_by(pad, segAB)) {
            padOnAB = true;
            break;
        }
    }

    if (padOnAB) {
        // —— A 分支：貼邊＋壓 Pad
        double delta = K * res;
        Pt Ap=A, Bp=B;
        if (sA==TOP||sA==BOTTOM) {
            double dir = (sA==TOP?+1:-1);
            bg::set<1>(Ap, Ay+dir*delta);
            bg::set<1>(Bp, By+dir*delta);
        } else {
            double dir = (sA==RIGHT?+1:-1);
            bg::set<0>(Ap, Ax+dir*delta);
            bg::set<0>(Bp, Bx+dir*delta);
        }
        cerr << "[applyLift] A'= ("<<bg::get<0>(Ap)<<","<<bg::get<1>(Ap)
             <<") B'= ("<<bg::get<0>(Bp)<<","<<bg::get<1>(Bp)<<")\n";
        /*
		cout << "_view.drawLine("
             << bg::get<0>(Ap)<<","<<bg::get<1>(Ap)<<"," 
             << bg::get<0>(Bp)<<","<<bg::get<1>(Bp)<<");\n";
		*/
        double dx = bg::get<0>(Bp) - bg::get<0>(Ap);
        double dy = bg::get<1>(Bp) - bg::get<1>(Ap);
        double L  = hypot(dx, dy);
        double span = 2.0 * hypot(dieRect.xmax - dieRect.xmin,
                                 dieRect.ymax - dieRect.ymin);
        double ux = dx / L, uy = dy / L;
        Pt far1, far2;
        bg::set<0>(far1, bg::get<0>(Ap) - ux*span);
        bg::set<1>(far1, bg::get<1>(Ap) - uy*span);
        bg::set<0>(far2, bg::get<0>(Ap) + ux*span);
        bg::set<1>(far2, bg::get<1>(Ap) + uy*span);
        Seg extSeg(far1, far2);
        Pt Bpp = Bp;
        for (size_t i=0; i+1<ls.size(); ++i) {
            Seg seg(ls[i], ls[i+1]);
            vector<Pt> tmp;
            bg::intersection(extSeg, seg, tmp);
            if (!tmp.empty()) {
                Bpp = tmp.front();
                if (sA == TOP || sA == BOTTOM)
                    bg::set<1>(Bpp, bg::get<1>(Ap));
                else
                    bg::set<0>(Bpp, bg::get<0>(Ap));
                break;
            }
        }
        vector<Pt> nls;
        nls.reserve(ls.size()+2);
        nls.push_back(A);
        nls.push_back(Ap);
        nls.push_back(Bpp);
        for (size_t i=2; i<ls.size(); ++i)
            nls.push_back(ls[i]);
        ls.swap(nls);
        return;
    }

    // 分支 B：貼邊但不壓 Pad
	int cnt = 0;
    bool fullOverlap = false;
    if (sA==TOP||sA==BOTTOM) {
        double lo = min(Ax, Bx), hi = max(Ax, Bx);
        if (lo >= dieRect.xmin - EPS && hi <= dieRect.xmax + EPS)
            fullOverlap = true;
    } else {
        double lo = min(Ay, By), hi = max(Ay, By);
        if (lo >= dieRect.ymin - EPS && hi <= dieRect.ymax + EPS)
            fullOverlap = true;
    }
    if (fullOverlap) {
        ls.erase(ls.begin());
    //cerr<<" full covered pads loc: "<< bg::get<0>(pad)<<","<< bg::get<1>(pad)<< "\n";
        return;
    }
    {
        Pt E1, E2;
        switch(sA) {
        case TOP:    E1 = Pt(dieRect.xmin, dieRect.ymax), E2 = Pt(dieRect.xmax, dieRect.ymax); break;
        case BOTTOM: E1 = Pt(dieRect.xmin, dieRect.ymin), E2 = Pt(dieRect.xmax, dieRect.ymin); break;
        case LEFT:   E1 = Pt(dieRect.xmin, dieRect.ymin), E2 = Pt(dieRect.xmin, dieRect.ymax); break;
        case RIGHT:  E1 = Pt(dieRect.xmax, dieRect.ymin), E2 = Pt(dieRect.xmax, dieRect.ymax); break;
        default: break;
        }
        Seg brd(E1, E2), ab(A, B);
        vector<Pt> tmp;
        bg::intersection(ab, brd, tmp);
        if (!tmp.empty()) ls[0] = tmp.front();
    //cerr<<" partical covered pads loc: "<< bg::get<0>(pad)<<","<< bg::get<1>(pad)<< "\n";
        return;
    }

    // 分支 C：其餘不動
}




// 7. buildAndPrintPolygon (unchanged)
void buildAndPrintPolygon(LS raw0,LS raw1,const Rect &dieRect,const Rect &ballRect,const Rect &fullRect,const SESInfo &inf,int idx) {
    Rect br=ballRect, fr=fullRect;
    br.xmin*=inf.res; br.xmax*=inf.res; br.ymin*=inf.res; br.ymax*=inf.res;
    fr.xmin*=inf.res; fr.xmax*=inf.res; fr.ymin*=inf.res; fr.ymax*=inf.res;
    const double eps=10.0;
    auto wsl=[&](const Pt &p,const Rect &r){ return whichSide(p,r); };
    auto projPt=[&](const Pt &p){ Pt q=p; Side s=wsl(p,br);
        if(s==LEFT) bg::set<0>(q,fr.xmin);
        else if(s==RIGHT) bg::set<0>(q,fr.xmax);
        else if(s==BOTTOM) bg::set<1>(q,fr.ymin);
        else if(s==TOP) bg::set<1>(q,fr.ymax);
        return q;
    };
    Pt A=raw0.front(), B=raw1.front();
    Side siA=wsl(A,dieRect), siB=wsl(B,dieRect);
    Pt Aout=projPt(raw0.back()), Bout=projPt(raw1.back());
    Side soA=wsl(raw0.back(),br), soB=wsl(raw1.back(),br);
    vector<Pt> inner{A}, outer{Aout};
    if(siA!=NONE && siB!=NONE && siA!=siB){ double cx = (siA==LEFT||siB==LEFT?dieRect.xmin:dieRect.xmax);
        double cy = (siA==BOTTOM||siB==BOTTOM?dieRect.ymin:dieRect.ymax);
        inner.push_back(Pt(cx,cy));
    }
    inner.push_back(B);
    if(soA!=NONE && soB!=NONE && soA!=soB){ double cx = (soA==LEFT||soB==LEFT?fr.xmin:fr.xmax);
        double cy = (soA==BOTTOM||soB==BOTTOM?fr.ymin:fr.ymax);
        outer.push_back(Pt(cx,cy));
    }
    outer.push_back(Bout);
    //for(size_t i=0;i+1<inner.size();++i){ auto&p=inner[i],&q=inner[i+1]; cout<<"_view.drawLine("<<bg::get<0>(p)<<","<<bg::get<1>(p)<<","<<bg::get<0>(q)<<","<<bg::get<1>(q)<<");\n"; }
    //for(size_t i=0;i+1<outer.size();++i){ auto&p=outer[i],&q=outer[i+1]; cout<<"_view.drawLine("<<bg::get<0>(p)<<","<<bg::get<1>(p)<<","<<bg::get<0>(q)<<","<<bg::get<1>(q)<<");\n"; }
    vector<Pt> ring;
    ring.insert(ring.end(), inner.begin(), inner.end());
    ring.insert(ring.end(), raw1.begin(), raw1.end());
    ring.insert(ring.end(), outer.rbegin(), outer.rend());
    ring.insert(ring.end(), raw0.rbegin(), raw0.rend());
    ring.push_back(ring.front());
    Polygon poly;
    bg::assign_points(poly, ring);
    bg::correct(poly);
}

int main(int argc,char**argv) {
    if(argc<7){ cerr<<"Usage: ./parse_boundary <ses> <orig> minX minY width height\n"; return 1; }
    SESInfo inf = parseSesInfo(argv[1]);
    double minX = stod(argv[3]), minY = stod(argv[4]);
    double W    = stod(argv[5]), H    = stod(argv[6]);
    double maxX = minX + W, maxY = minY + H;
    Rect dieRect{ minX*inf.res, minY*inf.res, maxX*inf.res, maxY*inf.res };
    map<string,PB> pbs;
    Rect ballRect{}, fullRect{};
    readOriginalCoords(argv[2], inf, pbs, ballRect, fullRect);
    LSM nets = parseSesPaths(
        argv[1],
        minX*inf.res, minY*inf.res,
        maxX*inf.res, maxY*inf.res
    );


    auto pairs = assignByBallEndNearest(pbs, nets);
	
    // 1) 把 pairs 反轉，收集 netName -> pads[]
    map<string, vector<Pt>> net2pads;
    for (auto &pr : pairs) {
        const Pt pad = pbs[pr.first].pad;
        for (auto &netName : pr.second) {
            net2pads[netName].push_back(pad);
        }
    }

    int idx = 0;
    for (auto &pr : pairs) {
        // 原本是 pr.first = padID, pr.second = {net0, net1}
        auto net0 = pr.second[0];
        auto net1 = pr.second[1];
        PB pb = pbs[pr.first];

        // 2) 複製原線，不要就地改 nets[]
        LS ls0 = nets[net0];
        LS ls1 = nets[net1];

        // 3) 傳入所有對應的 pads
        applyLift(ls0, net2pads[net0], dieRect, inf.res);
        applyLift(ls1, net2pads[net1], dieRect, inf.res);

        // 4) 繪圖與多邊形
        drawScene(ls0, ls1, pb, ballRect, fullRect, inf);
        buildAndPrintPolygon(ls0, ls1, dieRect, ballRect, fullRect, inf, idx++);
    }
    return 0;
}