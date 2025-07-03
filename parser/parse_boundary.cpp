// parse_boundary.cpp — 完整可編譯版 V1.32
// 功能：
//   1. 解析 SES (.ses) 取得解析度與偏移
//   2. 從 argv 讀入 Die Rect (minX,minY,width,height)
//   3. 解析 original_coords 取得 Pad/Ball 座標、Ball Rect 及 Full Rect
//   4. 解析所有網路路徑，只保留穿過 Die Rect 邊緣的折線
//   5. 配對：以 ball 與 net 終點最近二條 (cap=2)
//   6. 輸出 NetGroup 資訊
//   7. 繪圖延伸：drawScene 輸出 _view.drawArc/_view.drawLine
//   8. 建立多邊形並輸出 WKT
//   9. 輸出 inner/outer boundary 繪圖命令

#include <bits/stdc++.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
using namespace std;
namespace bg = boost::geometry;

using Pt      = bg::model::point<long long,2,bg::cs::cartesian>;
using LS      = bg::model::linestring<Pt>;
using LSM     = map<string,LS>;
using Polygon = bg::model::polygon<Pt>;

struct PB   { Pt pad, ball; };
struct Rect { long long xmin,ymin,xmax,ymax; };

static string trim(const string &s){
    auto a=s.find_first_not_of(" \t\r\n"),
         b=s.find_last_not_of(" \t\r\n");
    return (a==string::npos)?string():s.substr(a,b-a+1);
}
static int countChar(const string &s,char c){
    return int(count(s.begin(),s.end(),c));
}

enum Side { NONE, LEFT, RIGHT, BOTTOM, TOP };
Side whichSide(const Pt &p,const Rect &r,long long eps){
    long long x=bg::get<0>(p), y=bg::get<1>(p);
    if(llabs(x-r.xmin)<=eps) return LEFT;
    if(llabs(x-r.xmax)<=eps) return RIGHT;
    if(llabs(y-r.ymin)<=eps) return BOTTOM;
    if(llabs(y-r.ymax)<=eps) return TOP;
    return NONE;
}

// 1. 解析 SES header
struct SESInfo { long long res=1, offX=0, offY=0; };
SESInfo parseSesInfo(const string &file){
    SESInfo inf;
    ifstream fin(file); if(!fin) throw runtime_error("open SES fail");
    string line;
    while(getline(fin,line)){
        auto t=trim(line);
        if(t.rfind("(resolution",0)==0){
            istringstream is(t); string tag,unit;
            is>>tag>>unit>>inf.res;
        } else if(t.rfind("(place",0)==0){
            istringstream is(t);
            string plc,comp; long long x,y;
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
                   "Ball Rect: xmin=%lld ymin=%lld xmax=%lld ymax=%lld",
                   &ballRect.xmin,&ballRect.ymin,
                   &ballRect.xmax,&ballRect.ymax);
        }
        else if(t.rfind("Full Rect:",0)==0){
            sscanf(t.c_str(),
                   "Full Rect: xmin=%lld ymin=%lld xmax=%lld ymax=%lld",
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
            bg::set<0>(b.pad, llround(x*inf.res) + inf.offX);
            bg::set<1>(b.pad, llround(y*inf.res) + inf.offY);
        }
        else if(t.rfind("Ball_location",0)==0){
            double x,y;
            sscanf(line.c_str()+line.find('(')+1,"%lf,%lf",&x,&y);
            PB &b = pbs[cur];
            bg::set<0>(b.ball, llround(x*inf.res));
            bg::set<1>(b.ball, llround(y*inf.res));
        }
    }
}

// 3. parse SES paths
LSM parseSesPaths(
    const string &file,
    long long minX,long long minY,
    long long maxX,long long maxY
){
    ifstream fin(file); if(!fin) throw runtime_error("open SES paths fail");
    LSM nets;
    string line,name;
    int depth=0;
    vector<Pt> buf;
    const long long eps=10;

    auto flush=[&](){
        if(name.empty()||buf.size()<2){ buf.clear(); name.clear(); return; }
        vector<Pt> pts;
        for(auto &p:buf)
            if(pts.empty()|| bg::distance(p,pts.back())>0)
                pts.push_back(p);
        buf.clear();
        if(pts.size()<2){ name.clear(); return; }

        auto onEdge=[&](const Pt &p){
            return llabs(bg::get<0>(p)-minX)<=eps
                || llabs(bg::get<0>(p)-maxX)<=eps
                || llabs(bg::get<1>(p)-minY)<=eps
                || llabs(bg::get<1>(p)-maxY)<=eps;
        };
        Pt ctr;
        bg::set<0>(ctr,(minX+maxX)/2);
        bg::set<1>(ctr,(minY+maxY)/2);
        auto dist2=[&](int i){
            auto &q=pts[i];
            long long dx=bg::get<0>(q)-bg::get<0>(ctr),
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
            long long best=LLONG_MAX;
            int nxt=-1;
            for(int i=0;i<(int)pts.size();++i) if(!used[i]){
                long long d = llabs(bg::get<0>(pts[cur])-bg::get<0>(pts[i]))
                            + llabs(bg::get<1>(pts[cur])-bg::get<1>(pts[i]));
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
            long long x,y;
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
        vector<pair<long long,string>> ds;
        ds.reserve(nets.size());

        for(auto &nt:nets){
            Pt ed = nt.second.back();
            long long dx=bg::get<0>(ed)-bg::get<0>(bp),
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
    // SES 單位轉換
    Rect br=ballRect, fr=fullRect;
    br.xmin*=inf.res; br.xmax*=inf.res;
    br.ymin*=inf.res; br.ymax*=inf.res;
    fr.xmin*=inf.res; fr.xmax*=inf.res;
    fr.ymin*=inf.res; fr.ymax*=inf.res;
    const long long eps=10;

    auto project=[&](const LS &ls){
        Pt ed = ls.back();
        long long x=bg::get<0>(ed), y=bg::get<1>(ed);
        Pt q = ed;
        // 延伸到 FullRect 並畫出
        if(llabs(x-br.xmin)<=eps)      bg::set<0>(q, fr.xmin);
        else if(llabs(x-br.xmax)<=eps) bg::set<0>(q, fr.xmax);
        else if(llabs(y-br.ymin)<=eps) bg::set<1>(q, fr.ymin);
        else if(llabs(y-br.ymax)<=eps) bg::set<1>(q, fr.ymax);
        else return;
        cout<<"_view.drawLine("
            <<x<<","<<y<<","
            <<bg::get<0>(q)<<","<<bg::get<1>(q)<<");\n";
    };
    project(l1);
    project(l2);
	/*
    //59,500io
	cout<<"_view.drawArc("<<bg::get<0>(pb.pad)<<","<<bg::get<1>(pb.pad)<<",20000);\n";
    cout<<"_view.drawArc("<<bg::get<0>(pb.ball)<<","<<bg::get<1>(pb.ball)<<",20000);\n";
	*/
	//else
	
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

// 6-9. 建立多邊形並輸出 inner/outer
void buildAndPrintPolygon(
    LS raw0, LS raw1,
    const Rect &dieRect,
    const Rect &ballRect,
    const Rect &fullRect,
    const SESInfo &inf,
    int idx
){
    // SES 單位轉換
    Rect dr=dieRect, br=ballRect, fr=fullRect;

    br.xmin*=inf.res; br.xmax*=inf.res;
    br.ymin*=inf.res; br.ymax*=inf.res;
    fr.xmin*=inf.res; fr.xmax*=inf.res;
    fr.ymin*=inf.res; fr.ymax*=inf.res;
    const long long eps=10;

    // 投影點計算 (BallRect→FullRect)
    auto projectPt=[&](const Pt &p){
        Pt q=p; Side s=whichSide(p,br,eps);
        if(s==LEFT)    bg::set<0>(q,fr.xmin);
        else if(s==RIGHT) bg::set<0>(q,fr.xmax);
        else if(s==BOTTOM)bg::set<1>(q,fr.ymin);
        else if(s==TOP)   bg::set<1>(q,fr.ymax);
        return q;
    };

    // 內側 (DieRect)
    Pt A = raw0.front(), B = raw1.front();
    Side sAi = whichSide(A, dr, eps), sBi = whichSide(B, dr, eps);

    // 外側 (BallRect)
    Pt Aout = projectPt(raw0.back()),
       Bout = projectPt(raw1.back());
    Side sAo = whichSide(raw0.back(), br, eps),
         sBo = whichSide(raw1.back(), br, eps);

    vector<Pt> inner{A}, outer{Aout};

    // 插 inner 角點
    if(sAi!=NONE && sBi!=NONE && sAi!=sBi){
        long long cx = (sAi==LEFT||sBi==LEFT ? dr.xmin : dr.xmax);
        long long cy = (sAi==BOTTOM||sBi==BOTTOM ? dr.ymin : dr.ymax);
        inner.push_back(Pt(cx,cy));
    }
    inner.push_back(B);

    // 插 outer 角點
    if(sAo!=NONE && sBo!=NONE && sAo!=sBo){
        long long ocx = (sAo==LEFT||sBo==LEFT ? fr.xmin : fr.xmax);
        long long ocy = (sAo==BOTTOM||sBo==BOTTOM ? fr.ymin : fr.ymax);
        outer.push_back(Pt(ocx,ocy));
    }
    outer.push_back(Bout);

    // 9. 輸出 inner boundary
    for(size_t i=0;i+1<inner.size();++i){
        auto &p1=inner[i], &p2=inner[i+1];
        cout<<"_view.drawLine("
            <<bg::get<0>(p1)<<","<<bg::get<1>(p1)<<","
            <<bg::get<0>(p2)<<","<<bg::get<1>(p2)<<");\n";
    }
    // 9. 輸出 outer boundary
    for(size_t i=0;i+1<outer.size();++i){
        auto &p1=outer[i], &p2=outer[i+1];
        cout<<"_view.drawLine("
            <<bg::get<0>(p1)<<","<<bg::get<1>(p1)<<","
            <<bg::get<0>(p2)<<","<<bg::get<1>(p2)<<");\n";
    }

    // build ring
    vector<Pt> ring;
    ring.insert(ring.end(), inner.begin(), inner.end());
    ring.insert(ring.end(), raw1.begin(), raw1.end());
    ring.insert(ring.end(), outer.rbegin(), outer.rend());
    ring.insert(ring.end(), raw0.rbegin(), raw0.rend());
    ring.push_back(ring.front());

    Polygon poly;
    bg::assign_points(poly, ring);
    bg::correct(poly);
   // cout<<"Polygon "<<idx<<": "<<bg::wkt(poly)<<"\n\n";
}

int main(int argc,char**argv){
    if(argc<7){
        cerr<<"Usage: ./parse_boundary <ses> <orig> minX minY width height\n";
        return 1;
    }
    string ses=argv[1], orig=argv[2];
    long long minX=stoll(argv[3]), minY=stoll(argv[4]);
    long long W=stoll(argv[5]), H=stoll(argv[6]);
    long long maxX=minX+W, maxY=minY+H;

    // SES header
    SESInfo inf = parseSesInfo(ses);

    // Die Rect
    Rect dieRect{minX*inf.res, minY*inf.res, maxX*inf.res, maxY*inf.res};

    // Pad/Ball + Rects
    map<string,PB> pbs;
    Rect ballRect{}, fullRect{};
    readOriginalCoords(orig, inf, pbs, ballRect, fullRect);

    // SES paths
    LSM nets = parseSesPaths(
        ses,
        minX*inf.res, minY*inf.res,
        maxX*inf.res, maxY*inf.res
    );

    // 配對
    auto pairs = assignByBallEndNearest(pbs, nets);

    int idx=0;
    for(auto &pr: pairs){
        const string &id = pr.first;
        auto &bds = pr.second;
        PB pb = pbs[id];
/*
        cout<<"=== NetGroup "<<idx<<" ("<<id<<") ===\n";
        cout<<"Pad:  "<<bg::get<0>(pb.pad)<<","<<bg::get<1>(pb.pad)<<"\n";
        cout<<"Ball: "<<bg::get<0>(pb.ball)<<","<<bg::get<1>(pb.ball)<<"\n";
        cout<<"Boundary0="<<bds[0]<<", Boundary1="<<bds[1]<<"\n\n";
*/
        drawScene(nets[bds[0]], nets[bds[1]], pb, ballRect, fullRect, inf);
        buildAndPrintPolygon(
            nets[bds[0]],
            nets[bds[1]],
            dieRect,
            ballRect,
            fullRect,
            inf,
            idx
        );
        idx++;
    }
    return 0;
}
