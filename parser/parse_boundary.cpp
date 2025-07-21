// parse_boundary.cpp — 統合新版 applyLift
#include <fstream>
#include <iomanip>
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

	/*
    //59,500io
	cout<<"_view.drawArc("<<bg::get<0>(pb.pad)<<","<<bg::get<1>(pb.pad)<<",20000);\n";
    cout<<"_view.drawArc("<<bg::get<0>(pb.ball)<<","<<bg::get<1>(pb.ball)<<",20000);\n";
	*/
	//else
	
	cout<<"_view.drawArc("<<bg::get<0>(pb.pad)<<","<<bg::get<1>(pb.pad)<<",100000);\n";
    cout<<"_view.drawArc("<<bg::get<0>(pb.ball)<<","<<bg::get<1>(pb.ball)<<",100000);\n";
	
    project(l1);
    project(l2);
	
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
    //cout<<"\n";
}

//-----------------------------------------------------------------------------
// applyLift：接受多個 pad，只要任一個 pad 符合壓在線段上，即走 A 分支
void applyLift(LS &ls,
               const vector<Pt> &pads,
               const Rect &dieRect,
               double res)
{
    const double EPS = 1e-6;
    const double K   = 10.0;

    // 0) 至少有 A,B,C 三個點
    if (ls.size() < 3) return;

    // 1) 取 A,B
    Pt A = ls[0], B = ls[1];
    double Ax = bg::get<0>(A), Ay = bg::get<1>(A);
    double Bx = bg::get<0>(B), By = bg::get<1>(B);

    // 2) 判斷 A 在哪條邊
    Side sA = whichSide(A, dieRect);
    if (sA == NONE) return;

    // 3) 檢查 AB 是否平行
    if ((sA==TOP||sA==BOTTOM) && fabs(Ay-By) > EPS) return;
    if ((sA==LEFT||sA==RIGHT) && fabs(Ax-Bx) > EPS) return;

    // 4) 多 pad 覆蓋判斷
    Seg segAB(A, B);
    bool padOnAB = false;
    for (auto &pad : pads) {
        if (bg::covered_by(pad, segAB)) { padOnAB = true; break; }
    }

    // A 分支：貼邊＋壓 Pad → 抬高
    if (padOnAB) {
        double delta = K * res;
        Pt Ap = A, Bp = B;
        if (sA==TOP||sA==BOTTOM) {
            double dir = (sA==TOP?+1:-1);
            bg::set<1>(Ap, Ay + dir*delta);
            bg::set<1>(Bp, By + dir*delta);
        } else {
            double dir = (sA==RIGHT?+1:-1);
            bg::set<0>(Ap, Ax + dir*delta);
            bg::set<0>(Bp, Bx + dir*delta);
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

    // B 分支：貼邊但不壓 Pad
    // 先判 full overlap
    bool fullOv = false;
    if (sA==TOP||sA==BOTTOM) {
        double lo=min(Ax,Bx), hi=max(Ax,Bx);
        if (lo>=dieRect.xmin-EPS && hi<=dieRect.xmax+EPS) fullOv=true;
    } else {
        double lo=min(Ay,By), hi=max(Ay,By);
        if (lo>=dieRect.ymin-EPS && hi<=dieRect.ymax+EPS) fullOv=true;
    }
    if (fullOv) {
        ls.erase(ls.begin());
		//cerr<<"full overlap!\n";
        return;
    }

    // 部分重疊：clamp B
    Pt corner;
    switch (sA) {
        case TOP:
            bg::set<0>(corner, std::clamp(Bx, dieRect.xmin, dieRect.xmax));
            bg::set<1>(corner, dieRect.ymax);
            break;
        case BOTTOM:
            bg::set<0>(corner, std::clamp(Bx, dieRect.xmin, dieRect.xmax));
            bg::set<1>(corner, dieRect.ymin);
            break;
        case LEFT:
            bg::set<0>(corner, dieRect.xmin);
            bg::set<1>(corner, std::clamp(By, dieRect.ymin, dieRect.ymax));
            break;
        case RIGHT:
            bg::set<0>(corner, dieRect.xmax);
            bg::set<1>(corner, std::clamp(By, dieRect.ymin, dieRect.ymax));
            break;
        default:
            return;
    }
    ls[0] = corner;
	//cerr<<"partical overlap!\n";
    return;

    // C 分支：其餘不動
}





// 生成多邊形，並把 raw0/raw1 延伸到 FullRect 尾端
Polygon buildPolygon(
    LS &raw0, LS &raw1,
    const Rect &dieRect,
    const Rect &ballRect,
    const Rect &fullRect,
    const SESInfo &inf
) {
    // 同 buildAndPrintPolygon 前半，計算延伸點
    Rect br = ballRect, fr = fullRect;
    br.xmin *= inf.res; br.xmax *= inf.res; br.ymin *= inf.res; br.ymax *= inf.res;
    fr.xmin *= inf.res; fr.xmax *= inf.res; fr.ymin *= inf.res; fr.ymax *= inf.res;
    auto whichSideLocal = [&](const Pt &p, const Rect &r) { return whichSide(p,r); };
    auto projectPt = [&](const Pt &p){ Pt q=p; Side s=whichSideLocal(p,br);
        if(s==LEFT)      bg::set<0>(q,fr.xmin);
        else if(s==RIGHT)bg::set<0>(q,fr.xmax);
        else if(s==BOTTOM)bg::set<1>(q,fr.ymin);
        else if(s==TOP)   bg::set<1>(q,fr.ymax);
        return q;
    };
    // 延伸 raw0/raw1
    Pt Aout = projectPt(raw0.back()); raw0.push_back(Aout);
    Pt Bout = projectPt(raw1.back()); raw1.push_back(Bout);
    // 構造 inner/outer
    vector<Pt> inner; vector<Pt> outer;
    Pt A = raw0.front(), B = raw1.front();
    Side siA = whichSideLocal(A, dieRect), siB = whichSideLocal(B, dieRect);
    inner.push_back(A);
    if(siA!=NONE && siB!=NONE && siA!=siB){
        double cx = (siA==LEFT||siB==LEFT? dieRect.xmin : dieRect.xmax);
        double cy = (siA==BOTTOM||siB==BOTTOM? dieRect.ymin: dieRect.ymax);
        inner.push_back(Pt(cx,cy));
    }
    inner.push_back(B);
    Side soA = whichSideLocal(raw0.back(), br), soB = whichSideLocal(raw1.back(), br);
    outer.push_back(Aout);
    if(soA!=NONE && soB!=NONE && soA!=soB){
        double cx = (soA==LEFT||soB==LEFT? fr.xmin : fr.xmax);
        double cy = (soA==BOTTOM||soB==BOTTOM? fr.ymin: fr.ymax);
        outer.push_back(Pt(cx,cy));
    }
    outer.push_back(Bout);
    // 拼 ring
    vector<Pt> ring;
    ring.insert(ring.end(), inner.begin(), inner.end());
    ring.insert(ring.end(), raw1.begin(), raw1.end());
    ring.insert(ring.end(), outer.rbegin(), outer.rend());
    ring.insert(ring.end(), raw0.rbegin(), raw0.rend());
    ring.push_back(ring.front());
    // build poly
    Polygon poly;
    bg::assign_points(poly, ring);
    bg::correct(poly); 
	if(!(bg::is_valid(poly))){
		cerr << "Poly is not valid!\n";
	}
    return poly;
}
// 列印多邊形邊界 (inner + outer 折線)
void printPolygon(
    LS raw0, LS raw1,
    const Rect &dieRect,
    const Rect &ballRect,
    const Rect &fullRect,
    const SESInfo &inf
) {
    Rect br = ballRect, fr = fullRect;
    br.xmin *= inf.res; br.xmax *= inf.res; br.ymin *= inf.res; br.ymax *= inf.res;
    fr.xmin *= inf.res; fr.xmax *= inf.res; fr.ymin *= inf.res; fr.ymax *= inf.res;
    auto whichSideLocal = [&](const Pt &p, const Rect &r) { return whichSide(p,r); };
    // 頭尾與延伸已在 raw0/raw1
    Pt A = raw0.front(), B = raw1.front();
    Side siA = whichSideLocal(A,dieRect), siB = whichSideLocal(B,dieRect);
    Pt Aout = raw0.back(), Bout = raw1.back();
    // inner
    vector<Pt> inner{A};
    if(siA!=NONE && siB!=NONE && siA!=siB){
        double cx = (siA==LEFT||siB==LEFT? dieRect.xmin : dieRect.xmax);
        double cy = (siA==BOTTOM||siB==BOTTOM? dieRect.ymin: dieRect.ymax);
        inner.push_back(Pt(cx,cy));
    }
    inner.push_back(B);
    // outer
    Side soA = whichSideLocal(Aout, fr), soB = whichSideLocal(Bout, fr);
    vector<Pt> outer{Aout};
    if(soA!=NONE && soB!=NONE && soA!=soB){
        double cx = (soA==LEFT||soB==LEFT? fr.xmin : fr.xmax);
        double cy = (soA==BOTTOM||soB==BOTTOM? fr.ymin: fr.ymax);
        outer.push_back(Pt(cx,cy));
    }
    outer.push_back(Bout);
    // print
    for(size_t i=0;i+1<inner.size();++i){ auto&p=inner[i],&q=inner[i+1];
        cout<<"_view.drawLine("<<bg::get<0>(p)<<","<<bg::get<1>(p)<<","<<bg::get<0>(q)<<","<<bg::get<1>(q)<<");\n";
    }
    for(size_t i=0;i+1<outer.size();++i){ auto&p=outer[i],&q=outer[i+1];
        cout<<"_view.drawLine("<<bg::get<0>(p)<<","<<bg::get<1>(p)<<","<<bg::get<0>(q)<<","<<bg::get<1>(q)<<");\n";
    }
}

// 檢查多邊形是否覆蓋 pad 與 ball
bool checkPolygonContainsPadBall(
    const Polygon &poly,
    const PB &pb
) {
    return bg::covered_by(pb.pad,  poly)
        && bg::covered_by(pb.ball, poly);
}
// -----------------------------------------------------------------------------
// 按 whichSide 将 NetGroup 分到 TOP/RIGHT/BOTTOM/LEFT 四组，
// 并在组内按“左→右”、“上→下”、“右→左”、“下→上”排序，最后拼接成一份扁平列表
vector<pair<string, vector<string>>> reorderBySide(
    const vector<pair<string, vector<string>>> &pairs,
    const map<string, PB>               &pbs,
    const Rect                         &dieRect
) {
    // 四组临时容器
    vector<pair<string, vector<string>>> topV, rightV, bottomV, leftV, others;

    // 1) 分类
    for (auto &pr : pairs) {
        const Pt pad = pbs.at(pr.first).pad;
        switch (whichSide(pad, dieRect)) {
            case TOP:    topV.push_back(pr);    break;
            case RIGHT:  rightV.push_back(pr);  break;
            case BOTTOM: bottomV.push_back(pr); break;
            case LEFT:   leftV.push_back(pr);   break;
            default:     others.push_back(pr);  break;
        }
    }

    // 2) 组内排序
    auto cmpXasc  = [&](auto &a, auto &b){ return bg::get<0>(pbs.at(a.first).pad)
                                                   < bg::get<0>(pbs.at(b.first).pad); };
    auto cmpXdesc = [&](auto &a, auto &b){ return bg::get<0>(pbs.at(a.first).pad)
                                                   > bg::get<0>(pbs.at(b.first).pad); };
    auto cmpYasc  = [&](auto &a, auto &b){ return bg::get<1>(pbs.at(a.first).pad)
                                                   < bg::get<1>(pbs.at(b.first).pad); };
    auto cmpYdesc = [&](auto &a, auto &b){ return bg::get<1>(pbs.at(a.first).pad)
                                                   > bg::get<1>(pbs.at(b.first).pad); };

    sort(topV.begin(),    topV.end(),    cmpXasc);
    sort(rightV.begin(),  rightV.end(),  cmpYdesc);
    sort(bottomV.begin(), bottomV.end(), cmpXdesc);
    sort(leftV.begin(),   leftV.end(),   cmpYasc);

    // 3) 扁平化输出顺序
    vector<pair<string, vector<string>>> ordered;
    ordered.reserve(pairs.size());
    for (auto &x : topV)    ordered.push_back(x);
    for (auto &x : rightV)  ordered.push_back(x);
    for (auto &x : bottomV) ordered.push_back(x);
    for (auto &x : leftV)   ordered.push_back(x);
    for (auto &x : others)  ordered.push_back(x);
    return ordered;
}

// -----------------------------------------------------------------------------
void outputNetInfo(
    ofstream                                 &fout,
    int                                       IDcnt,
    const PB                                 &pb,
    const vector<LS>                         &boundaries,   // 已 applyLift 的 ls0, ls1
    const Rect                               &dieRect,
    const Rect                               &ballRect,
    const Rect                               &fullRect,
    const SESInfo                            &inf,
    const Polygon                            &poly
) {
    fout << fixed << scientific << setprecision(6);

    // 1) Net header + pad/ball
    fout << "Net" << IDcnt << ":\n";
    fout << "pad_location:  "
         << bg::get<0>(pb.pad) << " " << bg::get<1>(pb.pad) << "\n";
    fout << "ball_location: "
         << bg::get<0>(pb.ball)<< " " << bg::get<1>(pb.ball)<< "\n";

    // 2) Initial / Extended (empty)
    fout << "Initial_route_segment:\n";
    fout << "Extended_Initial_route_segment:\n";

    // 3) Boundary segments
    for (size_t i = 0; i < boundaries.size(); ++i) {
        const LS &ls = boundaries[i];
        fout << "Boundary_segment_" << i << "_info_start:\n";
        fout << "Boundary_" << i << "_startPoint: "
             << bg::get<0>(ls.front()) << " " << bg::get<1>(ls.front())
             << "  Boundary_" << i << "_endPoint: "
             << bg::get<0>(ls.back())  << " " << bg::get<1>(ls.back()) << "\n";
        fout << "Boundary_" << i << "_segment:";
        for (auto &pt : ls) {
            fout << " " << bg::get<0>(pt) << " " << bg::get<1>(pt);
        }
        fout << "\n";
    }

    // 4) InnerBoundary (1 or 2 segments)
    {
        auto whichSideLocal = [&](const Pt &p, const Rect &r){ return whichSide(p,r); };
        Pt A = boundaries[0].front(), B = boundaries[1].front();
        Side sA = whichSideLocal(A, dieRect), sB = whichSideLocal(B, dieRect);

        vector<Pt> pts{A};
        if (sA!=NONE && sB!=NONE && sA!=sB) {
            double cx = (sA==LEFT||sB==LEFT ? dieRect.xmin : dieRect.xmax);
            double cy = (sA==BOTTOM||sB==BOTTOM ? dieRect.ymin : dieRect.ymax);
            pts.push_back(Pt(cx, cy));
        }
        pts.push_back(B);

        for (size_t j = 0; j + 1 < pts.size(); ++j) {
            fout << "InnerBoundary_segment_" << j << "_info_start:\n";
            fout << "InnerBoundary_"<< j <<"_startPoint: "
                 << bg::get<0>(pts[j])<<" "<<bg::get<1>(pts[j])
                 <<"  InnerBoundary_"<< j <<"_endPoint: "
                 << bg::get<0>(pts[j+1])<<" "<<bg::get<1>(pts[j+1])<<"\n";
            fout << "InnerBoundary_"<< j <<"_segment:";
            for (auto &p : pts) {
                fout << " " << bg::get<0>(p) << " " << bg::get<1>(p);
            }
            fout << "\n";
        }
    }

    // 5) OutterBoundary (1 or 2 segments)
    {
        auto whichSideLocal = [&](const Pt &p, const Rect &r){ return whichSide(p,r); };
        Rect br = ballRect, fr = fullRect;
        br.xmin*=inf.res; br.xmax*=inf.res; br.ymin*=inf.res; br.ymax*=inf.res;
        fr.xmin*=inf.res; fr.xmax*=inf.res; fr.ymin*=inf.res; fr.ymax*=inf.res;

        vector<Pt> outs;
        for (auto &ls : boundaries) {
            Pt ed = ls.back(), q = ed;
            Side s = whichSideLocal(ed, fr);
            if      (s==LEFT)   bg::set<0>(q, fr.xmin);
            else if (s==RIGHT)  bg::set<0>(q, fr.xmax);
            else if (s==BOTTOM) bg::set<1>(q, fr.ymin);
            else if (s==TOP)    bg::set<1>(q, fr.ymax);
            outs.push_back(q);
        }

        Side s0 = whichSideLocal(outs[0], fr),
             s1 = whichSideLocal(outs[1], fr);

        vector<Pt> pts{outs[0]};
        if (s0!=NONE && s1!=NONE && s0!=s1) {
            double cx = (s0==LEFT||s1==LEFT ? fr.xmin : fr.xmax);
            double cy = (s0==BOTTOM||s1==BOTTOM ? fr.ymin : fr.ymax);
            pts.push_back(Pt(cx, cy));
        }
        pts.push_back(outs[1]);

        for (size_t k = 0; k + 1 < pts.size(); ++k) {
            fout << "OutterBoundary_segment_"<< k <<"_info_start:\n";
            fout << "OutterBoundary_"<< k <<"_startPoint: "
                 << bg::get<0>(pts[k])<<" "<<bg::get<1>(pts[k])
                 <<"  OutterBoundary_"<< k <<"_endPoint: "
                 << bg::get<0>(pts[k+1])<<" "<<bg::get<1>(pts[k+1])<<"\n";
            fout << "OutterBoundary_"<< k <<"_segment:";
            for (auto &p : pts) {
                fout << " " << bg::get<0>(p) << " " << bg::get<1>(p);
            }
            fout << "\n";
        }
    }

    // 6) Area
    fout << "Net" << IDcnt << "_area: "
         << bg::area(poly) << "\n";
/*
    // 7) Echo draw commands into same fout
    //    Pad & Ball
    fout << "_view.drawArc("
         << bg::get<0>(pb.pad)<<","<<bg::get<1>(pb.pad)<<",20000);\n";
    fout << "_view.drawArc("
         << bg::get<0>(pb.ball)<<","<<bg::get<1>(pb.ball)<<",20000);\n";

    //    Original boundaries
    for (auto &ls : boundaries) {
        for (size_t i = 0; i+1 < ls.size(); ++i) {
            auto &p = ls[i], &q = ls[i+1];
            fout << "_view.drawLine("
                 << bg::get<0>(p)<<","<<bg::get<1>(p)<<","
                 << bg::get<0>(q)<<","<<bg::get<1>(q)<<");\n";
        }
    }
    //    InnerBoundary
    {
        auto whichSideLocal = [&](const Pt &p,const Rect &r){ return whichSide(p,r); };
        Pt A = boundaries[0].front(), B = boundaries[1].front();
        Side sA = whichSideLocal(A, dieRect), sB = whichSideLocal(B, dieRect);
        vector<Pt> pts{A};
        if (sA!=NONE && sB!=NONE && sA!=sB) {
            double cx = (sA==LEFT||sB==LEFT ? dieRect.xmin : dieRect.xmax);
            double cy = (sA==BOTTOM||sB==BOTTOM ? dieRect.ymin : dieRect.ymax);
            pts.push_back(Pt(cx,cy));
        }
        pts.push_back(B);
        for (size_t i = 0; i+1 < pts.size(); ++i) {
            fout << "_view.drawLine("
                 << bg::get<0>(pts[i])<<","<<bg::get<1>(pts[i])<<","
                 << bg::get<0>(pts[i+1])<<","<<bg::get<1>(pts[i+1])<<");\n";
        }
    }
    //    OutterBoundary
    {
        auto whichSideLocal = [&](const Pt &p,const Rect &r){ return whichSide(p,r); };
        Rect br=ballRect, fr=fullRect;
        br.xmin*=inf.res; br.xmax*=inf.res; br.ymin*=inf.res; br.ymax*=inf.res;
        fr.xmin*=inf.res; fr.xmax*=inf.res; fr.ymin*=inf.res; fr.ymax*=inf.res;
        vector<Pt> outs;
        for (auto &ls : boundaries) {
            Pt ed=ls.back(), q=ed;
            Side s=whichSideLocal(ed, fr);
            if      (s==LEFT)   bg::set<0>(q, fr.xmin);
            else if (s==RIGHT)  bg::set<0>(q, fr.xmax);
            else if (s==BOTTOM) bg::set<1>(q, fr.ymin);
            else if (s==TOP)    bg::set<1>(q, fr.ymax);
            outs.push_back(q);
        }
        Side s0=whichSideLocal(outs[0], fr), s1=whichSideLocal(outs[1], fr);
        vector<Pt> pts{outs[0]};
        if (s0!=NONE && s1!=NONE && s0!=s1) {
            double cx=(s0==LEFT||s1==LEFT?fr.xmin:fr.xmax);
            double cy=(s0==BOTTOM||s1==BOTTOM?fr.ymin:fr.ymax);
            pts.push_back(Pt(cx,cy));
        }
        pts.push_back(outs[1]);
        for (size_t i=0;i+1<pts.size();++i){
            fout << "_view.drawLine("
                 << bg::get<0>(pts[i])<<","<<bg::get<1>(pts[i])<<","
                 << bg::get<0>(pts[i+1])<<","<<bg::get<1>(pts[i+1])<<");\n";
        }
    }
*/
}
void exportNetAreasToCSV(
    const std::string &filepath,
    const std::vector<std::pair<int,double>> &netAreas
) {

    std::ofstream fout(filepath);
	fout << std::fixed 
     << std::scientific 
     << std::setprecision(6)  // match your TXT precision
     << std::nouppercase;     // lowercase “e” if you prefer
    if (!fout) {
        std::cerr << "ERROR: cannot open " << filepath << "\n";
        return;
    }
    // CSV 标头
    fout << "NetIndex,Area\n";
    // 每行一个 Net
    for (auto &p : netAreas) {
        fout << p.first << "," << p.second << "\n";
    }
    fout.close();
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
	auto orderedPairs = reorderBySide(pairs, pbs, dieRect);
    // 1) 把 pairs 反轉，收集 netName -> pads[]
    map<string, vector<Pt>> net2pads;
    for (auto &pr : orderedPairs) {
        const Pt pad = pbs[pr.first].pad;
        for (auto &netName : pr.second) {
            net2pads[netName].push_back(pad);
        }
    }
	ofstream fout("net_info.txt", ios::trunc);
    if (!fout) { cerr<<"Cannot open net_info.txt\n"; return 1; }
	//output area info to csv file
	std::vector<std::pair<int,double>> netAreas;
	netAreas.reserve(orderedPairs.size());
    int IDcnt = 0;
    for (auto &pr : orderedPairs) {
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
        // 畫線
        drawScene(ls0, ls1, pb, ballRect, fullRect, inf);
        Polygon poly = buildPolygon(ls0, ls1, dieRect, ballRect, fullRect, inf);
        printPolygon(ls0, ls1, dieRect, ballRect, fullRect, inf);

        if (!checkPolygonContainsPadBall(poly, pb)) {
            cerr << "[Warning] net " << pr.first
                 << " polygon does not cover pad/ball\n";
        }
		 // 调用写入 + echo
		outputNetInfo(fout,
                  IDcnt,
                  pb,
                  vector<LS>{ls0, ls1},
                  dieRect, ballRect, fullRect,
                  inf,
                  poly);
		double area = bg::area(poly);
		netAreas.emplace_back(IDcnt, area);
	    IDcnt++;
    }
	exportNetAreasToCSV("net_areas.csv", netAreas);
    return 0;
}