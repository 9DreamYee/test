// parse_boundary.cpp — 完整可編譯版 V1.11
// 功能：
//   1. 解析 SES (.ses) 與 original_coords 檔案
//   2. 讀取 Ball Rect 四邊、Pad/Ball 座標
//   3. 解析所有網路路徑，過濾指定區域邊緣、排序點序列
//   4. 採用「貪婪掃描」演算法，以容量=2 的方式為每個 pad 分配兩條 net
//   5. 對每組 NetGroup，分別從兩條邊界的交點中，各選靠近 pad 的一個，再取中點更新 ball，並輸出資訊與繪圖指令

#include <bits/stdc++.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
using namespace std;
namespace bg = boost::geometry;

using Pt  = bg::model::point<long long,2,bg::cs::cartesian>;
using LS  = bg::model::linestring<Pt>;
using LSM = map<string,LS>;

struct PB   { Pt pad, ball; };
struct Rect { long long xmin,ymin,xmax,ymax; };

static string trim(const string &s) {
    auto a = s.find_first_not_of(" \t\r\n");
    auto b = s.find_last_not_of(" \t\r\n");
    return (a==string::npos)?string():s.substr(a,b-a+1);
}
static int countChar(const string &s,char c) {
    return int(count(s.begin(), s.end(), c));
}
static long long manh(const Pt &a,const Pt &b) {
    return llabs(bg::get<0>(a)-bg::get<0>(b)) + llabs(bg::get<1>(a)-bg::get<1>(b));
}

// 1. 解析 SES header
struct SESInfo { long long res=1, offX=0, offY=0; };
SESInfo parseSesInfo(const string &file) {
    ifstream fin(file); if(!fin) throw runtime_error("open SES fail");
    SESInfo inf; string line;
    while(getline(fin,line)){
        auto t=trim(line);
        if(t.rfind("(resolution",0)==0) {
            istringstream is(t); string tag,unit; is>>tag>>unit>>inf.res;
        }
        else if(t.rfind("(place",0)==0) {
            istringstream is(t);
            string plc,comp;
            long long x,y;
            is>>plc>>comp>>x>>y;
            if(comp=="DIE1") { inf.offX=x; inf.offY=y; }
        }
    }
    return inf;
}

// 2. 讀 Ball Rect
Rect readBallRect(const string &file) {
    ifstream fin(file); if(!fin) throw runtime_error("open orig fail");
    string line;
    while(getline(fin,line)){
        auto t=trim(line);
        if(t.rfind("Ball Rect:",0)==0) {
            Rect r;
            sscanf(t.c_str(),"Ball Rect: xmin=%lld ymin=%lld xmax=%lld ymax=%lld",
                   &r.xmin,&r.ymin,&r.xmax,&r.ymax);
            return r;
        }
    }
    throw runtime_error("Missing Ball Rect");
}

// 3. 解析 SES paths
LSM parseSesPaths(const string &file,
                  long long minX,long long minY,
                  long long maxX,long long maxY) {
    ifstream fin(file); if(!fin) throw runtime_error("open SES paths fail");
    LSM nets;
    string line,name;
    int depth=0;
    vector<Pt> buf;
    const long long eps=10;
    auto flush=[&]() {
        if(name.empty()||buf.size()<2) { buf.clear(); name.clear(); return; }
        vector<Pt> pts;
        for(auto &p:buf)
            if(pts.empty()|| bg::distance(p,pts.back())>0)
                pts.push_back(p);
        buf.clear();
        if(pts.size()<2) { name.clear(); return; }
        auto onEdge=[&](const Pt &p){
            return llabs(bg::get<0>(p)-minX)<=eps
                || llabs(bg::get<0>(p)-maxX)<=eps
                || llabs(bg::get<1>(p)-minY)<=eps
                || llabs(bg::get<1>(p)-maxY)<=eps;
        };
        Pt ctr;
        bg::set<0>(ctr,(minX+maxX)/2);
        bg::set<1>(ctr,(minY+maxY)/2);
        auto sqd=[&](int i){
            long long dx=bg::get<0>(pts[i])-bg::get<0>(ctr);
            long long dy=bg::get<1>(pts[i])-bg::get<1>(ctr);
            return dx*dx+dy*dy;
        };
        vector<int> idx;
        for(int i=0;i<(int)pts.size();++i)
            if(onEdge(pts[i])) idx.push_back(i);
        int st = idx.empty()?0:*min_element(idx.begin(),idx.end(),[&](int a,int b){return sqd(a)<sqd(b);});
        if(idx.empty()) {
            vector<int> all(pts.size()); iota(all.begin(),all.end(),0);
            st = *min_element(all.begin(),all.end(),[&](int a,int b){return sqd(a)<sqd(b);});
        }
        vector<char> used(pts.size());
        vector<Pt> ord;
        ord.reserve(pts.size());
        ord.push_back(pts[st]); used[st]=1;
        int cur=st;
        for(size_t k=1;k<pts.size();++k) {
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
        if(t.rfind("(net",0)==0) {
            flush();
            istringstream is(t);
            string tmp;
            is>>tmp>>name;
            if(!name.empty()&&name.back()==')') name.pop_back();
        }
        else if(t.rfind("(path",0)==0) depth=1;
        else if(depth>0) {
            depth += countChar(t,'(')-countChar(t,')');
            istringstream is(t);
            long long x,y;
            if(is>>x>>y) buf.emplace_back(x,y);
        }
    }
    flush();
    return nets;
}

// 4. 解析 original_coords: Pad/Ball
map<string,PB> parseOriginalCoords(const string &file,const SESInfo &inf) {
    ifstream fin(file); if(!fin) throw runtime_error("open orig fail");
    map<string,PB> mp;
    string line,cur;
    auto readPt=[&](const string &s){
        auto l=s.find('('),r=s.find(')');
        string v=s.substr(l+1,r-l-1);
        replace(v.begin(),v.end(),',',' ');
        istringstream is(v);
        double x,y;
        is>>x>>y;
        return make_pair(x,y);
    };
    while(getline(fin,line)){
        auto t=trim(line);
        if(t.rfind("Net",0)==0 && t.find(':')!=string::npos)
            cur = t.substr(0,t.find(':'));
        else if(t.rfind("Pad_location",0)==0) {
            auto pr=readPt(t);
            PB &b = mp[cur];
            bg::set<0>(b.pad, llround(pr.first*inf.res)+inf.offX);
            bg::set<1>(b.pad, llround(pr.second*inf.res)+inf.offY);
        }
        else if(t.rfind("Ball_location",0)==0) {
            auto pr=readPt(t);
            PB &b = mp[cur];
            bg::set<0>(b.ball, llround(pr.first*inf.res)+inf.offX);
            bg::set<1>(b.ball, llround(pr.second*inf.res)+inf.offY);
        }
    }
    return mp;
}

// 5. 貪婪掃描配對 (capacity=2)
vector<pair<string,vector<string>>> greedyPair(
    const map<string,PB>&pbs,
    const LSM& nets
) {
    int P = pbs.size(), N = nets.size();
    vector<string> padIds, netIds;
    padIds.reserve(P); netIds.reserve(N);
    for(auto &kv:pbs) padIds.push_back(kv.first);
    for(auto &kv:nets) netIds.push_back(kv.first);
    struct E{int pi,ni; long long d2;};
    vector<E> edges; edges.reserve(P*N);
    for(int i=0;i<P;++i) {
        auto pp = pbs.at(padIds[i]).pad;
        for(int j=0;j<N;++j) {
            auto st = nets.at(netIds[j]).front();
            long long dx = bg::get<0>(st)-bg::get<0>(pp);
            long long dy = bg::get<1>(st)-bg::get<1>(pp);
            edges.push_back({i,j,dx*dx+dy*dy});
        }
    }
    sort(edges.begin(),edges.end(),[](auto &a,auto &b){ return a.d2 < b.d2; });
    vector<int> padCnt(P), netCnt(N);
    vector<vector<string>> assign(P);
    for(auto &e: edges) {
        if(padCnt[e.pi]<2 && netCnt[e.ni]<2) {
            assign[e.pi].push_back(netIds[e.ni]);
            padCnt[e.pi]++; netCnt[e.ni]++;
            bool done=true;
            for(int x=0;x<P;++x) if(padCnt[x]<2){ done=false; break; }
            if(done) break;
        }
    }
    vector<pair<string,vector<string>>> res;
    for(int i=0;i<P;++i) res.emplace_back(padIds[i],assign[i]);
    return res;
}

// collect intersections
void collectIntersections(
    const LS &ls,
    const Rect &r,
    vector<Pt> &pts
) {
    for(size_t i=0;i+1<ls.size();++i) {
        auto &A=ls[i], &B=ls[i+1];
        long long x1=bg::get<0>(A), y1=bg::get<1>(A);
        long long x2=bg::get<0>(B), y2=bg::get<1>(B);
        // horizontal overlapping
        if(y1==y2 && (y1==r.ymax||y1==r.ymin)) {
            long long lo=min(x1,x2), hi=max(x1,x2);
            lo = max(lo,r.xmin); hi = min(hi,r.xmax);
            if(lo<=hi) pts.emplace_back((lo+hi)/2, y1);
        }
        // vertical overlapping
        if(x1==x2 && (x1==r.xmin||x1==r.xmax)) {
            long long lo=min(y1,y2), hi=max(y1,y2);
            lo = max(lo,r.ymin); hi = min(hi,r.ymax);
            if(lo<=hi) pts.emplace_back(x1,(lo+hi)/2);
        }
        // crossing horizontal
        if(y1!=y2) for(auto ey:{r.ymax,r.ymin}) {
            if((y1<ey&&ey<y2)||(y2<ey&&ey<y1)){
                double t=double(ey-y1)/double(y2-y1);
                long long xi=llround(x1+t*(x2-x1));
                if(xi>=r.xmin&&xi<=r.xmax) pts.emplace_back(xi,ey);
            }
        }
        // crossing vertical
        if(x1!=x2) for(auto ex:{r.xmin,r.xmax}) {
            if((x1<ex&&ex<x2)||(x2<ex&&ex<x1)){
                double t=double(ex-x1)/double(x2-x1);
                long long yi=llround(y1+t*(y2-y1));
                if(yi>=r.ymin&&yi<=r.ymax) pts.emplace_back(ex,yi);
            }
        }
    }
}

// output NetGroup info + intersections
void outputNetGroupInfo(
    int idx,
    const string &id,
    const PB &pb,
    const vector<string> &bds,
    const LSM &nets,
    const Rect &r
) {
    cout<<"NetGroup "<<idx<<"  ("<<id<<")\n";
    cout<<"  Pad  = ("<<bg::get<0>(pb.pad)<<","<<bg::get<1>(pb.pad)<<")\n";
    cout<<"  Ball = ("<<bg::get<0>(pb.ball)<<","<<bg::get<1>(pb.ball)<<")\n";
    for(int k=0;k<2;++k){
        auto &tag=bds[k]; auto &ls=nets.at(tag);
        cout<<"  Boundary"<<(k+1)<<"="<<tag<<" "<<ls.size()<<" pts: ";
        for(auto &p:ls) cout<<"("<<bg::get<0>(p)<<","<<bg::get<1>(p)<<") ";
        cout<<"\n";
    }
    vector<Pt> pts;
    collectIntersections(nets.at(bds[0]),r,pts);
    collectIntersections(nets.at(bds[1]),r,pts);
    sort(pts.begin(),pts.end(),[](auto&a,auto&b){ if(bg::get<0>(a)!=bg::get<0>(b)) return bg::get<0>(a)<bg::get<0>(b); return bg::get<1>(a)<bg::get<1>(b); });
    pts.erase(unique(pts.begin(),pts.end(),[](auto&a,auto&b){ return bg::get<0>(a)==bg::get<0>(b)&&bg::get<1>(a)==bg::get<1>(b); }),pts.end());
    for(int i=0;i<2 && i<pts.size();++i)
        cout<<"  Intersection"<<(i+1)<<" = ("<<bg::get<0>(pts[i])<<","<<bg::get<1>(pts[i])<<")\n";
    cout<<"\n";
}

// drawScene: Pad/Ball + boundaries + intersections
void drawScene(
    const LS &l1,
    const LS &l2,
    const PB &pb,
    const Rect &r
) {
    // Pad & Ball
	/*
    cout<<"_view.drawArc("<<bg::get<0>(pb.pad)<<","<<bg::get<1>(pb.pad)<<",10000);\n";
    cout<<"_view.drawArc("<<bg::get<0>(pb.ball)<<","<<bg::get<1>(pb.ball)<<",10000);\n";
	*/
    cout<<"_view.drawArc("<<bg::get<0>(pb.pad)<<","<<bg::get<1>(pb.pad)<<",350000);\n";
    cout<<"_view.drawArc("<<bg::get<0>(pb.ball)<<","<<bg::get<1>(pb.ball)<<",350000);\n";
    // Intersections
    vector<Pt> pts;
    collectIntersections(l1,r,pts);
    collectIntersections(l2,r,pts);
    sort(pts.begin(),pts.end(),[](auto&a,auto&b){ if(bg::get<0>(a)!=bg::get<0>(b)) return bg::get<0>(a)<bg::get<0>(b); return bg::get<1>(a)<bg::get<1>(b); });
    pts.erase(unique(pts.begin(),pts.end(),[](auto&a,auto&b){ return bg::get<0>(a)==bg::get<0>(b)&&bg::get<1>(a)==bg::get<1>(b); }),pts.end());
    //for(auto &p:pts)
        //cout<<"_view.drawArc("<<bg::get<0>(p)<<","<<bg::get<1>(p)<<",10000);\n";
    // Boundaries
    auto drawLines=[&](const LS &ls){
        for(size_t i=0;i+1<ls.size();++i){ auto&p1=ls[i],&p2=ls[i+1];
            cout<<"_view.drawLine("<<bg::get<0>(p1)<<","<<bg::get<1>(p1)<<","<<bg::get<0>(p2)<<","<<bg::get<1>(p2)<<");\n";
        }
    };
    drawLines(l1);
    drawLines(l2);
    cout<<"\n";
}

int main(int argc,char**argv){
    if(argc<7){ cerr<<"Usage: ./parse_boundary <ses> <orig> minX minY width height\n"; return 1; }
    string ses=argv[1], orig=argv[2];
    long long minX=stoll(argv[3]), minY=stoll(argv[4]);
    long long W=stoll(argv[5]), H=stoll(argv[6]);
    long long maxX=minX+W, maxY=minY+H;

    SESInfo inf = parseSesInfo(ses);
    auto nets = parseSesPaths(
        ses,
        minX*inf.res, minY*inf.res,
        maxX*inf.res, maxY*inf.res
    );
    auto pbs  = parseOriginalCoords(orig, inf);
    auto pairsVec = greedyPair(pbs, nets);

    Rect br0 = readBallRect(orig);
    // Ball Rect without offset
    Rect r = {
        br0.xmin*inf.res,
        br0.ymin*inf.res,
        br0.xmax*inf.res,
        br0.ymax*inf.res
    };

    int idx=0;
    for(auto &pr: pairsVec){
        const string &id = pr.first;
        const vector<string> &bds = pr.second;
        PB pb = pbs[id];
        // pick closest intersection from each boundary
        vector<Pt> pts1, pts2;
        collectIntersections(nets.at(bds[0]),r,pts1);
        collectIntersections(nets.at(bds[1]),r,pts2);
        auto pick=[&](const vector<Pt>& v){ return v.empty()?Pt():*min_element(v.begin(),v.end(),[&](auto&a,auto&b){return manh(a,pb.pad)<manh(b,pb.pad);}); };
        Pt i1 = pick(pts1), i2 = pick(pts2);
        Pt mid;
        bg::set<0>(mid,(bg::get<0>(i1)+bg::get<0>(i2))/2);
        bg::set<1>(mid,(bg::get<1>(i1)+bg::get<1>(i2))/2);
        pb.ball = mid;

        //outputNetGroupInfo(idx,id,pb,bds,nets,r);
        drawScene(nets.at(bds[0]),nets.at(bds[1]),pb,r);
        idx++;
    }
    return 0;
}
