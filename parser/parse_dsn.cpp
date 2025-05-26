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

// 投影点 p 到线段 ab 上
static Point projSeg(const Point& p, const Point& a, const Point& b) {
    double vx = b.x - a.x, vy = b.y - a.y;
    double wx = p.x - a.x, wy = p.y - a.y;
    double d = vx*vx + vy*vy;
    double t = d > 0 ? (vx*wx + vy*wy) / d : 0;
    t = max(0.0, min(1.0, t));
    return { a.x + t*vx, a.y + t*vy };
}

// 找 outline 上距离 p 最近的投影点，以及所在边的索引
static PPI nearest(const Point& p, const vector<Point>& outline) {
    Point best = p;
    double bestD2 = numeric_limits<double>::infinity();
    int bestIdx = -1;
    int n = outline.size();
    for (int i = 0; i < n; ++i) {
        Point q = projSeg(p, outline[i], outline[(i+1)%n]);
        double dx = p.x - q.x, dy = p.y - q.y;
        double d2 = dx*dx + dy*dy;
        if (d2 < bestD2) {
            bestD2 = d2;
            best = q;
            bestIdx = i;
        }
    }
    return { best, bestIdx };
}

// 生成矩形 outline 顶点（逆时针）
static vector<Point> rectOutline(double x0, double y0, double w, double h) {
    return { {x0,y0}, {x0+w,y0}, {x0+w,y0+h}, {x0,y0+h} };
}

// 解析 DIE1 block 内的 pin 行，将它们投影到 outline 上，但不写回 lines
static array<int,4> parsePads(
    const vector<string>& lines,
    int s, int e,
    const vector<Point>& outline,
    double padPitch,
    vector<PadEntry>& pads
) {
    array<int,4> cnt = {0,0,0,0};
    for (int i = s+1; i < e; ++i) {
        auto pos = lines[i].find("(pin");
        if (pos == string::npos) continue;
        string indent = lines[i].substr(0, pos);
        string rest = lines[i].substr(pos);
        istringstream iss(rest);
        string pin, name, num, xs, ys;
        if (!(iss >> pin >> name >> num >> xs >> ys)) continue;
        while (!ys.empty() && !(isdigit((unsigned char)ys.back()) || ys.back()=='.' || ys.back()=='-'))
            ys.pop_back();
        Point oldp{ stod(xs), stod(ys) };
        auto pr = nearest(oldp, outline);
        size_t yp = rest.find(ys);
        string suf = yp != string::npos ? rest.substr(yp + ys.size()) : ")";
        pads.push_back({ i, indent, name, num, suf, pr.first, pr.second });
        ++cnt[pr.second];
    }
    return cnt;
}

// 对每条边上重叠或距离过小的 pad 进行调整，保证间距至少 padPitch
static void resolveOverlaps(
    vector<PadEntry>& pads,
    double padPitch,
    const vector<Point>& outline
) {
    array<vector<PadEntry*>,4> groups;
    for (auto &pe : pads) {
        groups[pe.edgeIdx].push_back(&pe);
    }
    for (int e = 0; e < 4; ++e) {
        auto &vec = groups[e];
        if (vec.size() < 2) continue;
        // 按投影顺序排序
        sort(vec.begin(), vec.end(), [e](PadEntry* a, PadEntry* b) {
            if (e == 0) return a->newPos.x < b->newPos.x;
            if (e == 1) return a->newPos.y < b->newPos.y;
            if (e == 2) return a->newPos.x > b->newPos.x;
            return a->newPos.y > b->newPos.y;
        });
        // 边的起终点
        Point A = outline[e];
        Point B = outline[(e+1)%4];
        double dx = B.x - A.x, dy = B.y - A.y;
        double len = sqrt(dx*dx + dy*dy);
        double ux = dx / len, uy = dy / len;
        // 首个 pad 的参数化位置
        double tPrev = ((vec[0]->newPos.x - A.x)*ux + (vec[0]->newPos.y - A.y)*uy);
        for (int i = 1; i < vec.size(); ++i) {
            double tCur = ((vec[i]->newPos.x - A.x)*ux + (vec[i]->newPos.y - A.y)*uy);
            if (tCur - tPrev < padPitch) {
                tCur = min(tPrev + padPitch, len);
                vec[i]->newPos = { A.x + ux * tCur, A.y + uy * tCur };
            }
            tPrev = tCur;
        }
    }
}

// 重写 BGA_BGA image block 中的 pin 行，替换为新的 ball 位置
static void rewriteBGA(
    const vector<Point>& bp,
    const vector<string>& bk,
    vector<string>& lines
) {
    int bs = -1, be = -1, depth = 0;
    string padName;
    int n = lines.size();
    for (int i = 0; i < n; ++i) {
        if (bs < 0
            && lines[i].find("(image") != string::npos
            && lines[i].find("BGA_BGA") != string::npos) {
            bs = i;
            depth = count(lines[i].begin(), lines[i].end(), '(')
                  - count(lines[i].begin(), lines[i].end(), ')');
        }
        else if (bs >= 0 && be < 0) {
            depth += count(lines[i].begin(), lines[i].end(), '(')
                   - count(lines[i].begin(), lines[i].end(), ')');
            if (i == bs + 1) {
                istringstream iss(trim(lines[i]));
                string token;
                iss >> token; // "(pin"
                iss >> padName;
            }
            if (depth <= 0) {
                be = i;
            }
        }
    }
    if (bs < 0 || be < 0) return;
    string pinIndent = "   ";
    if (bs+1 < n) {
        auto p = lines[bs+1].find("(pin");
        if (p != string::npos) pinIndent = lines[bs+1].substr(0, p);
    }
    lines.erase(lines.begin() + bs + 1, lines.begin() + be);
    vector<string> newPins;
    for (int k = 0; k < (int)bp.size(); ++k) {
        ostringstream os;
        os << pinIndent
           << "(pin " << padName
           << " "  << bk[k]
           << " "  << bp[k].x << " " << bp[k].y
           << ")";
        newPins.push_back(os.str());
    }
    lines.insert(lines.begin() + bs + 1, newPins.begin(), newPins.end());
}

static void debugCounts(
    const array<int,4>& pc,
    const vector<string>& bk,
    const vector<PadEntry>& pads,
	double &minDist
) {
    // 原来的 pad/ball 计数
    array<int,4> bc={0,0,0,0};
    for (auto &k: bk) {
        int e = (k[0]=='B'?0:k[0]=='R'?1:k[0]=='T'?2:3);
        ++bc[e];
    }
    cerr << "DEBUG Pads:   B=" << pc[0]
         << " R=" << pc[1]
         << " T=" << pc[2]
         << " L=" << pc[3] << "\n";
    cerr << "DEBUG Balls:  B=" << bc[0]
         << " R=" << bc[1]
         << " T=" << bc[2]
         << " L=" << bc[3] << "\n";

    // 计算所有 pad 中心之间的最小距离
    minDist = numeric_limits<double>::infinity();
    int n = pads.size();
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double dx = pads[i].newPos.x - pads[j].newPos.x;
            double dy = pads[i].newPos.y - pads[j].newPos.y;
            double d = sqrt(dx*dx + dy*dy);
            if (d < minDist) minDist = d;
        }
    }
    if (minDist == numeric_limits<double>::infinity()) {
        cerr << "DEBUG Min pad spacing: (only one pad)\n";
    } else {
        cerr << "DEBUG Min pad spacing: " << minDist << "\n";
    }
}


int main(int argc, char* argv[]) {
    if (argc != 11) {
        cerr << "Usage: parse_dsn input.dsn output.dsn x0 y0 w h padPitch ballPitch marginPercent boundaryExpand\n";
        return 1;
    }
    string inF   = argv[1], outF = argv[2];
    double x0    = stod(argv[3]);
    double y0    = stod(argv[4]);
    double w     = stod(argv[5]);
    double h     = stod(argv[6]);
    double padPitch       = stod(argv[7]);
    double ballPitch      = stod(argv[8]);
    double marginPct      = stod(argv[9]);
    double boundaryExpand = stod(argv[10]);

    vector<string> lines;
    ifstream ifs(inF);
    for (string l; getline(ifs, l); ) lines.push_back(l);
    ifs.close();
	



    // DIE1 区块 outline
    auto dieOutline = rectOutline(x0, y0, w, h);
    int ds=-1, de=-1, n = lines.size();
    for (int i=0; i<n; ++i) {
        if (ds<0 && lines[i].find("(image")!=string::npos && lines[i].find("DIE1")!=string::npos)
            ds = i;
        if (ds>=0 && de<0 && lines[i].find("(outline")!=string::npos)
            de = i;
    }
    if (ds<0||de<0) { cerr<<"DIE1 block not found\n"; return 1; }

    // 解析并投影 pads
    vector<PadEntry> pads;
    auto padCnt = parsePads(lines, ds, de, dieOutline, padPitch, pads);

    // 调整 pad 间距
    resolveOverlaps(pads, padPitch, dieOutline);

    // 写回 lines 中的 pad 位置

for (auto &pe : pads) {
    ostringstream os;
    os << pe.indent
       << "(pin " << pe.pinName << " " << pe.pinNum
       << " " << pe.newPos.x << " " << pe.newPos.y;
    // 如果是在右边（edgeIdx==1）或左边（edgeIdx==3），加上旋转标记
    if (pe.edgeIdx == 1 || pe.edgeIdx == 3) {
        os << " (rotate 90)";
    }
    os << pe.suffix;
    lines[pe.lineIndex] = os.str();
}
    // 解析 signal boundary
    double sig_x0=0, sig_y0=0, sig_x1=0, sig_y1=0;
    for (int i=0; i<n; ++i) {
        string t = trim(lines[i]);
        if (t.rfind("(boundary (rect signal",0)==0) {
            istringstream iss(t);
            string b,r,name;
            iss>>b>>r>>name>>sig_x0>>sig_y0>>sig_x1>>sig_y1;
            break;
        }
    }

    // 计算缩小后的 ballOutline 区域
    double width  = sig_x1 - sig_x0;
    double height = sig_y1 - sig_y0;
    double sx = width  * marginPct/100.0;
    double sy = height * marginPct/100.0;
    double bx0 = sig_x0 + sx, by0 = sig_y0 + sy;
    double bw  = width  - 2*sx, bh   = height - 2*sy;

    // 根据 boundaryExpand 略微放大 signal 区域并写回 lines
    double newSigX0 = bx0 - boundaryExpand;
    double newSigY0 = by0 - boundaryExpand;
    double newSigX1 = bx0 + bw + boundaryExpand;
    double newSigY1 = by0 + bh + boundaryExpand;
    for (int i = 0; i < n; ++i) {
        string t = trim(lines[i]);
        if (t.rfind("(boundary (rect signal",0)==0) {
            size_t p = lines[i].find("(boundary");
            string indent = lines[i].substr(0, p);
            ostringstream os;
            os << indent
               << "(boundary (rect signal "
               << newSigX0 << " " << newSigY0 << " "
               << newSigX1 << " " << newSigY1 << "))";
            lines[i] = os.str();
            break;
        }
    }

    // 构造新的 ballOutline
    auto ballOutline = rectOutline(bx0, by0, bw, bh);

    // 生成 ball 及重写 BGA_BGA
    vector<Point> ballPos;
    vector<string> ballKeys;
    const char edgeNames[4] = {'B','R','T','L'};
	/*
	double alpha = 0.6; // <1 时越往中间收，范围 (0,1]
    for (int e = 0; e < 4; ++e) {
        int N = padCnt[e]; if (N <= 0) continue;
        Point A = ballOutline[e], B = ballOutline[(e+1)%4];
        bool asc = (e==0||e==2);
        for (int j = 0; j < N; ++j) {
            // 1) 先算原始线性参数 u
            double u = asc
                       ? (N>1? double(j)/(N-1) : 0.5)
                       : double(j+1)/(N+1);
            // 2) 非线性收缩到 [0.5-(alpha/2),0.5+(alpha/2)]
            double t = 0.5 + (u - 0.5) * alpha;
            ballPos.emplace_back(Point{ A.x + t*(B.x-A.x), A.y + t*(B.y-A.y) });
            ballKeys.push_back(string(1,edgeNames[e]) + to_string(j+1));
        }
    }
	*/
    for (int e=0; e<4; ++e) {
        int N = padCnt[e]; if (N<=0) continue;
        Point A = ballOutline[e], B = ballOutline[(e+1)%4];
        bool asc = (e==0||e==2);
        for (int j=0; j<N; ++j) {
            double t = asc
                       ? (N>1? double(j)/(N-1) : 0.5)
                       : double(j+1)/(N+1);
            ballPos.emplace_back(Point{A.x + t*(B.x-A.x), A.y + t*(B.y-A.y)});
            ballKeys.push_back(string(1,edgeNames[e]) + to_string(j+1));
        }
    }
    rewriteBGA(ballPos, ballKeys, lines);
	double minDist = 0.0;
    debugCounts(padCnt, ballKeys, pads, minDist);
	
    // 根据 padPitch 调整 clearance 和 width
    {
        double Pitch = minDist;
		double ruleVal_clearance = floor((Pitch - 1) * (sqrt(2)/2));
        double ruleVal_width     = 1;
        ostringstream rs, rw;
        rs << ruleVal_clearance;
        rw << ruleVal_width;
        string rv = rs.str();
        string rwv = rw.str();
        for (auto &l : lines) {
            string t = trim(l);
            size_t p;
            if (t.rfind("(rule (clearance",0)==0) {
                p = l.find("(rule");
                string idt = (p!=string::npos?l.substr(0,p):"");
                l = idt + "(rule (clearance " + rv + "))";
            } else if (t.rfind("(rule (width",0)==0) {
                p = l.find("(rule");
                string idt = (p!=string::npos?l.substr(0,p):"");
                l = idt + "(rule (width " + rwv + "))";
            }
        }
		cerr << "DEBUG rule spacing: " << ruleVal_clearance << "\n";
		cerr << "DEBUG rule width: " << ruleVal_width << "\n";
    }

    // 重写 network 块（同前）
    array<vector<PadEntry>,4> pbe;
    array<vector<string>,4> bbe;
    for (auto &pe : pads) pbe[pe.edgeIdx].push_back(pe);
    for (int e=0;e<4;++e) {
        sort(pbe[e].begin(), pbe[e].end(), [e](auto &a, auto &b){
            if(e==0) return a.newPos.x < b.newPos.x;
            if(e==1) return a.newPos.y < b.newPos.y;
            if(e==2) return a.newPos.x > b.newPos.x;
            return a.newPos.y > b.newPos.y;
        });
    }
    for (auto &k: ballKeys) bbe[string("BRTL").find(k[0])].push_back(k);
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

    int ns=-1, ne=-1, depth=0;
    for (int i=0; i<(int)lines.size(); ++i) {
        if (ns<0 && trim(lines[i]).rfind("(network",0)==0) {
            ns = i;
            depth = 1
                  + count(lines[i].begin(), lines[i].end(),'(')
                  - 1
                  - count(lines[i].begin(), lines[i].end(),')');
        }
        else if (ns>=0 && ne<0) {
            depth += count(lines[i].begin(), lines[i].end(),'(')
                   - count(lines[i].begin(), lines[i].end(),')');
            if (depth==0) { ne = i; break; }
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
