#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// 定義 Segment 結構
struct Segment {
    long long x1;
    long long y1;
    long long x2;
    long long y2;
    Segment(long long _x1, long long _y1,
            long long _x2, long long _y2)
      : x1(_x1), y1(_y1), x2(_x2), y2(_y2) {}
};

// 解析 .ses 檔案，提取 IO 資訊，輸出到中介檔案
bool parseSES(const std::string& inputFile, const std::string& outputFile) {
    std::fstream file(inputFile, std::ios::in);
    if (!file) {
        std::cerr << "Cannot open SES file: " << inputFile << "\n";
        return false;
    }
    std::ofstream outfile(outputFile, std::ios::out);
    if (!outfile) {
        std::cerr << "Cannot create output file: " << outputFile << "\n";
        return false;
    }

    std::string str, temp;
    std::stringstream ss;
    while(getline(file,str)){
        ss.clear();
        ss.str("");
        ss<<str;
        ss>>temp;
        std::string x,y,str1,str3;
        int x_cor=0,y_cor=0;
        if(temp=="(net"){
            getline(file,str);
            ss<<str;
            ss>>temp;
            getline(file,str1);
            ss<<str1;
            ss>>temp;
            if(temp=="(via"){
                continue;    
             }
            else if(temp == "(wire"){
                ss.clear();
                ss.str("");
                while(str != "          )"){
                    ss.clear();
                    ss.str("");
                    getline(file,str);
                    ss<<str;
                    ss>>str1;
                    ss<<str;
                    ss>>str3;
                    if(str1 == ")"){
                        outfile<<"\n";
                        continue;
                    }
                    outfile<<str1<<" "<<str3<<std::endl;
                }
            }
        }
    }// end of while

    file.close();
    outfile.close();
    return true;
}

// 讀取中介 IO 檔案，轉換成 Segment push_back 語句
bool transeIOToSegments(const std::string& inputFile, const std::string& outputFile) {
    std::ifstream file(inputFile);
    if (!file) {
        std::cerr << "Unable to open IO info file: " << inputFile << "\n";
        return false;
    }
    std::ofstream outfile(outputFile, std::ios::out);
    if (!outfile) {
        std::cerr << "Cannot create output file: " << outputFile << "\n";
        return false;
    }

    std::string line;
    std::vector<long long> vec;
    while(getline(file, line)){
        if (!line.empty()){
            std::istringstream iss(line);
            int a, b, c, d;
            iss >> a >> b;
            vec.push_back(a);
            vec.push_back(b);
        }
        else{
            for(int i = 2;i<vec.size();i+=2){
                /*
                outfile<<"segments.push_back(Segment("<<vec[i-2]<<","<<vec[i-1]<<", "<<vec[i]
                    <<","<<vec[i+1]<<"));\n";
                */
                outfile<<vec[i-2]<<" "<<vec[i-1]<<" "<<vec[i]<<" "<<vec[i+1]<<"\n";
            }
            vec.clear();
        }
    } // end of while
    file.close();
    outfile.close();
    return true;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <input.ses> <intermediate_io.txt> <output_segments.txt>\n";
        return 1;
    }

    const std::string sesFile  = argv[1];
    const std::string ioFile   = argv[2];
    const std::string segFile  = argv[3];

    if (!parseSES(sesFile, ioFile)) {
        return 1;
    }
    if (!transeIOToSegments(ioFile, segFile)) {
        return 1;
    }
    return 0;
}
