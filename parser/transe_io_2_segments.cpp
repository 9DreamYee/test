#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
using namespace std;
int main(int argc, char *argv[]) {
    //usage ./transe_io_info_2_segments <input_file>
    std::ifstream file(argv[1]);
    ofstream outfile(argv[2], ios::out);
    if (!file) {
        std::cerr << "Unable to open file\n"<<"usage ./transe_io_info_2_segments <input_file> <output_file>\n";
        return 1;
    }

    std::string line;
    /*while (std::getline(file, line)) {
        if (!line.empty()) {
            std::istringstream iss(line);
            int a, b, c, d;
            iss >> a >> b;
            if(std::getline(file, line) && !line.empty()){
            std::istringstream iss2(line);
            iss2 >> c >> d;
            std::cout << "segments.push_back(Segment(" 
                << a << " " << b << "," << c << " " << d << "))\n";
            
            }
            if(std::getline(file, line) && !line.empty()){
                a = c;
                b = d;
                std::istringstream iss3(line);
                iss3 >> c >> d;
                std::cout << "segments.push_back(Segment(" 
                    << a << " " << b << "," << c << " " << d << "))\n";
            }    

        }
    }*/
    vector<int> vec;
    while(getline(file, line)){
        if (!line.empty()){
        std::istringstream iss(line);
        int a, b, c, d;
        iss >> a >> b;
        vec.push_back(a);
        vec.push_back(b);
        /*std::cout << "segments.push_back(Segment(" 
            << a << " " << b << "," << c << " " << d << "))\n";
        */
        }
        else{
            for(int i = 2;i<vec.size();i+=2){
                outfile<<"segments.push_back(Segment("<<vec[i-2]<<","<<vec[i-1]<<", "<<vec[i]
                    <<","<<vec[i+1]<<"));\n";
            }
            vec.clear();
        }
    }
    return 0;
}
