#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
using namespace std;
int main(int argc, char *argv[]){
    string str,temp,str2;
    fstream file;
    ofstream outfile;
    file.open(argv[1],ios::in);
    outfile.open(argv[2],ios::out);
    //usage ./parse_ses <input_file> <output_file>
    stringstream ss;
    while(getline(file,str)){
        ss.clear();
        ss.str("");
        ss<<str;
        ss>>temp;
        string x,y,str1,str3;
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
                    //cout<<str1<<" "<<str3<<endl;
                    outfile<<str1<<" "<<str3<<endl;
                }
            }
        }
    }
    file.close();
    outfile.close();
    return 0;
}
