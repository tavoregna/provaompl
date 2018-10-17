#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
using namespace std;
int main(int argc, char **argv)
{
    map <string,double> mappavalori;
    ifstream filevalori;
    filevalori.open("/home/riccardo/ros_ws/src/g04-carraro-sassi-sturaro/param/points.pts");
    if(filevalori.is_open())
    {
        while (!filevalori.eof())
        {
            string linea;
            getline(filevalori,linea);
            string nome="";
            string valore="";
            int i=0;
            for(;i<linea.size();i++)
            {
                char c=linea.at(i);
                if(c!=' ')
                {
                    nome+=c;
                }
                else
                {
                    break;
                }
            }
            if(nome=="")
                continue;
            for(i=i+1;i<linea.size();i++)
            {
                char c=linea.at(i);
                valore+=c;
            }
            printf("nome: %s, valore: %f \n",nome.c_str(),atof(valore.c_str())); 
            mappavalori.insert(pair <string, double> (nome.c_str(), atof(valore.c_str()))); 
        }
    }
    filevalori.close();
    
    std::cout << "mappavalori[dddd] is " << mappavalori["dddd"] << '\n';
    std::cout << "mappavalori[fgfdf] is " << mappavalori["fgfdf"] << '\n';
    return 0;
}
