#ifndef SOLUTION
#define SOLUTION

#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>

#include "route.h"
#include "vertex.h"

using namespace std;

struct Solution{
    vector<Route *> routes;
    float * timeIn;
    float * timeOut;
    float * timeWindowInitialDrone;
    float * timeWindowEndDrone;

    Solution(int numberOfClientes, float capacity){
        timeIn = new float[numberOfClientes];
        timeOut = new float[numberOfClientes];
        timeWindowInitialDrone = new float[numberOfClientes];
        timeWindowEndDrone = new float[numberOfClientes];
        for(int i=0;i<numberOfClientes;i++){
            timeIn[i]=-1.0;
            timeOut[i]=-1.0;
            timeWindowInitialDrone[i]=-1.0;
            timeWindowEndDrone[i]=-1.0;
        }
        routes.push_back( new Route(routes.size(), capacity, numberOfClientes));
    }

    Solution(Solution * s, int numberOfClientes){
        timeIn = new float[numberOfClientes];
        timeOut = new float[numberOfClientes];
        timeWindowInitialDrone = new float[numberOfClientes];
        timeWindowEndDrone = new float[numberOfClientes];
        for(int i=0;i<numberOfClientes;i++){
            timeIn[i]=s->timeIn[i];
            timeOut[i]=s->timeOut[i];
            timeWindowInitialDrone[i]=s->timeWindowInitialDrone[i];
            timeWindowEndDrone[i]=s->timeWindowEndDrone[i];
        }
        for(int i=0;i<s->routes.size();i++){
            routes.push_back( new Route(s->routes[i], numberOfClientes));
        }
    }

    Solution(int numberOfClientes){
        timeIn = new float[numberOfClientes];
        timeOut = new float[numberOfClientes];
        timeWindowInitialDrone = new float[numberOfClientes];
        timeWindowEndDrone = new float[numberOfClientes];
        for(int i=0;i<numberOfClientes;i++){
            timeIn[i]=-1.0;
            timeOut[i]=-1.0;
            timeWindowInitialDrone[i]=-1.0;
            timeWindowEndDrone[i]=-1.0;
        }
    }

    ~Solution(){
        for(int i=0;i<routes.size();i++){
            delete(routes[i]);
        }
        routes.clear();
        delete[](timeIn);
        delete[](timeOut);
        delete[](timeWindowInitialDrone);
        delete[](timeWindowEndDrone);
    }
};

float calcCostSolution(Solution *s){
    float cost = 0.0;
    for(int i=0;i<s->routes.size();i++){
        cost+=s->routes[i]->cost;
    }
    return cost;
}

void printSolution(Solution *s){
    cout << "Custo: " << calcCostSolution(s) << endl;
    for(int i=0; i<s->routes.size(); i++){
        cout << "Caminhão " << i << ": " << endl;
        for(int j=0; j<s->routes[i]->pathTruck.size(); j++){
            cout << " - " << s->routes[i]->pathTruck[j];
        }
        cout << endl << "Drone";
        for(int j=0;j<s->routes[i]->pathsDrone.size();j++){
            cout << " - " << s->routes[i]->pathsDrone[j];
        }
        cout << endl << endl;
    }
}

void updateTimeWindowDrone(Solution * s, int idVertex, float time){
    if(s->timeWindowInitialDrone[idVertex] == -1){
        s->timeWindowInitialDrone[idVertex]=time;
        s->timeWindowEndDrone[idVertex]=time;
    }else{
        if(time < s->timeWindowInitialDrone[idVertex]){
            s->timeWindowInitialDrone[idVertex] = time;
        }
        if(time > s->timeWindowEndDrone[idVertex]){
            s->timeWindowEndDrone[idVertex] = time;
        }
    }
}

float getTimeOut(Solution * s, int truck, int pos, vector<vector<float>> * graphTruck){
    if(pos==0){
        if(s->routes[truck]->pathTruck.size()==2){
            return 0;
        }else{
            return s->timeIn[s->routes[truck]->pathTruck[1]]-graphTruck->at(s->routes[truck]->pathTruck[0])[s->routes[truck]->pathTruck[1]];
        }
    }else if(pos==s->routes[truck]->pathTruck.size()-1){
        return MAXFLOAT;
    }else{
        return s->timeOut[s->routes[truck]->pathTruck[pos]];
    }
}

float getTimeIn(Solution * s, int truck, int pos, vector<vector<float>> * graphTruck){
    if(pos == s->routes[truck]->pathTruck.size()-1){
        float timeOut = getTimeOut(s,truck,pos-1, graphTruck);
        return timeOut+graphTruck->at(s->routes[truck]->pathTruck[pos-1])[s->routes[truck]->pathTruck[pos]];
    }
    if(pos == 0){
        return 0;
    }
    return s->timeIn[s->routes[truck]->pathTruck[pos]];
}

float getTimeAttended(Solution * s, int truck, int pos, vector<vector<float>> * graphTruck, vector<Vertex> * vertex){
    float time = getTimeIn(s,truck,pos, graphTruck);
    if(time < vertex->at(s->routes[truck]->pathTruck[pos]).initialTime){
        time = vertex->at(s->routes[truck]->pathTruck[pos]).initialTime;
    }
    return time;
}

float getTimeWindowInitialDrone(Solution * s, int truck, int v, vector<vector<float>> * graphTruck, vector<vector<float>> * graphDrone, int numberOfVertex){
    if(v == 0){
        return 0.0;
    }
    if(v == numberOfVertex-1){
        float time = -1.0;
        for(int i=1;i<s->routes[truck]->pathsDrone.size();i=i+3){
            if( s->routes[truck]->pathsDrone[i-1] == v ){
                float t = s->timeIn[ s->routes[truck]->pathsDrone[i] ] - graphDrone->at(s->routes[truck]->pathsDrone[i-1])[s->routes[truck]->pathsDrone[i]];
                if(time == -1 || t < time){
                    time=t;
                }
            }
            if( s->routes[truck]->pathsDrone[i+1] == v ){
                float t = s->timeOut[ s->routes[truck]->pathsDrone[i] ] + graphDrone->at(s->routes[truck]->pathsDrone[i])[s->routes[truck]->pathsDrone[i+1]];
                float timeIn = getTimeIn(s,truck,s->routes[truck]->pathTruck.size()-1, graphTruck);
                if(t < timeIn){
                    t=timeIn;
                }
                if(time == -1 || t < time){
                    time=t;
                }
            }
        }
        return time;
    }
    return s->timeWindowInitialDrone[ v ];
}

float getTimeWindowEndDrone(Solution * s, int truck, int v, vector<vector<float>> * graphTruck, vector<vector<float>> * graphDrone, int numberOfVertex){
    if(v==0){
        float time=-1.0;
        for(int i=1;i<s->routes[truck]->pathsDrone.size();i=i+3){
            if( s->routes[truck]->pathsDrone[i-1] == 0 ){
                float t = s->timeIn[ s->routes[truck]->pathsDrone[i] ] - graphDrone->at(s->routes[truck]->pathsDrone[i-1])[s->routes[truck]->pathsDrone[i]];
                float timeOut = getTimeOut(s,truck,0,graphTruck);
                if(t > timeOut){
                    t=timeOut;
                }
                if(time == -1 || t > time){
                    time=t;
                }
            }
            if( s->routes[truck]->pathsDrone[i+1] == 0 ){
                float t = s->timeOut[ s->routes[truck]->pathsDrone[i] ] + graphDrone->at(s->routes[truck]->pathsDrone[i])[s->routes[truck]->pathsDrone[i+1]];
                float timeIn = getTimeIn(s,truck,0,graphTruck);
                if(t < timeIn){
                    t=timeIn;
                }
                if(time == -1 || t > time){
                    time=t;
                }
            }
        }
    }
    if(v == numberOfVertex-1){
        return MAXFLOAT;
    }
    return s->timeWindowEndDrone[ v ];
}

bool solutionIsViable(Solution *s, vector<vector<float>> * graphTruck, vector<vector<float>> * graphDrone, vector<Vertex> * vertex, float distMaxDrone, float maxAwaitTime, float capacityTruck){
    for(int i=1;i<vertex->size()-1;i++){
        if(s->timeWindowInitialDrone[vertex->at(i).id] != -1){
            if(s->timeOut[vertex->at(i).id] < s->timeIn[vertex->at(i).id]){
                cout << "Tempo de chegada e maior que o tempo de saida" <<  endl;
                return false;
            }
            if(s->timeIn[vertex->at(i).id] > s->timeWindowInitialDrone[vertex->at(i).id] || s->timeOut[vertex->at(i).id] < s->timeWindowEndDrone[vertex->at(i).id]){
                cout << "Erro na janela de tempo dos drones" <<  endl;
                return false;
            }
            if(s->timeIn[vertex->at(i).id] > vertex->at(i).endTime){
                cout << "Cliente foi atendido depois do fechamento de sua janela" << endl;
                return false;
            }
            if(s->timeOut[vertex->at(i).id] < vertex->at(i).initialTime){
                cout << "Cliente foi atendido antes da abertura de sua janela" << endl;
                return false;
            }
        }
    }
    int qtd = 0;
    vector<bool> attended(vertex->size(),false);
    for(int i=0;i<s->routes.size();i++){
        float currentWeight = 0.0;
        float time = 0.0;
        for(int j=1;j<s->routes[i]->pathTruck.size()-1;j++){
            if(!attended[s->routes[i]->pathTruck[j]]){
                attended[s->routes[i]->pathTruck[j]]=true;
                currentWeight+=vertex->at(s->routes[i]->pathTruck[j]).request;
                qtd++;
            }else{
                cout << "Cliente atendido 2 vezes" << endl;
                return false;
            }
            time+=graphTruck->at(s->routes[i]->pathTruck[j-1])[s->routes[i]->pathTruck[j]];
            if(time > s->timeIn[s->routes[i]->pathTruck[j]]){
                cout << "Tempo de chegada incorreto" << endl;
                return false;
            }
            float timeAttended = getTimeAttended(s,i,j,graphTruck,vertex);
            if( time <=  timeAttended){
                time = timeAttended;
            }else{
                cout << "Tempo de atendimento incorreto" << endl;
                return false;
            }
            time+=vertex->at(s->routes[i]->pathTruck[j]).serviceTimeTruck;
            if(time <= s->timeOut[s->routes[i]->pathTruck[j]]){
                time = s->timeOut[s->routes[i]->pathTruck[j]];
            }else{
                cout << "Tempo de saida incorreto" << endl;
                return false;
            }
        }
        for(int j=1;j<s->routes[i]->pathsDrone.size();j=j+3){
            int posOut = s->routes[i]->pathsDrone[j-1];
            int posAttended = s->routes[i]->pathsDrone[j];
            int posBack = s->routes[i]->pathsDrone[j+1];
            if(!attended[posAttended]){
                attended[posAttended]=true;
                currentWeight+=vertex->at(posAttended).request;
                qtd++;
            }else{
                cout << "Cliente atendido 2 vezes" << endl;
                return false;
            }
            if(graphDrone->at(posOut)[posAttended]+graphDrone->at(posAttended)[posBack] > distMaxDrone){
                cout << "Drone percorreu distancia maior que a permitida em uma entrega" << endl;
                return false;
            }
            if(!vertex->at(posAttended).drone){
                cout << "Cliente que nao pode ser atendido por drone foi atendido por drone" << endl;
                return false;
            }
            float timeMin;
            float timeMax;
            if(posOut==0){
                timeMin = 0;
                timeMax = getTimeIn(s,i,1,graphTruck) - graphTruck->at(s->routes[i]->pathTruck[0])[s->routes[i]->pathTruck[1]];
            }else if(posOut==vertex->size()-1){
                timeMax = MAXFLOAT;
                timeMin = getTimeOut(s,i,s->routes[i]->pathTruck.size()-2,graphTruck)+graphTruck->at(s->routes[i]->pathTruck[s->routes[i]->pathTruck.size()-2])[s->routes[i]->pathTruck.back()];
            }else{
                timeMin = s->timeIn[posOut];
                timeMax = s->timeOut[posOut];
            }
            timeMin+=graphDrone->at(posOut)[posAttended];
            timeMax+=graphDrone->at(posOut)[posAttended]+maxAwaitTime;
            if(s->timeIn[posAttended] < timeMin || s->timeIn[posAttended] > timeMax){
                cout << "Tempos de chegada ao cliente via drone impraticavel" << endl;
                return false;
            }
            timeMin = s->timeIn[posAttended];
            if(timeMin < vertex->at(posAttended).initialTime){
                timeMin = vertex->at(posAttended).initialTime;
            }
            timeMin+=vertex->at(posAttended).serviceTimeDrone;
            if(s->timeOut[posAttended] < timeMin){
                cout << "Tempo de saída do cliente incorreto" << endl;
                return false;
            }
            float timeIn;
            float timeOut;
            if(posBack == 0){
                timeIn = 0;
                timeOut = getTimeOut(s,i,0,graphTruck);
            }else if(posBack == vertex->back().id){
                timeIn = getTimeIn(s,i,s->routes[i]->pathTruck.size()-1,graphTruck);
                timeOut = MAXFLOAT;
            }else{
                timeIn = s->timeIn[posBack];
                timeOut = s->timeOut[posBack];
            }
            float timeBack = s->timeOut[posAttended]+graphDrone->at(posAttended)[posBack];
            if((timeIn - timeBack)  > maxAwaitTime){
                cout << "Drone teve que esperar tempo maior que o possível para voltar ao caminhão" << endl;
                return false;
            }
            if(timeBack>timeOut){
                cout << "Drone chegou tarde demais para o caminhao" << endl;
                return false;
            }
        }
        
        if(currentWeight > capacityTruck){
            cout << "Caminhao carregou peso demais" << endl;
            return false;
        }
    }
    if(qtd!=vertex->size()-2){
        cout << "Algum cliente nao foi atendido" << endl;
        return false;
    }
    return true;
}

void plotSolution(Solution *s, string instance){
    string filename = "solutionForPlot.txt";
    ofstream file(filename);
    file << s->routes.size() << endl;
    file << calcCostSolution(s) << endl;
    for(int i=0;i<s->routes.size();i++){
        string pathTruck = "";
        for(int j=0;j<s->routes[i]->pathTruck.size();j++){
            pathTruck= pathTruck + "-" + to_string(s->routes[i]->pathTruck[j]);
        }
        if(!pathTruck.empty())
            file << pathTruck.substr(1);
        file << endl;
        string pathDrone = "";
        for(int j=0;j<s->routes[i]->pathsDrone.size();j++){
            pathDrone= pathDrone + "-" + to_string(s->routes[i]->pathsDrone[j]);
        }
        if(!pathDrone.empty())
            file << pathDrone.substr(1);
        file << endl;
    }
    file.close();
    string command = "python3 plotSolution.py " + instance + " " + filename;
    int aux = system(command.c_str());
    command = "rm " + filename;
    aux = system(command.c_str());
}

#endif