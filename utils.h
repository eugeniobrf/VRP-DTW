#ifndef UTILS
#define UTILS

#include<vector>
#include<string>
#include<sstream>
#include<cmath>
#include<fstream>
#include "vertex.h"
#include "solution.h"

using namespace std;

vector<string> split(string str, char delimiter){
    vector<string> s;
    stringstream ss(str);
    string part;
    while(getline(ss, part, delimiter)) {
        s.push_back(part);
    }
    return s;
}

int calcDistEuclidean(int id1, int id2, vector<Vertex> * vertex){
	return round( sqrt((vertex->at(id1).x-vertex->at(id2).x)*(vertex->at(id1).x-vertex->at(id2).x)+(vertex->at(id1).y-vertex->at(id2).y)*(vertex->at(id1).y-vertex->at(id2).y)) );
}

int calcDistManhattan(int id1, int id2, vector<Vertex> * vertex){
    return round( abs(vertex->at(id1).x-vertex->at(id2).x) + abs(vertex->at(id1).y-vertex->at(id2).y));
}

float readFile(string fileName, vector<Vertex> * vertex){
    float capacityTruck;
    fileName = "./" + fileName;
    string line;
    int numVertex;
	ifstream arq(fileName);
    vector <string> s;
    vector<string> lineBroken;
    if(arq.is_open()){
        Vertex aux;
        getline(arq,line);
		numVertex = stoi(line);

        getline(arq,line);
		capacityTruck = stof(line);
		
		getline(arq, line);
		
		while(getline(arq,line)){
            lineBroken=split(line,',');
            aux.id=stoi(lineBroken[0]);
            aux.type=stoi(lineBroken[1]);
            aux.x=stof(lineBroken[2]);
            aux.y=stof(lineBroken[3]);
            aux.initialTime=stoi(lineBroken[4]);
            aux.endTime=stoi(lineBroken[5]);
            aux.request=stoi(lineBroken[6]);
            aux.serviceTimeTruck=stoi(lineBroken[7]);
			aux.serviceTimeDrone=stoi(lineBroken[8]);
			aux.drone=stoi(lineBroken[9]);
            vertex->push_back(aux);
        }
        arq.close();
        aux.id = numVertex;
        aux.type = vertex->at(0).type;
        aux.x = vertex->at(0).x;
        aux.y = vertex->at(0).y;
        aux.initialTime = vertex->at(0).initialTime;
        aux.request = vertex->at(0).request;
        aux.serviceTimeTruck = vertex->at(0).serviceTimeTruck;
		aux.serviceTimeDrone = vertex->at(0).serviceTimeDrone;
		aux.drone = vertex->at(0).drone;
        aux.endTime = vertex->at(0).endTime;
        vertex->push_back(aux);
    }else{
        cout << "Erro ao abrir o arquivo!!!" << endl;
        exit(0);
    }
    return capacityTruck;
}

void makeGraph(vector<vector<float>> * graphTruck, vector<vector<float>> * graphDrone, vector<Vertex> * vertex, float velocityDrone, float velocityTruck){
    graphDrone->resize(vertex->size());
    graphTruck->resize(vertex->size());
    for(int i=0;i<vertex->size();i++){
        graphDrone->at(i).resize(vertex->size());
        graphTruck->at(i).resize(vertex->size());
    }
    for(int i=0;i<vertex->size();i++){
        for(int j=i;j<vertex->size();j++){
            graphDrone->at(i).at(j)=calcDistEuclidean(i,j,vertex)/velocityDrone;
            graphDrone->at(j).at(i)=graphDrone->at(i).at(j);
            graphTruck->at(i).at(j)=calcDistManhattan(i,j,vertex)/velocityTruck;
            graphTruck->at(j).at(i)=graphTruck->at(i).at(j);
        }
    }
}

float calcMaxDistDrone(vector<Vertex> * vertex, vector<vector<float>> * graphDrone){
	int maximo = -1;
	for(int i=0;i<vertex->size();i++){
		for(int j=0;j<vertex->size();j++){
            if(maximo<graphDrone->at(vertex->at(i).id)[vertex->at(j).id]){
			    maximo=graphDrone->at(vertex->at(i).id)[vertex->at(j).id];
            }
		}
	}
	return maximo;
}

#endif