#ifndef ROUTE
#define ROUTE

#include <vector>

using namespace std;

struct Route{
	int idTruck;
	float currentCapacity;
	vector<int> pathTruck;
	vector<int> pathsDrone;
    double * attended;
    double cost;

    Route(int id, float capacity, int numberOfClients){
        idTruck = id;
        currentCapacity = capacity;
        pathTruck.push_back(0);
        pathTruck.push_back(numberOfClients-1);
        attended = new double[numberOfClients];
        for(int i=0;i<numberOfClients;i++){
            attended[i] = 0.0;
        }
        cost=0.0;
    }

    Route(Route * r, int numberOfClients){
        idTruck = r->idTruck;
        currentCapacity = r->currentCapacity;
        
        for(int i=0;i<r->pathsDrone.size();i++){
            pathsDrone.push_back(r->pathsDrone[i]);
        }

        for(int i=0;i<r->pathTruck.size();i++){
            pathTruck.push_back(r->pathTruck[i]);
        }
        
        attended = new double[numberOfClients];
        for(int i=0;i<numberOfClients;i++){
            attended[i] = r->attended[i];
        }
        cost=r->cost;
    }

    //construtor para guloso
    Route(int id, float capacity, int numberOfClients, bool greed){
        idTruck = id;
        currentCapacity = capacity;
        pathTruck.push_back(0);
        attended = new double[numberOfClients];
        for(int i=0;i<numberOfClients;i++){
            attended[i] = 0.0;
        }
        cost=0.0;
    }

    ~Route(){
        pathTruck.clear();
        pathsDrone.clear();
        delete[](attended);
    }
};

#endif