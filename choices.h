#ifndef CHOICES
#define CHOICES

#include <vector>
#include "vertex.h"

using namespace std;

struct Choice{
    int truck;
    int posInsert;
    int posBack;
};

struct Choice2{
    int truck;
    float cost;
	vector<int> drones;
};

struct Choice3{
    int truck;
    int posInsert;
    int posBack;
    float cost;
};

bool compareClosingTimeWindow(Vertex v1, Vertex v2){
	return v1.endTime < v2.endTime;
}

bool compareWeight(Vertex v1, Vertex v2){
    return v1.request > v2.request;
}

bool compareCost(Choice2 v1, Choice2 v2){
	return v1.cost < v2.cost;
}

bool compareCostChoice3(Choice3 v1, Choice3 v2){
    return v1.cost < v2.cost;
}

#endif