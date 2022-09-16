#ifndef VERTEX
#define VERTEX

struct Vertex{
    int id;
    int type; //0=depot, 1=client
    float initialTime;
    float endTime;
    float x;
    float y;
    int request;
    int serviceTimeTruck;
	int serviceTimeDrone;
	bool drone;
};

#endif