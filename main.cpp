#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <queue>
#include <list>
#include "gurobi_c++.h"
#include "vertex.h"
#include "route.h"
#include "solution.h"
#include "choices.h"
#include "utils.h"

using namespace std;

vector<Vertex> vertex;
float capacityTruck, velocityDrone, velocityTruck, costPerTimeDrone, costPerTimeTruck, maxAwaitTime, distMaxDrone;
vector<vector<float>> graphTruck;
vector<vector<float>> graphDrone;

float calcCostInsertInTruckPath(Route * t, int posInsert, int idVertexInsert){
	return costPerTimeTruck*(graphTruck[t->pathTruck[posInsert-1]][idVertexInsert] + graphTruck[idVertexInsert][t->pathTruck[posInsert]] - graphTruck[t->pathTruck[posInsert-1]][t->pathTruck[posInsert]]);
}

float calcCostInsertInDronePath(Solution *s, int truck, int posOut, int posBack, Vertex *v){
	return costPerTimeDrone*(graphDrone[s->routes[truck]->pathTruck[posOut]][v->id] + graphDrone[v->id][s->routes[truck]->pathTruck[posBack]]);
}

void separeClientes(list<Vertex> &vertexTruck, list<Vertex> &vertexDrone){
	for(int i=1;i<vertex.size()-1;i++){
		if(vertex[i].drone){
			vertexDrone.push_back(vertex[i]);
		}else{
			vertexTruck.push_back(vertex[i]);
		}
	}
}

bool insertInTruckPathViable(Solution * s, int truck, int posInsert, Vertex * insert){
	if(s->routes[truck]->currentCapacity < insert->request){
		return false;
	}
	float time = getTimeOut(s,truck,posInsert-1,&graphTruck);
	time += graphTruck[s->routes[truck]->pathTruck[posInsert-1]][insert->id];
	if(time > vertex[insert->id].endTime){
		return false;
	}
	if(time < vertex[insert->id].initialTime){
		time = vertex[insert->id].initialTime;
	}
	time += insert->serviceTimeTruck;
	time += graphTruck[insert->id][s->routes[truck]->pathTruck[posInsert]];
	for(int i=posInsert;i<s->routes[truck]->pathTruck.size()-1;i++){
		float timeWindowInitialDrone = getTimeWindowInitialDrone(s,truck,s->routes[truck]->pathTruck[i],&graphTruck,&graphDrone,vertex.size());
		if(timeWindowInitialDrone != -1 && time > timeWindowInitialDrone){
			return false;
		}
		if(time <= getTimeIn(s, truck, i, &graphTruck)){
			return true;
		}
		if(time > vertex[s->routes[truck]->pathTruck[i]].endTime){
			return false;
		}
		time+=vertex[s->routes[truck]->pathTruck[i]].serviceTimeTruck;
		float timeWindowEndDrone = getTimeWindowEndDrone(s,truck,s->routes[truck]->pathTruck[i],&graphTruck,&graphDrone, vertex.size());
		if(time < timeWindowEndDrone){
			time = timeWindowEndDrone;
		}
		time+=graphTruck[s->routes[truck]->pathTruck[i]][s->routes[truck]->pathTruck[i+1]];
	}

	float timeDrone = getTimeWindowInitialDrone(s,truck,vertex.back().id,&graphTruck,&graphDrone,vertex.size());
	if(time > timeDrone && timeDrone!=-1){
		return false;
	}
	return true;
}

bool createDroneMovementIsViable(Solution * s, int truck, int posOut, int posBack, Vertex * v){
	if(s->routes[truck]->currentCapacity < v->request){
		return false;
	}
	float dist = graphDrone[ s->routes[truck]->pathTruck[posOut] ][ v->id ] + graphDrone[ v->id  ][s->routes[truck]->pathTruck[posBack]];
	if(dist > distMaxDrone){
		return false;
	}
	float timeMin = getTimeIn(s,truck,posOut,&graphTruck) + graphDrone[s->routes[truck]->pathTruck[posOut]][v->id];
	//TODO: Melhoria
	float timeMax = getTimeOut(s,truck,posOut,&graphTruck) + graphDrone[s->routes[truck]->pathTruck[posOut]][v->id] + maxAwaitTime;

	if(v->endTime < timeMin){
		return false;
	}
	if(v->initialTime > timeMax){
		return false;
	}
	if(timeMin < v->initialTime){
		timeMin = v->initialTime;
	}
	if(timeMax > v->endTime){
		timeMax = v->endTime;
	}
	timeMin+=v->serviceTimeDrone + graphDrone[v->id][s->routes[truck]->pathTruck[posBack]];
	timeMax+=v->serviceTimeDrone + graphDrone[v->id][s->routes[truck]->pathTruck[posBack]]+maxAwaitTime;
	
	float timeOutCompare = getTimeOut(s,truck, posBack,&graphTruck);
	//TODO: melhoria
	if(timeMin > timeOutCompare){
		return false;
	}
	if(timeMax < getTimeIn(s,truck,posBack,&graphTruck)){
		return false;
	}

	return true;
}

void getMovementsPossiblesByTruck(Solution * s, Vertex * v, float * minCost, vector<Choice> * choices){
	float cost;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathTruck.size();j++){
			cost = calcCostInsertInTruckPath(s->routes[i],j,v->id);
			if(cost <= *minCost){
				if(insertInTruckPathViable(s,i,j,v)){
					Choice aux;
					aux.posInsert=j;
					aux.posBack=-1;
					aux.truck=i;
					if(cost < *minCost){
						choices->clear();
						*minCost = cost;
					}
					choices->push_back(aux);
				}
			}
			if(getTimeOut(s,i,j,&graphTruck) > v->endTime){
				break;
			}
		}
	}
}

void getMovementsPossiblesByDrone(Solution *s, Vertex *v, float *minCost, vector<Choice> *choices){
	for(int i=0;i<s->routes.size();i++){
		for(int j=0;j<s->routes[i]->pathTruck.size();j++){
			if(getTimeOut(s,i,j,&graphTruck) > v->endTime){
				break;
			}
			for(int k=j;k<s->routes[i]->pathTruck.size();k++){
				float cost = calcCostInsertInDronePath(s,i,j,k,v); 
				if(cost <= *minCost){
					if(createDroneMovementIsViable(s,i,j,k,v)){
						Choice aux;
						aux.truck = i;
						aux.posInsert = j;
						aux.posBack = k;
						if(cost<*minCost){
							choices->clear();
							*minCost=cost;
						}
						choices->push_back(aux);
					}
				}
			}
		}
	}
}

void makeMovementTruck(Solution * s, int truck, int posInsert, int insert){
	s->routes[truck]->currentCapacity-=vertex[insert].request;
	s->routes[truck]->cost+=calcCostInsertInTruckPath(s->routes[truck],posInsert,insert);
	s->routes[truck]->attended[insert]=1.0;
	float time = getTimeOut(s,truck,posInsert-1,&graphTruck);
	s->routes[truck]->pathTruck.insert(s->routes[truck]->pathTruck.begin()+posInsert, insert);
	time+=graphTruck[s->routes[truck]->pathTruck[posInsert-1]][insert];
	s->timeIn[s->routes[truck]->pathTruck[posInsert]] = time;
	if(time < vertex[s->routes[truck]->pathTruck[posInsert]].initialTime){
		time = vertex[s->routes[truck]->pathTruck[posInsert]].initialTime;
	}
	time+=vertex[insert].serviceTimeTruck;
	s->timeOut[s->routes[truck]->pathTruck[posInsert]] = time;
	time+=graphTruck[s->routes[truck]->pathTruck[posInsert]][s->routes[truck]->pathTruck[posInsert+1]];
	for(int i=posInsert+1; i<s->routes[truck]->pathTruck.size()-1; i++){
		float timeIn = getTimeIn(s,truck,i,&graphTruck);
		s->timeIn[s->routes[truck]->pathTruck[i]]=time;
		if(time <= timeIn){
			break;
		}
		time+=vertex[s->routes[truck]->pathTruck[i]].serviceTimeTruck;
		if(time <= getTimeOut(s,truck,i,&graphTruck)){
			break;
		}
		s->timeOut[s->routes[truck]->pathTruck[i]] = time;
		time += graphTruck[s->routes[truck]->pathTruck[i]][s->routes[truck]->pathTruck[i+1]];
	}
}

void GetTimesInsertInDrone(Solution *s, int truck, int posOut, int posBack, int insert, float * timeInInsert, float * timeOutInsert, float * timeWindowBack, float * timeWindowOut){
	float timeOut = getTimeIn(s,truck,posOut,&graphTruck);
	float time = timeOut+graphDrone[s->routes[truck]->pathTruck[posOut]][insert];
	float await = 0;
	if(time < vertex[insert].initialTime){
		await = vertex[insert].initialTime - time;
		if(vertex[insert].initialTime - time > maxAwaitTime ){
			await = maxAwaitTime;
			timeOut+=vertex[insert].initialTime - time - maxAwaitTime;
		}
		time = vertex[insert].initialTime;
	}
	*timeInInsert=time;
	time+= vertex[insert].serviceTimeDrone;
	*timeOutInsert = time;
	time += graphDrone[insert][s->routes[truck]->pathTruck[posBack]];
	
	float timeInBack = getTimeIn(s,truck,posBack,&graphTruck);
	if(time < timeInBack){
		if((timeInBack - time) > maxAwaitTime){
			*timeInInsert += (timeInBack - time - maxAwaitTime);
			*timeOutInsert += (timeInBack - time - maxAwaitTime);
			timeOut += max((timeInBack - time - maxAwaitTime) - (maxAwaitTime-await),0.0f);
		}
		time = timeInBack;
	}
	*timeWindowBack = time;
	*timeWindowOut = timeOut;
}

void makeMovementDrone(Solution *s, int truck, int posOut, int posBack, int insert){
	s->routes[truck]->attended[insert]=1.0;
	s->routes[truck]->currentCapacity-=vertex[insert].request;
	s->routes[truck]->cost+=calcCostInsertInDronePath(s,truck,posOut,posBack,&vertex[insert]);
	s->routes[truck]->pathsDrone.push_back(s->routes[truck]->pathTruck[posOut]);
	s->routes[truck]->pathsDrone.push_back(insert);
	s->routes[truck]->pathsDrone.push_back(s->routes[truck]->pathTruck[posBack]);
	
	float timeInInsert, timeOutInsert, timeWindowBack, timeWindowOut;

	GetTimesInsertInDrone(s, truck, posOut, posBack, insert, &timeInInsert, &timeOutInsert, &timeWindowBack, &timeWindowOut);

	s->timeIn[insert]=timeInInsert;
	s->timeOut[insert] = timeOutInsert;
	updateTimeWindowDrone(s,s->routes[truck]->pathTruck[posBack],timeWindowBack);
	updateTimeWindowDrone(s,s->routes[truck]->pathTruck[posOut],timeWindowOut);
}

void FLS(Solution * s){
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathTruck.size()-1;j++){
			float timeIn = getTimeWindowInitialDrone(s,i,s->routes[i]->pathTruck[j],&graphTruck,&graphDrone,vertex.size());
			float timeOut = getTimeWindowEndDrone(s,i,s->routes[i]->pathTruck[j],&graphTruck,&graphDrone,vertex.size());
			bool drone = vertex[s->routes[i]->pathTruck[j]].drone;
			if(drone && timeIn==-1 && timeOut==-1){

				Vertex insert = vertex[s->routes[i]->pathTruck[j]];
				
				float timeIn = s->timeIn[insert.id];
				float timeOut = s->timeOut[insert.id];
				float capacity = s->routes[i]->currentCapacity;
				float cost = s->routes[i]->cost;
				s->timeIn[insert.id]=-1;
				s->timeOut[insert.id]=-1;
				s->routes[i]->currentCapacity+=insert.request;
				s->routes[i]->attended[insert.id] = 0.0;
				float costRemoved = costPerTimeTruck*(graphTruck[s->routes[i]->pathTruck[j-1]][s->routes[i]->pathTruck[j+1]]-graphTruck[s->routes[i]->pathTruck[j-1]][insert.id]-graphTruck[s->routes[i]->pathTruck[j+1]][insert.id]);
				s->routes[i]->cost+= costRemoved;
				
				float timeIn2 = s->timeIn[ s->routes[i]->pathTruck[j+1] ];
				s->timeIn[ s->routes[i]->pathTruck[j+1] ] = timeIn - graphTruck[s->routes[i]->pathTruck[j-1]][s->routes[i]->pathTruck[j]] + graphTruck[s->routes[i]->pathTruck[j-1]][s->routes[i]->pathTruck[j+1]];
				
				s->routes[i]->pathTruck.erase(s->routes[i]->pathTruck.begin()+j);

				float costInsert = MAXFLOAT;
				vector<Choice> choices;
				bool improved = false;
				getMovementsPossiblesByDrone(s,&insert,&costInsert,&choices);
				if(!choices.empty()){
					Choice c = choices[ rand()%choices.size() ];
					
					if(costInsert < -costRemoved){
						makeMovementDrone(s,c.truck,c.posInsert,c.posBack,insert.id);
						improved = true;
					}
					
				}
				if(!improved){
					s->timeIn[insert.id]=timeIn;
					s->timeOut[insert.id]=timeOut;
					s->routes[i]->currentCapacity=capacity;
					s->routes[i]->attended[insert.id] = 1;
					s->routes[i]->cost= cost;
					s->timeIn[ s->routes[i]->pathTruck[j] ] = timeIn2;
					s->routes[i]->pathTruck.insert(s->routes[i]->pathTruck.begin()+j,insert.id);
				}else{
					if(s->routes[i]->pathTruck.size() == 2 && s->routes[i]->pathsDrone.empty()){
						s->routes.erase(s->routes.begin()+i);
					}
				}
			}
		}
	}
}

Solution * firstPhase(list<Vertex> vertexTruck){
	Solution* s = new Solution(vertex.size(), capacityTruck);
	while (!vertexTruck.empty()){
		float minCost = MAXFLOAT;
		Vertex v = vertexTruck.front();
		vertexTruck.pop_front();
		vector<Choice> choices;
		getMovementsPossiblesByTruck(s,&v,&minCost, &choices);
		Choice c = choices[rand()%choices.size()];
		makeMovementTruck(s,c.truck,c.posInsert,v.id);
		if(c.truck == s->routes.size()-1){
			s->routes.push_back( new Route( s->routes.size(), capacityTruck, vertex.size() ) );
		}
	}
	if(s->routes.back()->pathTruck.size()==2){
		delete(s->routes.back());
		s->routes.pop_back();
	}
	return s;
}

void secondPhase(list<Vertex> vertexDrone, Solution *s, bool localSearch){
	s->routes.push_back( new Route( s->routes.size(), capacityTruck, vertex.size() ) );
	while(!vertexDrone.empty()){
		list<Vertex>::iterator i;
		int pos = rand() % vertexDrone.size();
		int j=0;
		for(i = vertexDrone.begin(); j<pos; j++, i++);
		float minCost = MAXFLOAT;
		vector<Choice> choices;
		getMovementsPossiblesByTruck(s,&(*i),&minCost, &choices);
		getMovementsPossiblesByDrone(s,&(*i),&minCost,&choices);
		Choice c = choices[ rand()%choices.size() ];
		if(c.posBack==-1){
			makeMovementTruck(s,c.truck,c.posInsert,(*i).id);
		}else{
			makeMovementDrone(s,c.truck,c.posInsert,c.posBack,(*i).id);
		}
		vertexDrone.erase(i);
		if(c.truck == s->routes.size()-1){
			s->routes.push_back( new Route( s->routes.size(), capacityTruck, vertex.size() ) );
		}
	}
	if(s->routes.back()->pathTruck.size() == 2 && s->routes.back()->pathsDrone.empty()){
		delete(s->routes.back());
		s->routes.pop_back();
	}
	if(localSearch){
		FLS(s);
	}
}

Solution * TPH(list<Vertex> truck, list<Vertex> drone, bool localSearch){
	Solution * s = firstPhase(truck);
	secondPhase(drone,s,localSearch);
	return s;
}

float costInsertTruckJ(Route * s, int posInsert, int idVertexInsert){
	return costPerTimeTruck*(graphTruck[s->pathTruck[posInsert-1]][idVertexInsert] + graphTruck[idVertexInsert][s->pathTruck[posInsert]]);
}

list<Vertex>::iterator selectNode(Solution * s, list<Vertex> Q, list<Vertex> &vertexDrone){
	vector<Vertex> cd;
	vector< list<Vertex>::iterator > choices;
	for(list<Vertex>::iterator it = vertexDrone.begin(); it != vertexDrone.end(); it++){
		bool contains = false;
		for(list<Vertex>::iterator itQ = Q.begin(); itQ != Q.end(); itQ++){
			if((*itQ).id == (*it).id){
				contains = true;
				break;
			}
		}
		if(!contains){
			bool exists = false;
			for(int i=0;i<s->routes.size();i++){
				for(int j=0;j<s->routes[i]->pathTruck.size();j++){
					for(int k=j;k<s->routes[i]->pathTruck.size();k++){
						if(graphDrone[j][(*it).id]+graphDrone[(*it).id][k] <= distMaxDrone){
							exists = true;
							break;
						}
					}
					if(exists){
						break;
					}
				}
				if(exists){
					break;
				}
			}
			if(!exists){
				cd.push_back(*it);
			}
		}
	}
	if(cd.empty()){
		for(list<Vertex>::iterator it = vertexDrone.begin(); it != vertexDrone.end(); it++){
			bool inQ = false;
			for(list<Vertex>::iterator itQ = Q.begin(); itQ != Q.end(); itQ++){
				if((*itQ).id == (*it).id){
					inQ=true;
					break;
				}
			}
			if(!inQ){
				choices.push_back(it);
			}
		}
	}else{
		for(list<Vertex>::iterator it = vertexDrone.begin(); it != vertexDrone.end(); it++){
			bool inCd = false;
			for(int i=0;i<cd.size();i++){
				if(cd[i].id == (*it).id){
					inCd = true;
					break;
				}
			}
			if(!inCd){
				choices.push_back(it);
			}
		}
	}
	float minCost = MAXFLOAT;
	list<Vertex>::iterator best;
	for(int i=0;i<choices.size();i++){
		for(int j=0;j<s->routes.size();j++){
			for(int k=1;k<s->routes[j]->pathTruck.size();k++){
				float cost = costInsertTruckJ(s->routes[j],k,(*choices[i]).id);
				if(cost < minCost){
					minCost = cost;
					best = choices[i];
				}
			}
		}
	}
	if(!choices.empty()){
		return best;
	}else{
		return vertexDrone.end();
	}
}

Solution * MSTPH(int iterations, bool localSearch){
	list<Vertex> vertexTruck, vertexDrone, Q;
	separeClientes(vertexTruck,vertexDrone);
	vector<Solution * > s;
	s.push_back(TPH(vertexTruck, vertexDrone, localSearch));
	Solution * best = s.back();
	float cost;
	float bestCost = calcCostSolution(best);
	int i=0;
	list<Vertex>::iterator it;
	while( i < iterations && !vertexDrone.empty() ){ 
		it = selectNode(s.back(),Q,vertexDrone);
		if(it==vertexDrone.end()){
			break;
		}
		Vertex removed = *it;
		vertexTruck.push_back(removed);
		vertexDrone.erase(it);
		s.push_back( TPH(vertexTruck,vertexDrone, localSearch) );
		cost = calcCostSolution(s.back());
		if(cost < bestCost){
			best = s.back();
			bestCost = cost;
			i=0;
		}else{
			i++;
		}
		for(list<Vertex>::iterator j=vertexTruck.begin(); j!=vertexTruck.end(); j++){
			if((*j).drone && (*j).id!=removed.id){
				list<Vertex> auxTruck(vertexTruck), auxDrone(vertexDrone);
				list<Vertex>::iterator k;
				for(k=auxTruck.begin(); (*k).id != (*j).id; k++);
				auxDrone.push_back(*k);
				auxTruck.erase(k);
				s.push_back(TPH(auxTruck,auxDrone, localSearch));
				cost = calcCostSolution(s.back());
				if(cost < bestCost){
					best = s.back();
					bestCost = cost;
					i=0;
					Q.push_back(*j);
					vertexDrone.push_back(*j);
					j = vertexTruck.erase(j);
					j--;
				}else{
					i++;
				}
			}
		}
	}
	for(int i=0;i<s.size();i++){
		if(s.at(i)!=best){
			delete(s.at(i));
		}
	}
	return best;
}

list<Vertex> VRPTW (Solution * s){
	list<Vertex> vertexDrone;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathsDrone.size();j=j+3){
			s->timeIn[s->routes[i]->pathsDrone[j]]=-1.0;
			s->timeOut[s->routes[i]->pathsDrone[j]]=-1.0;
			s->timeWindowEndDrone[s->routes[i]->pathsDrone[j-1]]=-1.0;
			s->timeWindowEndDrone[s->routes[i]->pathsDrone[j+1]]=-1.0;
			s->timeWindowInitialDrone[s->routes[i]->pathsDrone[j-1]]=-1.0;
			s->timeWindowInitialDrone[s->routes[i]->pathsDrone[j+1]]=-1.0;
			s->routes[i]->attended[s->routes[i]->pathsDrone[j]] = 0.0;
			s->routes[i]->cost-=costPerTimeDrone*(graphDrone[s->routes[i]->pathsDrone[j-1]][s->routes[i]->pathsDrone[j]]+graphDrone[s->routes[i]->pathsDrone[j]][s->routes[i]->pathsDrone[j+1]]);
			s->routes[i]->currentCapacity+=vertex[s->routes[i]->pathsDrone[j]].request;
			vertexDrone.push_back(vertex[s->routes[i]->pathsDrone[j]]);
		}
		s->routes[i]->pathsDrone.clear();
	}
	return vertexDrone;
}

Vertex removeClientFromRoute(Solution *s, int truck, int posMove){
	s->timeIn[s->routes[truck]->pathTruck[posMove+1]]=getTimeOut(s,truck,posMove-1,&graphTruck)+graphTruck[s->routes[truck]->pathTruck[posMove-1]][s->routes[truck]->pathTruck[posMove+1]];
	s->timeIn[s->routes[truck]->pathTruck[posMove]]=-1;
	s->timeOut[s->routes[truck]->pathTruck[posMove]]=-1;
	s->routes[truck]->attended[s->routes[truck]->pathTruck[posMove]] = 0.0;
	float costWithRemoved = costPerTimeTruck * ( graphTruck[s->routes[truck]->pathTruck[posMove-1]][s->routes[truck]->pathTruck[posMove]]+graphTruck[s->routes[truck]->pathTruck[posMove]][s->routes[truck]->pathTruck[posMove+1]] );
	float costWithoutRemoved = costPerTimeTruck * ( graphTruck[s->routes[truck]->pathTruck[posMove-1]][s->routes[truck]->pathTruck[posMove+1]] );
	s->routes[truck]->cost += -costWithRemoved + costWithoutRemoved;
	s->routes[truck]->currentCapacity += vertex[s->routes[truck]->pathTruck[posMove]].request;
	Vertex v = vertex[s->routes[truck]->pathTruck[posMove]];
	s->routes[truck]->pathTruck.erase(s->routes[truck]->pathTruck.begin()+posMove);
	if(s->routes[truck]->pathTruck.size()==2 && s->routes[truck]->pathsDrone.empty()){
		s->routes.erase(s->routes.begin()+truck);
	}
	return v;
}

bool TryMakeMovementIntraRoute(Solution * s, int truck, int posMove){
	Vertex v = removeClientFromRoute(s,truck,posMove);
	for(int i=posMove-1;i>=1;i--){
		if(insertInTruckPathViable(s,truck,i,&v)){
			makeMovementTruck(s,truck,i,v.id);
			return true;
		}
	}
	for(int i=posMove+1;i<s->routes[truck]->pathTruck.size()-1;i++){
		if(insertInTruckPathViable(s,truck,i,&v)){
			makeMovementTruck(s,truck,i,v.id);
			return true;
		}
	}
	return false;
}

bool TryMakeMovementBetweenRoutes(Solution * s, int truck, int posMove, int truckMove){
	Vertex v = removeClientFromRoute(s,truck,posMove);
	for(int i=1 ; i<s->routes[truckMove]->pathTruck.size()-1 ; i++){
		if(insertInTruckPathViable(s,truckMove,i,&v)){
			makeMovementTruck(s,truckMove,i,v.id);
			return true;
		}
	}
	return false;
}

Solution* LS(Solution *s){
	float costOriginal = calcCostSolution(s);
	Solution * original = new Solution(s, vertex.size());
	list<Vertex> vertexDrone = VRPTW(s);
	Solution * solutionWithoutDrones = new Solution(s, vertex.size());
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathTruck.size()-1;j++){
			if(TryMakeMovementIntraRoute(s,i,j)){	
				secondPhase(vertexDrone, s, true);
				if(calcCostSolution(s) < costOriginal){
					delete(original);
					delete(solutionWithoutDrones);
					return s;
				}else{
					delete(s);
					s = new Solution(solutionWithoutDrones,vertex.size());
				}
			}else{
				delete(s);
				s = new Solution(solutionWithoutDrones,vertex.size());
			}
		}
	}

	for(int i=0;i<s->routes.size();i++){
		for(int j=0;j<s->routes.size();j++){
			if(i!=j){
				for(int k=1;k<s->routes[i]->pathTruck.size()-1;k++){
					if(TryMakeMovementBetweenRoutes(s,i,k,j)){
						secondPhase(vertexDrone,s,true);
						if(calcCostSolution(s) < costOriginal){
							delete(original);
							delete(solutionWithoutDrones);
							return s;
						}else{
							delete(s);
							s = new Solution(solutionWithoutDrones,vertex.size());
						}
					}else{
						delete(s);
						s = new Solution(solutionWithoutDrones,vertex.size());
					}
				}
			}
		}
	}

	delete(solutionWithoutDrones);
	delete(s);
	return original;
}

Solution* makeLS(Solution * s){
	float cost = calcCostSolution(s);
	float costBefore;
	do{
		costBefore = cost;
		s = LS(s);
		cost = calcCostSolution(s);
	}while(cost<costBefore);
	return s;
}

Solution * TPH2(list<Vertex> truck, list<Vertex> drone, int maxIterations){
	Solution * s = firstPhase(truck);
	secondPhase(drone,s,true);
	float costBest = MAXFLOAT;
	float cost;
	for(int i=0;i<maxIterations;i++){
		cost = calcCostSolution(s);
		if(cost < costBest){
			costBest = cost;
			i=0;
		}
		s = makeLS(s);
	}
	return s;
}

Solution * MSTPH2(int iterations, int iterationsWitoutImprovement){
	list<Vertex> vertexTruck, vertexDrone, Q;
	separeClientes(vertexTruck,vertexDrone);
	vector<Solution * > s;
	s.push_back(TPH2(vertexTruck, vertexDrone, iterationsWitoutImprovement));
	Solution * best = s.back();
	float cost;
	float bestCost = calcCostSolution(best);
	int i=0;
	list<Vertex>::iterator it;
	while( i < iterations && !vertexDrone.empty() ){
		it = selectNode(s.back(),Q,vertexDrone);
		if(it==vertexDrone.end()){
			break;
		}
		Vertex removed = *it;
		vertexTruck.push_back(removed);
		vertexDrone.erase(it);
		s.push_back( TPH2(vertexTruck,vertexDrone, iterationsWitoutImprovement) );
		cost = calcCostSolution(s.back());
		if(cost < bestCost){
			best = s.back();
			bestCost = cost;
			i=0;
		}else{
			i++;
		}
		for(list<Vertex>::iterator j=vertexTruck.begin(); j!=vertexTruck.end(); j++){
			if((*j).drone && (*j).id!=removed.id){
				list<Vertex> auxTruck(vertexTruck), auxDrone(vertexDrone);
				list<Vertex>::iterator k;
				for(k=auxTruck.begin(); (*k).id != (*j).id; k++);
				auxDrone.push_back(*k);
				auxTruck.erase(k);
				s.push_back(TPH2(auxTruck,auxDrone, iterationsWitoutImprovement));
				cost = calcCostSolution(s.back());
				if(cost < bestCost){
					best = s.back();
					bestCost = cost;
					i=0;
					Q.push_back(*j);
					vertexDrone.push_back(*j);
					j = vertexTruck.erase(j);
					j--;
				}else{
					i++;
				}
			}
		}
	}
	for(int i=0;i<s.size();i++){
		if(s.at(i)!=best){
			delete(s.at(i));
		}
	}
	return best;
}

void getMovementsPossiblesByTruckGreedy(Solution * s, Vertex * v, vector<Choice3> * choices){
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathTruck.size();j++){
			if(insertInTruckPathViable(s,i,j,v)){
				Choice3 aux;
				aux.posInsert=j;
				aux.posBack=-1;
				aux.truck=i;
				aux.cost=calcCostInsertInTruckPath(s->routes[i],j,v->id);
				choices->push_back(aux);
			}
			if(getTimeOut(s,i,j,&graphTruck) > v->endTime){
				break;
			}
		}
	}
}

void getMovementsPossiblesByDroneGreedy(Solution *s, Vertex *v, vector<Choice3> *choices){
	for(int i=0;i<s->routes.size();i++){
		for(int j=0;j<s->routes[i]->pathTruck.size();j++){
			if(getTimeOut(s,i,j,&graphTruck) > v->endTime){
				break;
			}
			for(int k=j;k<s->routes[i]->pathTruck.size();k++){
				if(createDroneMovementIsViable(s,i,j,k,v)){
					Choice3 aux;
					aux.truck = i;
					aux.posInsert = j;
					aux.posBack = k;
					aux.cost = calcCostInsertInDronePath(s,i,j,k,v);
					choices->push_back(aux);
				}
			}
		}
	}
}

Solution * greedyInsertMiddle(vector<Vertex> v, float alpha, bool (* compare)(Vertex, Vertex)){
	bool * attended = new bool[v.size()];
	int numberAttended = 0;
	for(int i=0;i<v.size();i++){
		attended[i]=false;
	}
	Solution *s = new Solution(v.size(), capacityTruck);
	sort(v.begin()+1,v.end()-1,compare);
	for(int i=1;numberAttended < v.size()-2;){
		while(attended[v[i].id]){
			i++;
		}
		int qtd = rand() % min((int)ceil((v.size()-2)*alpha), (int)v.size()-2-numberAttended);
		int clientAttended = i;
		while (qtd > 0){
			if(!attended[v[clientAttended].id]){
				qtd--;
				clientAttended++;
			}
			while(attended[v[clientAttended].id]){
				clientAttended++;
			}
		}
		vector<Choice3> choices;
		getMovementsPossiblesByTruckGreedy(s, &v[clientAttended], &choices);
		if(v[clientAttended].drone){
			getMovementsPossiblesByDroneGreedy(s,&v[clientAttended],&choices);
		}
		sort(choices.begin(),choices.end(),compareCostChoice3);
		Choice3 choice = choices[rand() % ((int)floor(choices.size()*alpha)+1)];
		if(choice.posBack==-1){
			makeMovementTruck(s,choice.truck,choice.posInsert,v[clientAttended].id);
		}else{
			makeMovementDrone(s,choice.truck,choice.posInsert,choice.posBack,v[clientAttended].id);
		}
		attended[v[clientAttended].id] = true;
		numberAttended++;
		if(choice.truck == s->routes.size()-1){
			s->routes.push_back( new Route( s->routes.size(), capacityTruck, vertex.size() ) );
		}
	}
	if(s->routes.back()->pathTruck.size() == 2 && s->routes.back()->pathsDrone.empty()){
		delete(s->routes.back());
		s->routes.pop_back();
	}
	delete[] attended;
	//solutionIsViable(s,&graphTruck,&graphDrone,&vertex,distMaxDrone,maxAwaitTime,capacityTruck);
	return s;
}

string initParameters(string instance, int seed){
	srand(seed);
	vector<string> aux = split(instance,'/');

	//tempo maximo de espera = 10 para R e RC e 90 para C
	if(aux[aux.size()-1][0]=='R'){
		maxAwaitTime = 10;
	}else{
		maxAwaitTime = 90;
	}

	capacityTruck = readFile(instance, &vertex);
	velocityDrone=2;
	velocityTruck=1;
	costPerTimeDrone=1*velocityDrone;
	costPerTimeTruck=25*velocityTruck;
	
	makeGraph(&graphTruck, &graphDrone, &vertex, velocityDrone, velocityTruck);
	
	distMaxDrone = calcMaxDistDrone(&vertex, &graphDrone);
	return aux[aux.size()-1].substr(0,aux[aux.size()-1].size()-4);
}

bool truckCanAttend(Solution * s, int truck, Vertex * candidate, vector<float> &time){
	return (s->routes[truck]->currentCapacity >= candidate->request) && (time[truck]+graphTruck[s->routes[truck]->pathTruck.back()][candidate->id] <= candidate->endTime);
}

bool droneCanAttend(Solution *s, int truck, Vertex * candidate, Vertex * droneCandidate){
	if(!droneCandidate->drone){
		return false;
	}
	if(s->routes[truck]->currentCapacity < candidate->request + droneCandidate->request){
		return false;
	}
	if((graphDrone[s->routes[truck]->pathTruck.back()][droneCandidate->id] + graphDrone[droneCandidate->id][candidate->id]) > distMaxDrone){
		return false;
	}
	int pos = s->routes[truck]->pathTruck.back();
	float timeMin, timeMax;
	if(pos == 0){
		timeMin = 0;
		timeMax = 0;
	}else{
		timeMin = s->timeIn[pos];
		timeMax = s->timeOut[pos];
	}
	timeMin += graphDrone[pos][droneCandidate->id];
	timeMax += graphDrone[pos][droneCandidate->id] + maxAwaitTime;
	if(droneCandidate->initialTime > timeMax){
		return false;
	}
	if(droneCandidate->endTime < timeMin){
		return false;
	}
	if(droneCandidate->initialTime > timeMin){
		timeMin = droneCandidate->initialTime;
	}
	if(droneCandidate->endTime < timeMax){
		timeMax = droneCandidate->endTime;
	}
	timeMin+=droneCandidate->serviceTimeDrone;
	timeMax+=droneCandidate->serviceTimeDrone;
	timeMin+=graphDrone[droneCandidate->id][candidate->id];
	timeMax+=graphDrone[droneCandidate->id][candidate->id]+maxAwaitTime;
	float timeIn;
	if(pos==0){
		timeIn = graphTruck[pos][candidate->id];
	}else{
		timeIn = s->timeOut[pos]+graphTruck[pos][candidate->id];
	}
	float timeOut = timeIn;
	if(timeOut < candidate->initialTime){
		timeOut = candidate->initialTime;
	}
	timeOut+=candidate->serviceTimeTruck;

	if(timeMin > timeOut){
		return false;
	}
	if(timeMax < timeIn){
		return false;
	}
	return true;
}

void addNewRoute(Solution * s, vector<float> * time){
	s->routes.push_back(new Route(s->routes.size(),capacityTruck, vertex.size(), true));
	time->push_back(0.0);
}

void makeMovement(Solution * s, Choice2 * c, Vertex * v, vector<float> * time1, bool * attended){
	s->routes[c->truck]->attended[v->id] = 1.0;
	s->routes[c->truck]->cost+=graphTruck[s->routes[c->truck]->pathTruck.back()][v->id]*costPerTimeTruck;
	s->routes[c->truck]->currentCapacity-=v->request;
	attended[ v->id ] = true;
	float t = time1->at(c->truck)+graphTruck[s->routes[c->truck]->pathTruck.back()][v->id];
	s->timeIn[ v->id ] = t;
	if(t<v->initialTime){
		t = v->initialTime;
	}
	t += v->serviceTimeTruck;
	s->timeOut[v->id] = t;
	time1->at(c->truck) = t;
	int posOut = s->routes[c->truck]->pathTruck.back();
	s->routes[c->truck]->pathTruck.push_back(v->id);
	
	for(int i=0;i<c->drones.size();i++){
		s->routes[c->truck]->attended[ c->drones[i] ] = 1.0;
		attended[c->drones[i]] = true;
		s->routes[c->truck]->currentCapacity -= vertex[c->drones[i]].request;
		s->routes[c->truck]->cost+= costPerTimeDrone*(graphDrone[posOut][c->drones[i]]+graphDrone[c->drones[i]][v->id]);
		s->routes[c->truck]->pathsDrone.push_back(posOut);
		s->routes[c->truck]->pathsDrone.push_back(c->drones[i]);
		s->routes[c->truck]->pathsDrone.push_back(v->id);
		int posBack = v->id;
		float timeOut, time;
		if(posOut == 0){
			timeOut = 0;
		}else{
			timeOut = s->timeIn[posOut];
		}
		time = timeOut + graphDrone[posOut][c->drones[i]];
		s->timeIn[c->drones[i]]=time;
		float await = 0;
		if(time < vertex[c->drones[i]].initialTime){
			await = vertex[c->drones[i]].initialTime - time;
			if(await > maxAwaitTime ){
				timeOut += await - maxAwaitTime;
				s->timeIn[c->drones[i]] += await - maxAwaitTime;
				await = maxAwaitTime;
			}
			time = vertex[c->drones[i]].initialTime;
		}
		time+= vertex[c->drones[i]].serviceTimeDrone;
		s->timeOut[c->drones[i]] = time;
		time += graphDrone[c->drones[i]][posBack];
		float timeInBack = s->timeIn[ posBack ];
		if(time < timeInBack){
			if((timeInBack - time) > maxAwaitTime){
				s->timeIn[c->drones[i]] += (timeInBack - time - maxAwaitTime);
				s->timeOut[c->drones[i]] += (timeInBack - time - maxAwaitTime);
				timeOut += max((timeInBack - time - maxAwaitTime) - (maxAwaitTime-await),0.0f);
			}
			time = timeInBack;
		}
		updateTimeWindowDrone(s,posBack,time);
		updateTimeWindowDrone(s,posOut,timeOut);
	}
	
	if(c->truck == s->routes.size()-1){
		addNewRoute(s,time1);
	}
}

Choice3 getMovemetsPossiblesLSDrone(Solution * s, Vertex * v, float bestCost, bool firstImprovement){
	Choice3 best;
	best.cost=-1;
	best.posBack=-1;
	best.posInsert=-1;
	best.truck=-1;
	if(v->drone){
		float cost;
		for(int i=0;i<s->routes.size();i++){
			for(int j=0;j<s->routes[i]->pathTruck.size();j++){
				if(getTimeOut(s,i,j,&graphTruck) > v->endTime){
					break;
				}
				for(int k=j;k<s->routes[i]->pathTruck.size();k++){
					cost = calcCostInsertInDronePath(s,i,j,k,v);
					if(cost<bestCost && createDroneMovementIsViable(s,i,j,k,v)){
						best.truck = i;
						best.posInsert = j;
						best.posBack = k;
						best.cost = cost;
						if(firstImprovement){
							return best;
						}
						bestCost = cost;
					}
				}
			}
		}
	}
	return best;
}

Choice3 getMovemetsPossiblesLSDroneIntraRoute(Solution * s, Vertex * v, float bestCost, bool firstImprovement, int route){
	Choice3 best;
	best.cost=-1;
	best.posBack=-1;
	best.posInsert=-1;
	best.truck=-1;
	if(v->drone){
		float cost;
		for(int j=0;j<s->routes[route]->pathTruck.size();j++){
			if(getTimeOut(s,route,j,&graphTruck) > v->endTime){
				break;
			}
			for(int k=j;k<s->routes[route]->pathTruck.size();k++){
				cost = calcCostInsertInDronePath(s,route,j,k,v);
				if(cost<bestCost && createDroneMovementIsViable(s,route,j,k,v)){
					best.truck = route;
					best.posInsert = j;
					best.posBack = k;
					best.cost = cost;
					if(firstImprovement){
						return best;
					}
					bestCost = cost;
				}
			}
		}
	}
	return best;
}

Choice3 getMovemetsPossiblesLSDroneInterRoute(Solution * s, Vertex * v, float bestCost, bool firstImprovement, int route){
	Choice3 best;
	best.cost=-1;
	best.posBack=-1;
	best.posInsert=-1;
	best.truck=-1;
	if(v->drone){
		float cost;
		for(int i=0;i<s->routes.size();i++){
			if(i!=route){
				for(int j=0;j<s->routes[i]->pathTruck.size();j++){
					if(getTimeOut(s,i,j,&graphTruck) > v->endTime){
						break;
					}
					for(int k=j;k<s->routes[i]->pathTruck.size();k++){
						cost = calcCostInsertInDronePath(s,i,j,k,v);
						if(cost<bestCost && createDroneMovementIsViable(s,i,j,k,v)){
							best.truck = i;
							best.posInsert = j;
							best.posBack = k;
							best.cost = cost;
							if(firstImprovement){
								return best;
							}
							bestCost = cost;
						}
					}
				}
			}
		}
	}
	return best;
}


Choice3 getMovemetsPossiblesLSTruck(Solution * s, Vertex * v, float bestCost, bool firstImprovement){
	Choice3 best;
	best.cost = -1;
	best.posBack=-1;
	best.posInsert=-1;
	best.truck=-1;
	float cost;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathTruck.size();j++){
			cost = calcCostInsertInTruckPath(s->routes[i],j,v->id);
			if(cost<bestCost && insertInTruckPathViable(s,i,j,v)){
				best.posInsert=j;
				best.posBack=-1;
				best.truck=i;
				best.cost=cost;
				if(firstImprovement){
					return best;
				}
				bestCost = cost;
			}
			if(getTimeOut(s,i,j,&graphTruck) > v->endTime){
				break;
			}
		}
	}
	return best;
}

Choice3 getMovemetsPossiblesLSTruckIntraRoute(Solution * s, Vertex * v, float bestCost, bool firstImprovement, int route){
	Choice3 best;
	best.cost = -1;
	best.posBack=-1;
	best.posInsert=-1;
	best.truck=-1;
	float cost;
	for(int j=1;j<s->routes[route]->pathTruck.size();j++){
		cost = calcCostInsertInTruckPath(s->routes[route],j,v->id);
		if(cost<bestCost && insertInTruckPathViable(s,route,j,v)){
			best.posInsert=j;
			best.posBack=-1;
			best.truck=route;
			best.cost=cost;
			if(firstImprovement){
				return best;
			}
			bestCost = cost;
		}
		if(getTimeOut(s,route,j,&graphTruck) > v->endTime){
			break;
		}
	}
	return best;
}

Choice3 getMovemetsPossiblesLSTruckInterRoute(Solution * s, Vertex * v, float bestCost, bool firstImprovement, int route){
	Choice3 best;
	best.cost = -1;
	best.posBack=-1;
	best.posInsert=-1;
	best.truck=-1;
	float cost;
	for(int i=0;i<s->routes.size();i++){
		if(i!=route){
			for(int j=1;j<s->routes[i]->pathTruck.size();j++){
				cost = calcCostInsertInTruckPath(s->routes[i],j,v->id);
				if(cost<bestCost && insertInTruckPathViable(s,i,j,v)){
					best.posInsert=j;
					best.posBack=-1;
					best.truck=i;
					best.cost=cost;
					if(firstImprovement){
						return best;
					}
					bestCost = cost;
				}
				if(getTimeOut(s,i,j,&graphTruck) > v->endTime){
					break;
				}
			}
		}
	}
	return best;
}

Choice3 getMovemetsPossiblesLSDroneTruck(Solution * s, Vertex * v, float bestCost, bool firstImprovement){
	Choice3 bestT = getMovemetsPossiblesLSDrone(s,v,bestCost,firstImprovement);
	if(bestT.truck!=-1){
		if(firstImprovement){
			return bestT;
		}
		bestCost = bestT.cost;
	}
	Choice3 best = getMovemetsPossiblesLSTruck(s,v,bestCost,firstImprovement);
	if(best.truck!=-1){
		return best;
	}else{
		return bestT;
	}
}

void ArrumaTimeWindowDrone(Solution * s, int idVertex1, int idVertex2, int truck){
	s->timeWindowInitialDrone[idVertex1] = -1;
	s->timeWindowEndDrone[idVertex1] = -1;
	s->timeWindowInitialDrone[idVertex2] = -1;
	s->timeWindowEndDrone[idVertex2] = -1;
	
	int posVertex1=-1, posVertex2=-1;
	for (int i=0; i<s->routes[truck]->pathTruck.size(); i++){
		if(s->routes[truck]->pathTruck[i]==idVertex1){
			posVertex1 = i;
		}
		if(s->routes[truck]->pathTruck[i]==idVertex2){
			posVertex2 = i;
		}
	}

	float a, b, timeWindowOut, timeWindowBack;
	for(int i=1;i<s->routes[truck]->pathsDrone.size();i=i+3){
		int posOut = s->routes[truck]->pathsDrone[i-1];
		int posBack = s->routes[truck]->pathsDrone[i+1];
		int posAttended = s->routes[truck]->pathsDrone[i];
		int posAux = -1;
		if(posOut == idVertex1){
			for (int i=0; i<s->routes[truck]->pathTruck.size(); i++){
				if(s->routes[truck]->pathTruck[i]==posBack){
					posAux = i;
					break;
				}
			}
			GetTimesInsertInDrone(s, truck, posVertex1, posAux, posAttended, &a, &b, &timeWindowBack, &timeWindowOut);
			updateTimeWindowDrone(s, idVertex1, timeWindowOut);
		}
		if(posBack == idVertex1){
			for (int i=0; i<s->routes[truck]->pathTruck.size(); i++){
				if(s->routes[truck]->pathTruck[i]==posOut){
					posAux = i;
					break;
				}
			}
			GetTimesInsertInDrone(s, truck, posAux, posVertex1, posAttended, &a, &b, &timeWindowBack, &timeWindowOut);
			updateTimeWindowDrone(s, idVertex1, timeWindowBack);
		}

		if(posOut == idVertex2){
			for (int i=0; i<s->routes[truck]->pathTruck.size(); i++){
				if(s->routes[truck]->pathTruck[i]==posBack){
					posAux = i;
					break;
				}
			}
			GetTimesInsertInDrone(s, truck, posVertex2, posAux, posAttended, &a, &b, &timeWindowBack, &timeWindowOut);
			updateTimeWindowDrone(s, idVertex2, timeWindowOut);
		}
		if(posBack == idVertex2){
			for (int i=0; i<s->routes[truck]->pathTruck.size(); i++){
				if(s->routes[truck]->pathTruck[i]==posOut){
					posAux = i;
					break;
				}
			}
			GetTimesInsertInDrone(s, truck, posAux, posVertex2, posAttended, &a, &b, &timeWindowBack, &timeWindowOut);
			updateTimeWindowDrone(s, idVertex2, timeWindowBack);
		}
	}
}

float RemoveDroneNode(Solution *s, int truck, int posDroneRoute){
	int posOut = s->routes[truck]->pathsDrone[posDroneRoute-1];
	int posAttended = s->routes[truck]->pathsDrone[posDroneRoute];
	int posBack = s->routes[truck]->pathsDrone[posDroneRoute+1];
	float costBefore = costPerTimeDrone*(graphDrone[posOut][posAttended] + graphDrone[posAttended][posBack]);
	s->routes[truck]->pathsDrone.erase(s->routes[truck]->pathsDrone.begin()+(posDroneRoute-1));
	s->routes[truck]->pathsDrone.erase(s->routes[truck]->pathsDrone.begin()+(posDroneRoute-1));
	s->routes[truck]->pathsDrone.erase(s->routes[truck]->pathsDrone.begin()+(posDroneRoute-1));
	s->timeIn[posAttended] = -1;
	s->timeOut[posAttended] = -1;
	s->routes[truck]->attended[posAttended] = 0.0;
	s->routes[truck]->cost -= costBefore;
	s->routes[truck]->currentCapacity += vertex[posAttended].request;
	ArrumaTimeWindowDrone(s, posOut, posBack, truck);
	return costBefore;
}

float removeTruckNode(Solution *s, int truck, int posMove){
	s->timeIn[s->routes[truck]->pathTruck[posMove+1]]=getTimeOut(s,truck,posMove-1,&graphTruck)+graphTruck[s->routes[truck]->pathTruck[posMove-1]][s->routes[truck]->pathTruck[posMove+1]];
	s->timeIn[s->routes[truck]->pathTruck[posMove]]=-1;
	s->timeOut[s->routes[truck]->pathTruck[posMove]]=-1;
	s->routes[truck]->attended[s->routes[truck]->pathTruck[posMove]] = 0.0;
	float costWithRemoved = costPerTimeTruck * ( graphTruck[s->routes[truck]->pathTruck[posMove-1]][s->routes[truck]->pathTruck[posMove]]+graphTruck[s->routes[truck]->pathTruck[posMove]][s->routes[truck]->pathTruck[posMove+1]] );
	float costWithoutRemoved = costPerTimeTruck * ( graphTruck[s->routes[truck]->pathTruck[posMove-1]][s->routes[truck]->pathTruck[posMove+1]] );
	s->routes[truck]->cost += -costWithRemoved + costWithoutRemoved;
	s->routes[truck]->currentCapacity += vertex[s->routes[truck]->pathTruck[posMove]].request;
	Vertex v = vertex[s->routes[truck]->pathTruck[posMove]];
	s->routes[truck]->pathTruck.erase(s->routes[truck]->pathTruck.begin()+posMove);
	if(s->routes[truck]->pathTruck.size()==2 && s->routes[truck]->pathsDrone.empty()){
		s->routes.erase(s->routes.begin()+truck);
	}
	return costWithRemoved - costWithoutRemoved;
}

Solution* LSRemoveKNodes(Solution * s, int k, int tentativas){
	Solution * original = new Solution(s, vertex.size());
	vector<int> inserir;
 	for(int j=0;j<tentativas;j++){
		inserir.clear();
		float cost = 0;
		for(int i=0;i<k;i++){
			if(rand() % 2 == 0){
				int route = rand() % s->routes.size();
				if(s->routes[route]->pathsDrone.empty()){
					i--;
					continue;
				}
				int pos = rand() % (s->routes[route]->pathsDrone.size()/3);
				pos = pos*3+1;
				inserir.push_back(s->routes[route]->pathsDrone[pos]);
				cost += RemoveDroneNode(s, route, pos);
			}else{
				int route = rand() % s->routes.size();
				if(s->routes[route]->pathTruck.size() == 2){
					i--;
					continue;
				}
				
				int pos = (rand() % (s->routes[route]->pathTruck.size()-2)) + 1;
				if(getTimeWindowInitialDrone(s, route, s->routes[route]->pathTruck[pos],&graphTruck, &graphDrone, vertex.size()) != -1){
					i--;
					continue;
				}
				
				inserir.push_back(s->routes[route]->pathTruck[pos]);
				cost += removeTruckNode(s,route,pos);
			}
		}
		for(int i=0;i<inserir.size();i++){
			Choice3 choice = getMovemetsPossiblesLSDroneTruck(s, &vertex[inserir[i]], MAXFLOAT, true);
			if(choice.truck!=-1){
				if(choice.posBack==-1){
					makeMovementTruck(s,choice.truck,choice.posInsert,inserir[i]);
				}else{
					makeMovementDrone(s,choice.truck,choice.posInsert,choice.posBack,inserir[i]);
				}
				cost -= choice.cost;
			}else{
				cost = -1;
				break;
			}
		}
		if(cost > 0){
			delete(original);
			return s;
		}

		delete(s);
		s = new Solution(original, vertex.size());
	}
	delete(s);
	return original;
}

Solution* LSRemoveDroneNodeInsertDroneNodeIntraRoute(Solution * s, bool firstImprovement){
	Solution * original = new Solution(s, vertex.size());
	Solution * best = new Solution(s, vertex.size());
	float costBest = -1;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathsDrone.size();j=j+3){
			int posOut = s->routes[i]->pathsDrone[j-1];
			int posAttended = s->routes[i]->pathsDrone[j];
			int posBack = s->routes[i]->pathsDrone[j+1];
			float costBefore = RemoveDroneNode(s, i, j);
			Choice3 choice = getMovemetsPossiblesLSDroneIntraRoute(s, &vertex[posAttended], costBefore, firstImprovement, i);
			if(choice.truck != -1 && posAttended){
				makeMovementDrone(s,choice.truck,choice.posInsert,choice.posBack,posAttended);
				if(firstImprovement){
					delete(original);
					delete(best);
					return s;
				}
				if(costBest == -1 || choice.cost < costBest){
					costBest = choice.cost;
					delete(best);
					best = new Solution(s, vertex.size());
				}
			}
			delete(s);
			s = new Solution(original, vertex.size());
		}
	}
	delete(original);
	delete(s);
	return best;
}

Solution* LSRemoveDroneNodeInsertTruckNodeIntraRoute(Solution * s, bool firstImprovement){
	Solution * original = new Solution(s, vertex.size());
	Solution * best = new Solution(s, vertex.size());
	float costBest = -1;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathsDrone.size();j=j+3){
			int posOut = s->routes[i]->pathsDrone[j-1];
			int posAttended = s->routes[i]->pathsDrone[j];
			int posBack = s->routes[i]->pathsDrone[j+1];
			float costBefore = RemoveDroneNode(s, i, j);
			Choice3 choice = getMovemetsPossiblesLSTruckIntraRoute(s, &vertex[posAttended], costBefore, firstImprovement, i);
			if(choice.truck != -1 && posAttended){
				makeMovementTruck(s, choice.truck, choice.posInsert, posAttended);
				if(firstImprovement){
					delete(original);
					delete(best);
					return s;
				}
				if(costBest == -1 || choice.cost < costBest){
					costBest = choice.cost;
					delete(best);
					best = new Solution(s, vertex.size());
				}
			}
			delete(s);
			s = new Solution(original, vertex.size());
		}
	}
	delete(original);
	delete(s);
	return best;
}

Solution * LSRemoveTruckNodeInsertDroneNodeIntraRoute(Solution * s, bool firstImprovement){
	Solution * original = new Solution(s, vertex.size());
	Solution * best = new Solution(s, vertex.size());
	float improvemment = -MAXFLOAT;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathTruck.size()-1;j++){
			if(s->timeWindowInitialDrone[s->routes[i]->pathTruck[j]] == -1 && s->timeWindowEndDrone[s->routes[i]->pathTruck[j]] == -1){
				Vertex *v = &vertex[s->routes[i]->pathTruck[j]];
				float cost = removeTruckNode(s,i,j);
				Choice3 choice = getMovemetsPossiblesLSDroneIntraRoute(s, v, cost, firstImprovement, i);
				if(choice.truck!=-1){
					makeMovementDrone(s,choice.truck,choice.posInsert,choice.posBack,v->id);
					if(firstImprovement){
						delete(original);
						delete(best);
						return s;
					}
					if((cost - choice.cost) > improvemment){
						improvemment = cost - choice.cost;
						delete(best);
						best = new Solution(s, vertex.size());
					}
				}
				delete(s);
				s = new Solution(original, vertex.size());
			}
		}
	}
	delete(original);
	delete(s);
	return best;
}

Solution * LSRemoveTruckNodeInsertTruckNodeIntraRoute(Solution * s, bool firstImprovement){
	Solution * original = new Solution(s, vertex.size());
	Solution * best = new Solution(s, vertex.size());
	float improvemment = -MAXFLOAT;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathTruck.size()-1;j++){
			if(s->timeWindowInitialDrone[s->routes[i]->pathTruck[j]] == -1 && s->timeWindowEndDrone[s->routes[i]->pathTruck[j]] == -1){
				Vertex *v = &vertex[s->routes[i]->pathTruck[j]];
				float cost = removeTruckNode(s,i,j);
				Choice3 choice = getMovemetsPossiblesLSTruckIntraRoute(s, v, cost, firstImprovement, i);
				if(choice.truck!=-1){
					makeMovementTruck(s,choice.truck,choice.posInsert,v->id);
					if(firstImprovement){
						delete(original);
						delete(best);
						return s;
					}
					if((cost - choice.cost) > improvemment){
						improvemment = cost - choice.cost;
						delete(best);
						best = new Solution(s, vertex.size());
					}
				}
				delete(s);
				s = new Solution(original, vertex.size());
			}
		}
	}
	delete(original);
	delete(s);
	return best;
}

Solution* LSRemoveDroneNodeInsertDroneNodeInterRoute(Solution * s, bool firstImprovement){
	Solution * original = new Solution(s, vertex.size());
	Solution * best = new Solution(s, vertex.size());
	float costBest = -1;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathsDrone.size();j=j+3){
			int posOut = s->routes[i]->pathsDrone[j-1];
			int posAttended = s->routes[i]->pathsDrone[j];
			int posBack = s->routes[i]->pathsDrone[j+1];
			float costBefore = RemoveDroneNode(s, i, j);
			Choice3 choice = getMovemetsPossiblesLSDroneInterRoute(s, &vertex[posAttended], costBefore, firstImprovement, i);
			if(choice.truck != -1 && posAttended){
				makeMovementDrone(s,choice.truck,choice.posInsert,choice.posBack,posAttended);
				if(firstImprovement){
					delete(original);
					delete(best);
					return s;
				}
				if(costBest == -1 || choice.cost < costBest){
					costBest = choice.cost;
					delete(best);
					best = new Solution(s, vertex.size());
				}
			}
			delete(s);
			s = new Solution(original, vertex.size());
		}
	}
	delete(original);
	delete(s);
	return best;
}

Solution* LSRemoveDroneNodeInsertTruckNodeInterRoute(Solution * s, bool firstImprovement){
	Solution * original = new Solution(s, vertex.size());
	Solution * best = new Solution(s, vertex.size());
	float costBest = -1;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathsDrone.size();j=j+3){
			int posOut = s->routes[i]->pathsDrone[j-1];
			int posAttended = s->routes[i]->pathsDrone[j];
			int posBack = s->routes[i]->pathsDrone[j+1];
			float costBefore = RemoveDroneNode(s, i, j);
			Choice3 choice = getMovemetsPossiblesLSTruckInterRoute(s, &vertex[posAttended], costBefore, firstImprovement, i);
			if(choice.truck != -1 && posAttended){
				makeMovementTruck(s, choice.truck, choice.posInsert, posAttended);
				if(firstImprovement){
					delete(original);
					delete(best);
					return s;
				}
				if(costBest == -1 || choice.cost < costBest){
					costBest = choice.cost;
					delete(best);
					best = new Solution(s, vertex.size());
				}
			}
			delete(s);
			s = new Solution(original, vertex.size());
		}
	}
	delete(original);
	delete(s);
	return best;
}

Solution * LSRemoveTruckNodeInsertDroneNodeInterRoute(Solution * s, bool firstImprovement){
	Solution * original = new Solution(s, vertex.size());
	Solution * best = new Solution(s, vertex.size());
	float improvemment = -MAXFLOAT;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathTruck.size()-1;j++){
			if(s->timeWindowInitialDrone[s->routes[i]->pathTruck[j]] == -1 && s->timeWindowEndDrone[s->routes[i]->pathTruck[j]] == -1){
				Vertex *v = &vertex[s->routes[i]->pathTruck[j]];
				float cost = removeTruckNode(s,i,j);
				Choice3 choice = getMovemetsPossiblesLSDroneInterRoute(s, v, cost, firstImprovement, i);
				if(choice.truck!=-1){
					makeMovementDrone(s,choice.truck,choice.posInsert,choice.posBack,v->id);
					if(firstImprovement){
						delete(original);
						delete(best);
						return s;
					}
					if((cost - choice.cost) > improvemment){
						improvemment = cost - choice.cost;
						delete(best);
						best = new Solution(s, vertex.size());
					}
				}
				delete(s);
				s = new Solution(original, vertex.size());
			}
		}
	}
	delete(original);
	delete(s);
	return best;
}

Solution * LSRemoveTruckNodeInsertTruckNodeInterRoute(Solution * s, bool firstImprovement){
	Solution * original = new Solution(s, vertex.size());
	Solution * best = new Solution(s, vertex.size());
	float improvemment = -MAXFLOAT;
	for(int i=0;i<s->routes.size();i++){
		for(int j=1;j<s->routes[i]->pathTruck.size()-1;j++){
			if(s->timeWindowInitialDrone[s->routes[i]->pathTruck[j]] == -1 && s->timeWindowEndDrone[s->routes[i]->pathTruck[j]] == -1){
				Vertex *v = &vertex[s->routes[i]->pathTruck[j]];
				float cost = removeTruckNode(s,i,j);
				Choice3 choice = getMovemetsPossiblesLSTruckInterRoute(s, v, cost, firstImprovement, i);
				if(choice.truck!=-1){
					makeMovementTruck(s,choice.truck,choice.posInsert,v->id);
					if(firstImprovement){
						delete(original);
						delete(best);
						return s;
					}
					if((cost - choice.cost) > improvemment){
						improvemment = cost - choice.cost;
						delete(best);
						best = new Solution(s, vertex.size());
					}
				}
				delete(s);
				s = new Solution(original, vertex.size());
			}
		}
	}
	delete(original);
	delete(s);
	return best;
}

Solution * RVND(Solution * s, bool firstImprovement){
	vector<int> choices;
	choices.reserve(8);
	float cost = MAXFLOAT;
	float costLS = calcCostSolution(s);
	while (cost > costLS){
		if(s->routes.back()->pathTruck.size() > 2 || s->routes.back()->pathTruck.size() == 2 && !s->routes.back()->pathsDrone.empty()){
			s->routes.push_back(new Route(s->routes.size(), capacityTruck, vertex.size()));
		}
		cost = costLS;
		choices.clear();
		choices.push_back(0);
		choices.push_back(1);
		choices.push_back(2);
		choices.push_back(3);
		choices.push_back(4);
		choices.push_back(5);
		choices.push_back(6);
		choices.push_back(7);

		while(!choices.empty() && cost == costLS){
			int pos = rand()%choices.size();
			
			switch (choices[pos])
			{
				case 0:
					s = LSRemoveDroneNodeInsertTruckNodeInterRoute(s, firstImprovement);
					break;
				case 1:
					s = LSRemoveDroneNodeInsertDroneNodeInterRoute(s, firstImprovement);
					break;
				case 2:
					s = LSRemoveTruckNodeInsertTruckNodeInterRoute(s, firstImprovement);
					break;
				case 3:
					s = LSRemoveTruckNodeInsertDroneNodeInterRoute(s, firstImprovement);
					break;
				case 4:
					s = LSRemoveDroneNodeInsertTruckNodeIntraRoute(s, firstImprovement);
					break;
				case 5:
					s = LSRemoveDroneNodeInsertDroneNodeIntraRoute(s, firstImprovement);
					break;
				case 6:
					s = LSRemoveTruckNodeInsertTruckNodeIntraRoute(s, firstImprovement);
					break;
				case 7:
					s = LSRemoveTruckNodeInsertDroneNodeIntraRoute(s, firstImprovement);
					break;
			}

			choices.erase(choices.begin()+pos);
			costLS = calcCostSolution(s);
		}

		if(cost == costLS){
			s = LSRemoveKNodes(s, 2, (vertex.size()-1)/2.5);
			costLS = calcCostSolution(s);
			if(cost == costLS){
				s = LSRemoveKNodes(s, 3, (vertex.size()-1)/1.25);
				costLS = calcCostSolution(s);
			}
		}
	}

	if(s->routes.back()->pathTruck.size() == 2 && s->routes.back()->pathsDrone.empty()){
		s->routes.pop_back();
	}

	return s;
}

int chooseAlpha(vector<float> &probabilities, float probabiliti){
    unsigned int i;
    float sum;
    for(i=0,sum=0.0;sum<=probabiliti && i<probabilities.size();i++){
        sum=sum+probabilities[i];
    }
    return i-1;
}

Solution* GRASPReativo (int numberOfIterations, bool firstImprovement, int * melhorIteracao, string instancia, int seed){
	int numberOfLooping = 20;
	*melhorIteracao = 0;
	int iterationsPerLoop = numberOfIterations/numberOfLooping;
	vector<float> alphas;
	alphas.push_back(0.1);
	alphas.push_back(0.2);
	alphas.push_back(0.3);

	vector<float> probability(alphas.size(),100.0/alphas.size());
	vector<float> media(alphas.size());
	vector<float> aux(alphas.size());
	vector<float> sum(alphas.size(),0);
	vector<unsigned long int> times(alphas.size(),0);

	float sumAll;
	
	Solution * s;
	float cost;
	float costBest = MAXFLOAT;
	Solution * best = NULL;

	for(int i=0;i<numberOfLooping;i++){
		for(int j=0;j<iterationsPerLoop;j++){
			int pos = chooseAlpha(probability,rand()%100);
			
			s = greedyInsertMiddle(vertex, alphas[pos], compareWeight);
			s = RVND(s, firstImprovement);
			cost = calcCostSolution(s);
			
			times[pos]++;
			sum[pos]=sum[pos]+cost;

			if(cost < costBest){
				*melhorIteracao = i*iterationsPerLoop+j;
				if(best != NULL){
					delete(best);
				}
				best = s;
				costBest = cost;
			}else{
				delete(s);
			}
			cout << instancia << ";" << seed << ";" << costBest << ";" << i*iterationsPerLoop+j << endl;
		}
		
		sumAll=0;
        for(int j=0;j<alphas.size();j++){
            media[j] = (float)sum[j]/times[j];
            aux[j] = pow(costBest/media[j],5);
            sumAll = sumAll+aux[j];
        }
		for(int j=0;j<alphas.size();j++){
            probability[j] = (aux[j]/sumAll)*100;
        }
	}
	return best;
}

Solution* GRASPReativoMIP (int numberOfIterations, bool firstImprovement, vector<Route *> * routes, bool (* compare)(Vertex, Vertex)){
	int numberOfLooping = 20;
	int iterationsPerLoop = numberOfIterations/numberOfLooping;
	vector<float> alphas;
	alphas.push_back(0.1);
	alphas.push_back(0.2);
	alphas.push_back(0.3);

	vector<float> probability(alphas.size(),100.0/alphas.size());
	vector<float> media(alphas.size());
	vector<float> aux(alphas.size());
	vector<float> sum(alphas.size(),0);
	vector<unsigned long int> times(alphas.size(),0);

	float sumAll;
	
	Solution * s;
	float cost;
	float costBest = MAXFLOAT;
	Solution * best = NULL;
	Route * r;
	for(int i=0;i<numberOfLooping;i++){
		for(int j=0;j<iterationsPerLoop;j++){
			int pos = chooseAlpha(probability,rand()%100);
			
			s = greedyInsertMiddle(vertex, alphas[pos], compare);
			for(int k=0;k<s->routes.size();k++){
				r = new Route(s->routes[k], vertex.size());
				routes->push_back(r);
			}

			s = RVND(s, firstImprovement);
			for(int k=0;k<s->routes.size();k++){
				r = new Route(s->routes[k], vertex.size());
				routes->push_back(r);
			}

			cost = calcCostSolution(s);
			
			times[pos]++;
			sum[pos]=sum[pos]+cost;

			if(cost < costBest){
				if(best != NULL){
					delete(best);
				}
				best = s;
				costBest = cost;
			}else{
				delete(s);
			}
		}
		
		sumAll=0;
        for(int j=0;j<alphas.size();j++){
            media[j] = (float)sum[j]/times[j];
            aux[j] = pow(costBest/media[j],5);
            sumAll = sumAll+aux[j];
        }
		for(int j=0;j<alphas.size();j++){
            probability[j] = (aux[j]/sumAll)*100;
        }
	}
	return best;
}

void everyoneIsAttended (vector<Route *> * s){
	vector<bool> attended(vertex.size(), false);
	int qtd = 0;
	for(int i=0;i<s->size();i++){
		for(int j=1;j<s->at(i)->pathTruck.size()-1;j++){
			if(!attended[s->at(i)->pathTruck.at(j)]){
				qtd++;
				attended[s->at(i)->pathTruck.at(j)] = true;
			}else{
				cout << "Alguem foi atendido 2 vezes" << endl;
			}
		}
		for(int j=1; j<s->at(i)->pathsDrone.size();j=j+3){
			if(!attended[s->at(i)->pathsDrone.at(j)]){
				qtd++;
				attended[s->at(i)->pathsDrone.at(j)] = true;
			}else{
				cout << "Opa!!! Alguem foi atendido 2 vezes" << endl;
			}
		}
	}
	if(qtd != vertex.size()-2){
		cout << "Opa!!! Alguem nao foi atendido" << endl;
	}
}

float mip(vector<Route *> *routes){
	GRBEnv *env = new GRBEnv();
	GRBModel model = GRBModel(*env);
	model.set(GRB_IntParam_MIPFocus,1);
	GRBVar vars[routes->size()];
	for(int i=0;i<routes->size();i++){
		vars[i]=model.addVar(0.0,1.0,routes->at(i)->cost,GRB_BINARY,"route" + to_string(i));
	}
	for(int i=1;i<vertex.size()-1;i++){
		GRBLinExpr * exp = new GRBLinExpr();
		for(int j=0;j<routes->size();j++){
			exp->addTerms(&routes->at(j)->attended[i],&vars[j],1);
		}
		model.addConstr(*exp,GRB_EQUAL,1,"vertex"+to_string(i));
	}
	model.set(GRB_IntParam_OutputFlag, 0);
	
	model.optimize();
	return model.getObjective().getValue();
}

void mainMSTPH(string instance, int seed, int interations){
	time_t t = clock();
	string instanceId = initParameters(instance, seed);
	Solution * s = MSTPH(interations,false);
	float cost = calcCostSolution(s); 
	float tempo = (clock()-t)/(float)CLOCKS_PER_SEC;
	cout << instanceId << "; " << seed << "; " << tempo << "; " << cost << endl;

	//plotSolution(s, instance);

	delete(s);
}

void mainMSTPHFLS(string instance, int seed, int interations){
	time_t t = clock();
	string instanceId = initParameters(instance, seed);
	Solution * s = MSTPH(interations,true);
	float cost = calcCostSolution(s); 
	float tempo = (clock()-t)/(float)CLOCKS_PER_SEC;
	cout << instanceId << "; " << seed << "; " << tempo << "; " << cost << endl;

	//plotSolution(s, instance);
	solutionIsViable(s, &graphTruck, &graphDrone, &vertex, distMaxDrone, maxAwaitTime, capacityTruck);
	delete(s);
}

void mainMSTPH2(string instance, int seed, int interations, int improvements){
	time_t t = clock();
	string instanceId = initParameters(instance, seed);
	Solution * s = MSTPH2(interations,improvements);
	float cost = calcCostSolution(s); 
	float tempo = (clock()-t)/(float)CLOCKS_PER_SEC;
	cout << instanceId << "; " << seed << "; " << tempo << "; " << cost << endl;

	//plotSolution(s, instance);
	solutionIsViable(s, &graphTruck, &graphDrone, &vertex, distMaxDrone, maxAwaitTime, capacityTruck);
	delete(s);
}

void mainGraspReativo(string instance, int seed, bool firstImprovement){
	
	string instanceId = initParameters(instance, seed);
	
	time_t t = clock();
	
	int melhorIteracao;

	Solution* s = GRASPReativo(1000, firstImprovement, &melhorIteracao, instanceId, seed);

	float tempo = (clock()-t)/(float)CLOCKS_PER_SEC;
	
	delete(s);
}

Solution * greedInsertEnd(vector<Vertex> v, float alpha){
	vector<float> currentTime;
	bool * attended = new bool[v.size()];
	for(int i=0;i<v.size();i++){
		attended[i]=false;
	}
	Solution *s = new Solution(v.size());
	addNewRoute(s,&currentTime);
	sort(v.begin()+1,v.end()-1,compareClosingTimeWindow);
	for(int i=1;i<v.size()-1;i++){
		while(attended[v[i].id]){
			i++;
		}
		if(i==v.size()-1){
			break;
		}
		vector<Choice2> choices;
		//v[i] == cliente a ser atendido
		for(int j=0;j<s->routes.size();j++){
			if(truckCanAttend(s,j,&v[i],currentTime)){
				Choice2 aux;
				aux.truck = j;
				aux.cost = costPerTimeTruck*graphTruck[s->routes[j]->pathTruck.back()][v[i].id];
				choices.push_back(aux);
				vector<int> drone;
				for(int k=i+1;k<v.size()-1;k++){
					if(! attended[ v[k].id ]){
						if(droneCanAttend( s,j,&v[i],&v[k] )){
							drone.push_back(k);
						}
					}
				}

				//1 drone
				if(drone.size()>=1){
					for(int a=0;a<drone.size();a++){
						Choice2 aux2;
						aux2.truck = j;
						aux2.cost = (aux.cost + costPerTimeDrone*(graphDrone[s->routes[j]->pathTruck.back()][v[drone[a]].id]+graphDrone[v[drone[a]].id][v[i].id]))/2;
						aux2.drones.push_back(v[ drone[a] ].id);
						choices.push_back(aux2);
					}
				}
				//2 drones
				if(drone.size()>=2){ 
					for(int a=0;a<drone.size();a++){
						for(int b=a+1;b<drone.size();b++){
							if(v[i].request+v[drone[a]].request+v[drone[b]].request <= s->routes[j]->currentCapacity){
								Choice2 aux2;
								aux2.truck = j;
								aux2.cost = (aux.cost + costPerTimeDrone*(graphDrone[s->routes[j]->pathTruck.back()][v[drone[a]].id]+graphDrone[v[drone[a]].id][v[i].id])+ costPerTimeDrone*(graphDrone[s->routes[j]->pathTruck.back()][v[drone[b]].id]+graphDrone[v[drone[b]].id][v[i].id]))/3;
								aux2.drones.push_back(v[ drone[a] ].id);
								aux2.drones.push_back(v[ drone[b] ].id);
								choices.push_back(aux2);
							}
						}
					}
				}
				//3 drones
				if(drone.size()>=3){ 
					for(int a=0;a<drone.size();a++){
						for(int b=a+1;b<drone.size();b++){
							for(int c=b+1;c<drone.size();c++){
								if(v[i].request+v[drone[a]].request+v[drone[b]].request+v[drone[c]].request <= s->routes[j]->currentCapacity){
									Choice2 aux2;
									aux2.truck = j;
									aux2.cost = (aux.cost + costPerTimeDrone*(graphDrone[s->routes[j]->pathTruck.back()][v[drone[a]].id]+graphDrone[v[drone[a]].id][v[i].id])+ costPerTimeDrone*(graphDrone[s->routes[j]->pathTruck.back()][v[drone[b]].id]+graphDrone[v[drone[b]].id][v[i].id])+ costPerTimeDrone*(graphDrone[s->routes[j]->pathTruck.back()][v[drone[c]].id]+graphDrone[v[drone[c]].id][v[i].id]))/4;
									aux2.drones.push_back(v[ drone[a] ].id);
									aux2.drones.push_back(v[ drone[b] ].id);
									aux2.drones.push_back(v[ drone[c] ].id);
									choices.push_back(aux2);
								}
							}
						}
					}
				}
			}
		}
		if(choices.empty()){
			cout << "Não foi possivel construir uma solução " << s->routes.size() << endl;
			exit(0);
		}
		sort(choices.begin(),choices.end(),compareCost);
		int qtd = floor(choices.size()*alpha)+1;
		makeMovement(s, &choices[rand() % qtd], &v[i],&currentTime,attended);
	}

	if(s->routes.back()->pathTruck.size()==1 && s->routes.back()->pathsDrone.size()==0){
		s->routes.erase(s->routes.end()-1);
	}

	//voltando ao deposito
	for (int i=0;i<s->routes.size();i++){
		s->routes[i]->cost+=costPerTimeTruck*graphTruck[s->routes[i]->pathTruck.back()][v.size()-1];
		s->routes[i]->pathTruck.push_back(v.size()-1);
	}

	delete[] attended;
	/*
	if(!solutionIsViable(s,&graphTruck,&graphDrone,&vertex,distMaxDrone,maxAwaitTime,capacityTruck)){
		cout << "Deu merda" << endl;
	}
	*/
	return s;
}

void mainMip(string instance, int seed){
	
	string instanceId = initParameters(instance, seed);
	
	time_t t = clock();
	
	vector<Route *> routes;
	Solution * s;
	float cost, bestCost = MAXFLOAT;

	s = GRASPReativoMIP(500,true, &routes, compareWeight);
	cost = calcCostSolution(s);
	if(cost < bestCost){
		bestCost = cost;
	}
	delete(s);
	
	s = GRASPReativoMIP(450,false, &routes, compareWeight);
	cost = calcCostSolution(s);
	if(cost < bestCost){
		bestCost = cost;
	}
	delete(s);
	
	s = GRASPReativoMIP(50,true, &routes, compareClosingTimeWindow);
	cost = calcCostSolution(s);
	if(cost < bestCost){
		bestCost = cost;
	}
	delete(s);

	s = GRASPReativoMIP(80,false, &routes, compareClosingTimeWindow);
	cost = calcCostSolution(s);
	if(cost < bestCost){
		bestCost = cost;
	}
	delete(s);

	Route * r;
	for(int i=0;i<100;i++){
		s = greedInsertEnd(vertex, 0.2);
		for(int j=0;j<s->routes.size();j++){
			r = new Route(s->routes[j], vertex.size());
			routes.push_back(r);
		}

		s = RVND(s, true);
		cost = calcCostSolution(s);
		if(cost < bestCost){
			bestCost = cost;
		}
		for(int j=0;j<s->routes.size();j++){
			r = new Route(s->routes[j], vertex.size());
			routes.push_back(r);
		}

		delete(s);
	}
	
	vector<Choice> choices;
	
	for(int i=1;i<vertex.size()-1;i++){
		choices.clear();
		float minCost = MAXFLOAT;
		s = new Solution(vertex.size(), capacityTruck);
		getMovementsPossiblesByTruck(s, &vertex[i], &minCost, &choices);
		if(vertex[i].drone){
			getMovementsPossiblesByDrone(s, &vertex[i], &minCost, &choices);
		}
		if(choices[0].posBack==-1){
			makeMovementTruck(s,choices[0].truck,choices[0].posInsert,vertex[i].id);
		}else{
			makeMovementDrone(s,choices[0].truck,choices[0].posInsert,choices[0].posBack,vertex[i].id);
		}
		r = new Route(s->routes[0],vertex.size());
		routes.push_back(r);
		delete(s);
	}

	time_t tMip = clock();
	float custoMip = mip(&routes);
	float tempoMip = (clock()-tMip)/(float)CLOCKS_PER_SEC; 
	float tempo = (clock()-t)/(float)CLOCKS_PER_SEC; 

	cout << "mip; " << instanceId << "; " << seed << "; " << tempo << "; " << tempoMip << "; " << bestCost << "; " << custoMip << endl;
	for(int i=0;i<routes.size();i++){
		delete(routes[i]);
	}
}

int main(int argc, char * argv[]){
	string instance = argv[1];
	int seed = stoi(argv[2]);

	mainMip(instance, seed);

	return 0;
}