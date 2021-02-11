#ifndef ann_update_k_neighbors
#define ann_update_k_neighbors

#include <stdint.h>
#include<vector>


#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include </home/roksana/Dropbox/ann/include/ANN/ANN.h>		// ANN declarations
//#include </home/shimu/Dropbox/ann/include/ANN/ANN.h>     //for laptop       

//-I/home/shimu/ann/include -L/home/shimu/ann/lib -lANN


#include "predefined_variables.h"
#include "milestones.h"
#include "update_k_neighbors.h"    //for the function "bool path_can_be_build(local_planner_node* A, local_planner_node* B, vector<Sphere> obstacles)"

using namespace std;






int		k	= number_of_elements_for_knn+1;			// number of nearest neighbors
int		dim	= 3;						// dimension
double		eps	= 0;						// error bound
int		maxPts	= init_number_of_samples;			// maximum number of data points
    
//istream*	dataIn	= NULL;			// input for data points
//istream*	queryIn	= NULL;			// input for query points


void readPt(ANNpoint p, local_planner_node* a)		// read point 
{
	p[0]=a->p.x;p[1]=a->p.y;p[2]=a->p.z;
}


void printPt( ANNpoint p)			// print point
{
	cout << "(" << p[0];
	for (int i = 1; i < dim; i++) {
		cout << ", " << p[i];
	}
	cout << ")\n";
}


//this function is already declared in the "update_k_neighbors.h"
/*bool path_can_be_build(local_planner_node* A, local_planner_node* B, vector<Sphere> obstacles){
	Sphere local_sample[9];
	int i=0; int j;
	for(float t=0.1;t<1;t=t+0.1){
		//cout<<t<< "    "<<i<<endl;
		local_sample[i].p.x=A->p.x+(B->p.x-A->p.x)*t;
		local_sample[i].p.y=A->p.y+(B->p.y-A->p.y)*t;
		local_sample[i].p.z=A->p.z+(B->p.z-A->p.z)*t;
		local_sample[i].radius=UAV_radius;
		i++;
	}

	
	for(i=0;i<9;i++){
		for(j=0;j<number_of_obstacles ;j++){
			if(doesItCollide(local_sample[i].p, obstacles[j])){
				//cout<<"collides with obstacle x: "<<obstacles[j].p.x<<" y: "<<obstacles[j].p.y<<" z: "<<obstacles[j].p.z<<" radius: "<<obstacles[j].radius<<" and the UAV location is x: "<< local_sample[i].p.x<<" y: "<<local_sample[i].p.y<<" z: "<<local_sample[i].p.z<<endl<<endl;
				return false;
			}
		}	
	}
	

	return true;
}*/



void update_kth_neighbors_with_ann(vector<local_planner_node*>& sample_local_planner_node, vector<Sphere> obstacles){
	if(sample_local_planner_node.size()<= number_of_elements_for_knn){
		cout<<"Error: the number of neighbors for knn is higher than the sample nodes" <<endl;
		return;
	}

	int		nPts;			// actual number of data points
	ANNpointArray	dataPts;		// data points
	ANNpoint	queryPt;		// query point
	ANNidxArray	nnIdx;			// near neighbor indices
	ANNdistArray	dists;			// near neighbor distances
	ANNkd_tree*	kdTree;			// search structure
	
	
	//getArgs(argc, argv);						// read command-line arguments
	
	queryPt = annAllocPt(dim);		// allocate query point
	dataPts = annAllocPts(maxPts, dim);	// allocate data points
	nnIdx = new ANNidx[k];			// allocate near neigh indices
	dists = new ANNdist[k];			// allocate near neighbor dists

	nPts = 0;									
	
	// read data points
	//cout << "Data Points:"<<endl;
	while (nPts < maxPts){

		readPt(dataPts[nPts],sample_local_planner_node[nPts]);
		//printPt(dataPts[nPts]);
		nPts++;
	}


	kdTree = new ANNkd_tree(		// build search structure
				dataPts,	// the data points
				nPts,		// number of points
				dim);		// dimension of space


	typedef pair<local_planner_node*, float> apair;
	for(int j=0; j<maxPts; j++){
		
		readPt(queryPt, sample_local_planner_node[j]); // read query points
			
		//cout << "Query point: ";		// echo query point
		//printPt(queryPt);

		kdTree->annkSearch(			// search
				queryPt,		// query point
				k,			// number of near neighbors
				nnIdx,			// nearest neighbors (returned)
				dists,			// distance (returned)
				eps);			// error bound


		// print summary

		//cout << "\tNN:\tIndex\tDistance\n";
		for (int i = 1; i < k; i++) {		//i sarted with 1 instead of 0, because it finds itself as NN with distance 0	

			if(path_can_be_build(sample_local_planner_node[j], sample_local_planner_node[nnIdx[i]], obstacles)){
				dists[i] = sqrt(dists[i]);		// unsquare distance
				//cout << "\t" << i << "\t" << nnIdx[i] << "\t" << dists[i] << "\n";

				sample_local_planner_node[j]->knn.push_back(apair(sample_local_planner_node[nnIdx[i]], dists[i] ));
			}
		}

	}



}



//this function is already declared in the "update_k_neighbors.h"
/*void print_neighbors_update(vector<local_planner_node*> sample_local_planner_node){
	for(int i =0; i<sample_local_planner_node.size();i++){
		cout<<"i= "<<i<<"  x= "<<sample_local_planner_node[i]->p.x<<"  y= "<<sample_local_planner_node[i]->p.y<<"  z= "<<sample_local_planner_node[i]->p.z<<"  no of neighbors= "<<sample_local_planner_node[i]->knn.size()<<endl; 
		for(int j=0; j<sample_local_planner_node[i]->knn.size(); j++){ 
			cout<<sample_local_planner_node[i]->knn[j].first->index<<", "<<sample_local_planner_node[i]->knn[j].second<<endl;
		}
		cout<<endl;
	}
}*/







#endif
