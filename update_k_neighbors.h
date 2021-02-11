#ifndef update_k_neighbors
#define update_k_neighbors

#include <stdint.h>
#include<vector>


#include "predefined_variables.h"
#include "milestones.h"

using namespace std;




bool path_can_be_build(local_planner_node* A, local_planner_node* B, vector<Sphere> obstacles){
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
}


float calculate_distance(local_planner_node* A, local_planner_node* B){
	return sqrt( (A->p.x-B->p.x)*(A->p.x-B->p.x) + (A->p.y-B->p.y)*(A->p.y-B->p.y) +(A->p.z-B->p.z)*(A->p.z-B->p.z) );
}



void get_nodes(vector<local_planner_node*>& S, int i, int start_index, int end_index, vector<Sphere> obstacles){
	//boundary conditions
	if(start_index <0 || end_index>S.size()-1)return;
	if(start_index> end_index)return;
	//end of boundary conditions
	
	typedef pair<local_planner_node*, float> apair;


	for(int j=start_index; j<=end_index; j++){
		if (path_can_be_build(S[i], S[j], obstacles)){
			S[i]->knn.push_back(apair(S[j], calculate_distance(S[i],S[j]) ));
		}
	}
}


void update_kth_neighbors(vector<local_planner_node*>& sample_local_planner_node, vector<Sphere> obstacles){
	if(sample_local_planner_node.size()<= number_of_elements_for_knn){
		cout<<"Error: the number of neighbors for knn is higher than the sample nodes" <<endl;
		return;
	}
	int m=sample_local_planner_node.size()-1;
	int n=number_of_elements_for_knn;    
	//number_of_elements_for_knn is multiple of 4. so number_of_elements_for_knn is always divisible by 2

	

	for(int i=0;i<=m;i++){

		if(i<(n/2)){ //not enough elements at the begining
			get_nodes(sample_local_planner_node, i, 0, i-1, obstacles);
			get_nodes(sample_local_planner_node, i, i+1, n, obstacles);

		}else if((m-i)<(n/2)){  //not enough elements at the end
			get_nodes(sample_local_planner_node, i, m-n, i-1, obstacles);
			get_nodes(sample_local_planner_node, i, i+1, m, obstacles);
			
		}else{
			get_nodes(sample_local_planner_node, i, i-(n/2), i-1, obstacles);
			get_nodes(sample_local_planner_node, i, i+1, i+(n/2), obstacles);

		}

	}

}


void print_neighbors_update(vector<local_planner_node*> sample_local_planner_node){
	for(int i =0; i<sample_local_planner_node.size();i++){
		cout<<"i= "<<i<<"  x= "<<sample_local_planner_node[i]->p.x<<"  y= "<<sample_local_planner_node[i]->p.y<<"  z= "<<sample_local_planner_node[i]->p.z<<"  no of neighbors= "<<sample_local_planner_node[i]->knn.size()<<endl; 
		for(int j=0; j<sample_local_planner_node[i]->knn.size(); j++){ 
			cout<<sample_local_planner_node[i]->knn[j].first->index<<", "<<sample_local_planner_node[i]->knn[j].second<<endl;
		}
		cout<<endl;
	}
}


#endif
