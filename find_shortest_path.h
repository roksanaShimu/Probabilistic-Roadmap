#ifndef find_shortest_path
#define find_shortest_path


#include<vector>
#include<fstream>


#include "predefined_variables.h"

using namespace std;

/*void Dijkstra_shortest_path(local_planner_node* N, float dist){
	if(N->visited) return;
	N->visited=true;
	for( int i=0; i<N->knn.size();i++){
		if(N->knn[i]->first->dist_from_start  >  dist+N->knn[i]->second){
			N->knn[i]->first->dist_from_start=dist+N->knn[i]->second;
			N->knn[i]->first->source=N;	
		}
		if(! N->knn[i]->first->in_the_min_heap){
			add_to_the_min_heap(N->knn[i]->first);
		}
	} 
}*/


bool find_the_shortest_path(local_planner_node* start, local_planner_node* destination, vector<local_planner_node*>& sample_local_planner_node){
	min_heap* MH= new min_heap();

		

	start->source=start;
	MH->add_node(start);
	
	while(!MH->isempty()){
		//cout<<"inside while loop"<<endl;
		local_planner_node* N=new local_planner_node();
		N= MH->get_node();
		if(N==destination){
			//cout<<"found the destination"<<endl;
			return true;
		}
		if(!N->visited){
			N->visited=true;
			for( int i=0; i<N->knn.size();i++){
				bool info_updated=false;
				//cout<<"N->knn[i].first->dist_from_start= "<<N->knn[i].first->dist_from_start<<"  and N->dist_from_start+N->knn[i].second= "<<N->dist_from_start+N->knn[i].second<<endl;
				if(N->knn[i].first->dist_from_start  >  N->dist_from_start+N->knn[i].second){
					//cout<<"inside the if condition"<<endl;
					N->knn[i].first->dist_from_start=N->dist_from_start+N->knn[i].second;
					N->knn[i].first->source=N;
					info_updated=true;	
				}
				if(! N->knn[i].first->in_the_min_heap){
					MH->add_node(N->knn[i].first);
				}else if(info_updated){
					MH->update_the_heap(N->knn[i].first);
				}
			}
		} 
		//cout<<"at the end of while loop the heap size is: "<<MH->size()<<endl;

	}
	return false;

}

void print_the_source_locations(vector<local_planner_node*> sample_local_planner_node){
	for (int i=0; i< sample_local_planner_node.size(); i++){
		cout<<"node index: "<<sample_local_planner_node[i]->index<<"  source index: "<<sample_local_planner_node[i]->source->index<<"  and distance from start: "<<sample_local_planner_node[i]->dist_from_start<<endl;	
	}
}

void print_destination_to_start_index(vector<local_planner_node*> sample_local_planner_node, local_planner_node* start, local_planner_node* destination){

	cout<<"*******************RESULT******************"<<endl;
	float d=0.0;

	local_planner_node* n=destination;
	float x1=n->p.x; float y1=n->p.y; float z1=n->p.z;	
	while (n!=start){
		cout<<n->index<<"   dist= "<< d<<endl;
		n=n->source;
		float x2=n->p.x; float y2=n->p.y; float z2=n->p.z;
		d=d+ sqrt(  (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2) );
		x1=x2;y1=y2;z1=z2;
	}
	cout<<n->index<<"   dist= "<< d<<endl;
}

void write_in_a_file(vector<local_planner_node*> sample_local_planner_node, local_planner_node* start, local_planner_node* destination, string name){

	vector<position> position_result;
	local_planner_node* n=destination;
	while (n!=start){
		position_result.push_back(n->p);
		//cout<<n->index<<endl;
		n=n->source;
	}
	position_result.push_back(n->p);
	//cout<<n->index<<endl;

	const char * fname= name.c_str();
	ofstream outfile;
	outfile.open(fname);
	
	outfile<<position_result.size()<<endl;
	
	


	for(int i=position_result.size()-1; i>=0; i--){
		outfile<<position_result[i].x<<"  "<<position_result[i].y<<"  "<<position_result[i].z<<endl; 
	}

	outfile.close();
}

void write_the_sample_nodes_in_a_file(vector<local_planner_node*>sample_local_planner_node, string f_name){


	const char * fname= f_name.c_str();
	ofstream outfile;
	outfile.open(fname);
	
	outfile<<sample_local_planner_node.size()<<endl;
	
	


	for(int i=0; i< sample_local_planner_node.size(); i++){
		outfile<<sample_local_planner_node[i]->p.x<<"  "<<sample_local_planner_node[i]->p.y<<"  "<<sample_local_planner_node[i]->p.z<<endl; 
	}

	outfile.close();
	
	
}





#endif
