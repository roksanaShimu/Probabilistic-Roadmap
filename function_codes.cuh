#ifndef function_codes
#define function_codes


#include<iostream>
#include <stdint.h>
#include <inttypes.h>
#include <cmath>
#include <sstream>

#include "predefined_variables.h"
#include "data_structures.h"

vector<position> read_input_nodes(){

	vector<position> input_nodes;
		
	// read the waypoints found from TSP
	ifstream infile; infile.open(input_file_name);
	
	position k; int n;
	infile>>n;

	for(int i=0;i<n ;i++){
		infile>>k.x>>k.y>>k.z;
		input_nodes.push_back(k);
	}
	
	infile.close();

	return input_nodes;
}

vector<local_planner_node*> read_sample_nodes(int m){

	vector<local_planner_node*> sample_local_planner_node;
		
	string sample_nodes_file_name=sample_node_debug_file_name;
	ostringstream temp;  //temp as in temporary
	temp<<m;
	string s2=temp.str();
	sample_nodes_file_name.insert(12,s2);
	string file_dir=sample_file_location;
	sample_nodes_file_name.insert(0,file_dir);
	
	ifstream infile; infile.open(sample_nodes_file_name.c_str());
	
	position k; int number_of_sample_nodes;
	infile>>number_of_sample_nodes;
	

	for(int i=0;i<number_of_sample_nodes ;i++){
		local_planner_node* s=new local_planner_node(); 
		
		infile>>k.x>>k.y>>k.z;
		s->p=k;
		sample_local_planner_node.push_back(s); 
	}

	infile.close();

	return sample_local_planner_node;
}


string generate_path_output_file_name(int n){
	string path_output_file_name=path_debug_file_name;
			
	ostringstream temp;  //temp as in temporary
	temp<<n;
	string s=temp.str();
	path_output_file_name.insert(10,s);

	string file_dir=file_location;
	path_output_file_name.insert(0,file_dir);
	return path_output_file_name;

}


string generate_sample_nodes_file_name(int n){
	string sample_nodes_file_name=sample_node_debug_file_name;
	ostringstream temp2;  //temp as in temporary
	
	temp2<<n;
	string s2=temp2.str();
	sample_nodes_file_name.insert(12,s2);

	string file_dir=file_location;
	sample_nodes_file_name.insert(0,file_dir);
	
	return sample_nodes_file_name;
}



#endif
