//The program finds the path for an UAV using Probabilistic Roadmap Method (PRM)- serial version. 
// nvcc PRM_parallel_17.cu -use_fast_math -Xptxas -v -arch=sm_30 -lcurand -I/home/roksana/Dropbox/ann/include -L/home/roksana/Dropbox/ann/lib -lANN
//nvcc PRM_parallel_17.cu -use_fast_math -Xptxas -v -arch=sm_21 -lcurand -I/home/shimu/Dropbox/ann/include -L/home/shimu/Dropbox/ann/lib -lANN
//nvcc main1.cu -use_fast_math -Xptxas -v -arch=sm_21 -lcurand -I/home/shimu/ann/include -L/home/shimu/ann/lib -lANN


//input: start & destination location
//output: a set of locations that includes start location, intermediate locations and destination location



#include<iostream>
#include <stdint.h>
#include <inttypes.h>
#include <cmath>
#include <sstream>

#include "predefined_variables.h"
#include "data_structures.h"
#include "milestones.h"
#include "update_z_number.h"
#include "update_k_neighbors.h"
#include "ann_update_k_neighbors.h"
#include "find_shortest_path.h"
#include "device_BitonicSort_prm.cuh"
#include "device_codes.cuh"

#include "obstacle_generation_file.cuh"


using namespace std;


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


int main(){

	//generate_input_nodes();

	bool parallel=false; //true;//false;   //false;	//true;
	
	

	// read the list of start and end nodes 
	// the nodes are given as A, B, C, D. we need to read the nodes coordinate first. Then find the path between A to B, B to C, C to D.	

	vector<position> input_nodes=read_input_nodes();
	if (input_nodes.size()<=1){
		cout<<"Error: Not enough nodes to find path"<<endl;
		return 0;
	} 
	
	//for(int i=0; i<input_nodes.size();i++)cout<<input_nodes[i].x<< "  "<<input_nodes[i].y<< "  "<<input_nodes[i].z<< endl;


	//****************************read obstacles************************************************
	//generate_obstacle(input_nodes);
	clock_t read_obstacle_tStart = clock();	
	vector<Sphere>obstacles; obstacles.resize(number_of_obstacles);
	read_the_obstacles(obstacles);
	clock_t read_obstacle_tEnd = clock();	
	printf("from the loop: Time taken to read the obstacle: %.6fs\n", (double)(read_obstacle_tEnd - read_obstacle_tStart)/CLOCKS_PER_SEC);


	clock_t tStart = clock();


	vector<vector<local_planner_node*> > ALL_sample_local_planner_node;
	for(int n=0; n<input_nodes.size()-1; n++){
		

		//******************initialize the start and the destination location***********************
		local_planner_node* start= new local_planner_node();
		start->set_local_planner_node(input_nodes[n].x, input_nodes[n].y, input_nodes[n].z);
		cout<<"start: " <<start->p.x<<"  "<<start->p.y<<"   "<<start->p.z<<endl;
		start->dist_from_start=0;

		local_planner_node* destination = new local_planner_node();
		destination->set_local_planner_node(input_nodes[n+1].x, input_nodes[n+1].y, input_nodes[n+1].z);
		cout<<"destination: "<<destination->p.x<<"  "<<destination->p.y<<"   "<<destination->p.z<<endl;

		
		//************************boundary conditions***********************************************
		//check if the start and destination location collides with any obstacles
		if(!boundary_condition_satisfied(start, destination, obstacles)){
			cout<<"boundary condition doesn't satisfied either for start or destination"<<endl;
			//return 0;
		}

		
		//******************* find the grid size for putting the samples****************************

		vector<float> xyz_limit(6);		
		vector<int> sample_grid_size =find_the_sample_grid_size(start->p, destination->p, xyz_limit);
		cout<<" nodes_in_x = "<<sample_grid_size[0]<<" nodes_in_y = "<<sample_grid_size[1]<<" nodes_in_z = "<<sample_grid_size[2]<<endl;

		//cout<<"x_start="<<xyz_limit[0]<<", x_end="<<xyz_limit[1]<<", y_start="<<xyz_limit[2]<<", y_end="<<xyz_limit[3]<<", z_start="<<xyz_limit[4]<<", z_end="<<xyz_limit[5]<<endl;



		//**************************generate the sample locations***********************************
		vector<local_planner_node*> sample_local_planner_node;
		//cout<<"Select one of the following options:"<<endl<<"   1. Generate Random Samples"<<endl<<"   2. Generate Grid Samples"<<endl<<"   3. Generate Exact Grid Samples"<<endl;
		
		clock_t sample_generation_tStart = clock();
		int a=2;
		//cin>>a;

		if(a==1){   // Generate Random Samples
			sample_local_planner_node=generate_milestone_sample_local_planner_node(obstacles, start, destination, sample_grid_size, xyz_limit);
		
		}else if(a==2){	//Generate Grid Samples
			sample_local_planner_node=generate_grid_milestone_sample_local_planner_node(obstacles, start, destination, sample_grid_size, xyz_limit );
		
		}else if(a==3){	//Generate exact Grid Samples
			sample_local_planner_node=generate_exact_grid_milestone_sample_local_planner_node(obstacles, start, destination, sample_grid_size, xyz_limit);
		
		}else{
			cout<<"Invaild selection!"<<endl<<"Program is terminating."<<endl;
			//return 0;
		}

		
		clock_t sample_generation_tEnd = clock();
		printf("from the loop: Time taken in the sample generation process: %.6fs\n", (double)(sample_generation_tEnd - sample_generation_tStart)/CLOCKS_PER_SEC);


		ALL_sample_local_planner_node.push_back(sample_local_planner_node);

		string sample_nodes_file_name=sample_node_debug_file_name;
		ostringstream temp2;  //temp as in temporary
		int j=n;
		temp2<<j;
		string s2=temp2.str();
		sample_nodes_file_name.insert(12,s2);

		string file_dir=file_location;
		sample_nodes_file_name.insert(0,file_dir);
		cout<<"sample_nodes_file_name: "<< sample_nodes_file_name<<endl;
		write_the_sample_nodes_in_a_file(sample_local_planner_node, sample_nodes_file_name);
	}


	//**************************************************************************************************
	//**********************************Serial codes - ann lib******************************************
	//**************************************************************************************************
	
	if(!parallel){

		double total_time_knn_ann=0;
		for(int n=0; n<input_nodes.size()-1; n++){

	
			vector<local_planner_node*> sample_local_planner_node=ALL_sample_local_planner_node[n];
			local_planner_node* start=sample_local_planner_node[init_number_of_samples-2];
			local_planner_node* destination =sample_local_planner_node[init_number_of_samples-1];

			//******************************** NEIGHBOR SELECTION PROCESS*******************************
			//for debugging
			for(int k =0; k<sample_local_planner_node.size();k++){
				sample_local_planner_node[k]->index=k;
			}


			clock_t knn_process_tStart=clock();
		
			//Select neighbors using ANN library
			//Nearest Neighbors
			update_kth_neighbors_with_ann(sample_local_planner_node, obstacles);
			//print_neighbors_update(sample_local_planner_node);
			
			clock_t knn_process_tEnd = clock();	
		
	

			//********************************** find the shortest path ********************************
			clock_t path_find_tStart = clock();
		
			if (find_the_shortest_path(start, destination, sample_local_planner_node)){
	 
				//print_the_source_locations(sample_local_planner_node);
	
				cout<<"distance from start to destination: "<<destination->dist_from_start<<endl;

				print_destination_to_start_index(sample_local_planner_node, start, destination);
		
				string path_output_file_name=path_debug_file_name;
			
				ostringstream temp;  //temp as in temporary
				int j=n;			
				temp<<j;
				string s=temp.str();
				path_output_file_name.insert(10,s);

				string file_dir=file_location;
				path_output_file_name.insert(0,file_dir);

				//cout<<"path_output_file_name: "<<path_output_file_name<<"  s="<<s<<endl;
				write_in_a_file(sample_local_planner_node, start, destination, path_output_file_name);
				//write_the_sample_nodes_in_a_file(sample_local_planner_node);

			}else{
				cout<<"couldn't found the path"<<endl;
			}
			clock_t path_find_tEnd = clock();

	
			total_time_knn_ann=total_time_knn_ann+(double)(knn_process_tEnd - knn_process_tStart)/CLOCKS_PER_SEC;
			printf("from the loop: Time taken for ann-knn process: %.6fs\n", (double)(knn_process_tEnd - knn_process_tStart)/CLOCKS_PER_SEC);
			//printf("from the loop: Time taken to find the shortest path: %.6fs\n", (double)(path_find_tEnd - path_find_tStart)/CLOCKS_PER_SEC);

		


		}

		cout<<"TOTAL TIME FOR KNN-ANN="<<total_time_knn_ann<<endl;
	}else{

		//**************************************************************************************************
		//**********************************parallel codes - gpu *******************************************
		//**************************************************************************************************

		
		double total_time_knn_gpu=0;

		//******************************** NEIGHBOR SELECTION PROCESS*******************************
		
		
		//for debugging
		for(int i=0;i<input_nodes.size()-1;i++){
			for(int k =0; k<ALL_sample_local_planner_node[i].size();k++){
				ALL_sample_local_planner_node[i][k]->index=k;
			}
		}

			
		clock_t knn_process_tStart=clock();
		
		//x,y,z coordinates of sample nodes 
		const int element_size_array=((int) ceil((double)init_number_of_samples/warp_size)*warp_size)*(input_nodes.size()-1);
		float xyz[element_size_array*3]; copy_the_coordinates(xyz, ALL_sample_local_planner_node);//host
		float *d_xyz; cudaMalloc((void**)&d_xyz, sizeof(float)*element_size_array*3) ; // device
	
		//obstacles
		float h_obstacles[number_of_obstacles*4]; copy_obstacles(h_obstacles,obstacles);
		float *d_obstacles; cudaMalloc((void**)&d_obstacles, sizeof(float)*number_of_obstacles*4) ; // device
		
		
		//knn_neighbour_nodes_no ->serial no in the vector
		size_t neighbors_serial_no_size= sizeof(int) *init_number_of_samples*number_of_elements_for_knn * (input_nodes.size()-1);
		int h_neighbors_serial_no[init_number_of_samples*number_of_elements_for_knn * (input_nodes.size()-1)];
		int *d_neighbors_serial_no; cudaMalloc((void**)&d_neighbors_serial_no, neighbors_serial_no_size ) ; // device

		//knn_neighbour_corresponding_distances
		size_t neighbors_sq_distance_size= sizeof(float) * init_number_of_samples * number_of_elements_for_knn * (input_nodes.size()-1);
		float h_neighbors_sq_distance[init_number_of_samples*number_of_elements_for_knn * (input_nodes.size()-1)];
		float *d_neighbors_sq_distance; cudaMalloc((void**)&d_neighbors_sq_distance, neighbors_sq_distance_size) ; // device

		
		cudaMemcpy(d_xyz, xyz, sizeof(float)*element_size_array*3, cudaMemcpyHostToDevice);
		cudaMemcpy(d_obstacles, h_obstacles, sizeof(float)*number_of_obstacles*4, cudaMemcpyHostToDevice);
		cudaDeviceSynchronize();
		int blocks=number_of_blocks*(input_nodes.size()-1);
		gpu_calculation<<< blocks, threads_per_block >>>(d_xyz, d_obstacles, d_neighbors_serial_no, d_neighbors_sq_distance);
		cudaDeviceSynchronize();
		cudaMemcpy(h_neighbors_serial_no,d_neighbors_serial_no, neighbors_serial_no_size, cudaMemcpyDeviceToHost);
		cudaMemcpy(h_neighbors_sq_distance,d_neighbors_sq_distance, neighbors_sq_distance_size, cudaMemcpyDeviceToHost);

		
		//UPDATE THE DATA
		typedef pair<local_planner_node*, float> apair;
		int counter=0;
		//cout<<endl<<endl;
		for(int k=0; k<(input_nodes.size()-1); k++){   //2; k++){
		//int k=0;
			for(int i=0;i<init_number_of_samples; i++){
				for(int j=0;j<number_of_elements_for_knn; j++){
					ALL_sample_local_planner_node[k][i]->knn.push_back(apair( ALL_sample_local_planner_node[k][h_neighbors_serial_no[counter]], h_neighbors_sq_distance[counter] ) ); 
				
					counter++;
				}	
				//cout<<endl<<endl;		
			}		
		}

		
		clock_t knn_process_tEnd = clock();

		total_time_knn_gpu=total_time_knn_gpu+(double)(knn_process_tEnd - knn_process_tStart)/CLOCKS_PER_SEC;
		printf("from the loop: Time taken for gpu- knn process: %.6fs\n", (double)(knn_process_tEnd - knn_process_tStart)/CLOCKS_PER_SEC);
	
		

		for(int n=0; n<input_nodes.size()-1; n++){

			vector<local_planner_node*> sample_local_planner_node=ALL_sample_local_planner_node[n];
			local_planner_node* start=sample_local_planner_node[init_number_of_samples-2];
			local_planner_node* destination =sample_local_planner_node[init_number_of_samples-1];
	
			//********************************** find the shortest path ********************************
			clock_t path_find_tStart = clock();
		
			if (find_the_shortest_path(start, destination, sample_local_planner_node)){
	 
				//print_the_source_locations(sample_local_planner_node);
		
				cout<<"distance from start to destination: "<<destination->dist_from_start<<endl;

				print_destination_to_start_index(sample_local_planner_node, start, destination);
		
				string path_output_file_name=path_debug_file_name;
			
				ostringstream temp;  //temp as in temporary
				int j=n;			
				temp<<j;
				string s=temp.str();
				path_output_file_name.insert(10,s);

				string file_dir=file_location;
				path_output_file_name.insert(0,file_dir);

				//cout<<"path_output_file_name: "<<path_output_file_name<<"  s="<<s<<endl;
				write_in_a_file(sample_local_planner_node, start, destination, path_output_file_name);
				

			}else{
				cout<<"couldn't found the path"<<endl;
			}
			clock_t path_find_tEnd = clock();
			
			printf("from the loop: Time taken to find the shortest path: %.6fs\n", (double)(path_find_tEnd - path_find_tStart)/CLOCKS_PER_SEC);
		
		}
		cout<<"TOTAL TIME FOR KNN-GPU="<<total_time_knn_gpu<<endl;

	}



	clock_t tEnd = clock();
	printf("At the end: Total Time taken: %.6fs\n", (double)(tEnd - tStart)/CLOCKS_PER_SEC);

	cout<<"ALL_sample_local_planner_node.size() = "<< ALL_sample_local_planner_node.size()<<endl;

	

	return 0;

}

