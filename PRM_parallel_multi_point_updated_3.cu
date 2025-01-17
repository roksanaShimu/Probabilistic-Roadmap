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
#include "function_codes.cuh"
#include "obstacle_generation_file.cuh"


using namespace std;




int main(){

	//generate_input_nodes();

	bool parallel=false;//true;//false;   //false;	//true;
	
	int iter=5;

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


	float volume=0;
	for(int i=0; i<obstacles.size(); i++){
		volume=volume+(4.0/3)*3.14*obstacles[i].radius*obstacles[i].radius*obstacles[i].radius;
	}

	cout<<"obstacle volume="<<volume<< "number_of_obstacles= "<<obstacles.size()<<endl;
	cout<<"world volume="<<(map_x-map_x_start) * ( map_y-map_y_start) * (map_z-map_z_start)<<endl;
	
	cout<<"number of waypoints="<< input_nodes.size() <<"  number of sample nodes = "<< init_number_of_samples <<"  number of nn= "<< number_of_elements_for_knn <<endl;


	clock_t tStart = clock();

	//********************************************************************************************
	//*********************************************************************************************

	
	
	//**************************************************************************************************
	//**********************************Serial codes - ann lib******************************************
	//**************************************************************************************************
	

	vector<int> path_not_found_ann(iter);
	vector<double> total_time_path_find_ann(iter);
	vector <double> total_time_knn_ann(iter);
	vector<float> total_dist_travel_ann(iter);
	for(int j=0 ; j<iter; j++){
		total_time_path_find_ann[j]=0;
		path_not_found_ann[j]=0;
		total_time_knn_ann[j]=0;
		total_dist_travel_ann[j]=0;
	}

	//if(!parallel){
	for(int j=0;j<iter;j++){
		
		// read sample node

		vector<vector<local_planner_node*> > ALL_sample_local_planner_node;
		ALL_sample_local_planner_node.resize(input_nodes.size()-1);
		for(int n=0; n<input_nodes.size()-1; n++)
			ALL_sample_local_planner_node[n].resize(init_number_of_samples);
		for(int n=0; n<input_nodes.size()-1; n++){
			ALL_sample_local_planner_node[n]= read_sample_nodes(n);
			ALL_sample_local_planner_node[n][init_number_of_samples-2]->dist_from_start=0;
		}
		//cout<<"sample file read complete"<<endl;


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
	 			//cout<<"distance from start to destination: "<<destination->dist_from_start<<endl;		
				//write_in_a_file(sample_local_planner_node, start, destination, generate_path_output_file_name(n));
				total_dist_travel_ann[j]=total_dist_travel_ann[j]+destination->dist_from_start;
			}else{
				cout<<"n="<<n<<"   couldn't found the path"<<endl;
				path_not_found_ann[j]++;
			}
			clock_t path_find_tEnd = clock();
			//******************************************************************************************
	
			total_time_knn_ann[j]=total_time_knn_ann[j]+(double)(knn_process_tEnd - knn_process_tStart)/CLOCKS_PER_SEC;

			total_time_path_find_ann[j]=total_time_path_find_ann[j]+(double)(path_find_tEnd - path_find_tStart)/CLOCKS_PER_SEC;
			
		}
		cout<<"ann iteration "<<j<<" completed."<<endl;
	}
	double avg=0;
	cout<<"TOTAL TIME FOR KNN-ANN="<<endl;
	for(int j=0;j<iter;j++){
		cout<<total_time_knn_ann[j]<<"  ";
		avg=avg+total_time_knn_ann[j];
	}
	cout<<endl <<"mean="<<avg/iter<<endl;

	avg=0;
	cout<<"TOTAL TIME FOR path find-ANN="<<endl;
	for(int j=0;j<iter;j++){
		cout<<total_time_path_find_ann[j]<<"  ";
		avg=avg+total_time_path_find_ann[j];
	}
	cout<<endl<<"mean="<<avg/iter<<endl;
	cout<<"Total path not found = "<<endl;
	for(int j=0;j<iter;j++){
		cout<<path_not_found_ann[j]<<"  ";
	}
	cout<<endl;
	cout<<"Total dist travel ann = "<<endl;
	for(int j=0;j<iter;j++){
		cout<<total_dist_travel_ann[j]<<"  ";
	}
	cout<<endl;


	//}else{

	//**************************************************************************************************
	//**********************************parallel codes - gpu *******************************************
	//**************************************************************************************************
	

	vector<int> path_not_found_gpu(iter);
	vector<double> total_time_path_find_gpu(iter);
	vector <double> total_time_knn_gpu(iter);
	vector<float> total_dist_travel_gpu(iter);
	for(int j=0 ; j<iter; j++){
		total_time_path_find_gpu[j]=0;
		path_not_found_gpu[j]=0;
		total_time_knn_gpu[j]=0;
		total_dist_travel_gpu[j]=0;
	}	
		

	for(int j=0 ; j<iter; j++){

		vector<vector<local_planner_node*> > ALL_sample_local_planner_node;
		ALL_sample_local_planner_node.resize(input_nodes.size()-1);
		for(int n=0; n<input_nodes.size()-1; n++)
			ALL_sample_local_planner_node[n].resize(init_number_of_samples);
		for(int n=0; n<input_nodes.size()-1; n++){
			ALL_sample_local_planner_node[n]= read_sample_nodes(n);
			ALL_sample_local_planner_node[n][init_number_of_samples-2]->dist_from_start=0;
		}
		//cout<<"sample file read complete"<<endl;

		//for debugging
		for(int i=0;i<input_nodes.size()-1;i++){
			for(int k =0; k<ALL_sample_local_planner_node[i].size();k++){
				ALL_sample_local_planner_node[i][k]->index=k;
			}
		}

		//******************************** NEIGHBOR SELECTION PROCESS*******************************
		
			
		clock_t knn_process_tStart=clock();
		if(number_of_waypoints-1<=paths_in_each_loop){

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


			//path search
			
			for(int n=0; n<input_nodes.size()-1; n++){

				vector<local_planner_node*> sample_local_planner_node=ALL_sample_local_planner_node[n];
				local_planner_node* start=sample_local_planner_node[init_number_of_samples-2];
				local_planner_node* destination =sample_local_planner_node[init_number_of_samples-1];
	
				//********************************** find the shortest path ********************************
				clock_t path_find_tStart = clock();
				//cout<<"finding path for n="<<n<<endl;
				if (find_the_shortest_path(start, destination, sample_local_planner_node)){
	 				total_dist_travel_gpu[j]=total_dist_travel_gpu[j]+destination->dist_from_start;
					write_in_a_file(sample_local_planner_node, start, destination, generate_path_output_file_name(n));
				

				}else{
					cout<<"couldn't found the path"<<endl;
					path_not_found_gpu[j]++;
				}
				clock_t path_find_tEnd = clock();
			
				
				total_time_path_find_gpu[j]=total_time_path_find_gpu[j]+(double)(path_find_tEnd - path_find_tStart)/CLOCKS_PER_SEC;
		
			}




			
		}else{
			//obstacles
			
			for(int p=0;p<ALL_sample_local_planner_node.size()-1;p=p+paths_in_each_loop){
				
				int start_index=p; int end_index; 
				if((p+paths_in_each_loop-1) <= (ALL_sample_local_planner_node.size()-1)){	
					end_index=p+paths_in_each_loop-1;
				}else{
					end_index=ALL_sample_local_planner_node.size()-1;
				}
				vector<vector<local_planner_node*> > temp_ALL_sample_node;
				

				for(int q=start_index; q<=end_index;q++){
					temp_ALL_sample_node.push_back(ALL_sample_local_planner_node[q]);

				}
				//cout<<"temp_ALL_sample_node size="<<temp_ALL_sample_node.size()<<endl;
				

				float h_obstacles[number_of_obstacles*4]; copy_obstacles(h_obstacles,obstacles);
				float *d_obstacles; cudaMalloc((void**)&d_obstacles, sizeof(float)*number_of_obstacles*4) ; // device
			

				//x,y,z coordinates of sample nodes 
				const int element_size_array=((int) ceil((double)init_number_of_samples/warp_size)*warp_size)*(temp_ALL_sample_node.size());
				float xyz[element_size_array*3]; copy_the_coordinates(xyz, temp_ALL_sample_node);//host
				float *d_xyz; cudaMalloc((void**)&d_xyz, sizeof(float)*element_size_array*3) ; // device
	
				//knn_neighbour_nodes_no ->serial no in the vector
				size_t neighbors_serial_no_size= sizeof(int) *init_number_of_samples*number_of_elements_for_knn * (temp_ALL_sample_node.size());
				int h_neighbors_serial_no[init_number_of_samples*number_of_elements_for_knn * (temp_ALL_sample_node.size())];
				int *d_neighbors_serial_no; cudaMalloc((void**)&d_neighbors_serial_no, neighbors_serial_no_size ) ; // device

				//knn_neighbour_corresponding_distances
				size_t neighbors_sq_distance_size= sizeof(float) * init_number_of_samples * number_of_elements_for_knn * (temp_ALL_sample_node.size());
				float h_neighbors_sq_distance[init_number_of_samples*number_of_elements_for_knn * (temp_ALL_sample_node.size())];
				float *d_neighbors_sq_distance; cudaMalloc((void**)&d_neighbors_sq_distance, neighbors_sq_distance_size) ; // device

				cudaMemcpy(d_xyz, xyz, sizeof(float)*element_size_array*3, cudaMemcpyHostToDevice);
				cudaMemcpy(d_obstacles, h_obstacles, sizeof(float)*number_of_obstacles*4, cudaMemcpyHostToDevice);
				cudaDeviceSynchronize();
				int blocks=number_of_blocks*(temp_ALL_sample_node.size());
				gpu_calculation<<< blocks, threads_per_block >>>(d_xyz, d_obstacles, d_neighbors_serial_no, d_neighbors_sq_distance);
				cudaDeviceSynchronize();
				cudaMemcpy(h_neighbors_serial_no,d_neighbors_serial_no, neighbors_serial_no_size, cudaMemcpyDeviceToHost);
				cudaMemcpy(h_neighbors_sq_distance,d_neighbors_sq_distance, neighbors_sq_distance_size, cudaMemcpyDeviceToHost);

				//UPDATE THE DATA
				typedef pair<local_planner_node*, float> apair;
				int counter=0;
				//cout<<endl<<endl;
				//int q=start_index;
				for(int k=0; k<(temp_ALL_sample_node.size()); k++){   //2; k++){
					//int k=0;
					for(int i=0;i<init_number_of_samples; i++){
						for(int j=0;j<number_of_elements_for_knn; j++){
							temp_ALL_sample_node[k][i]->knn.push_back(apair( temp_ALL_sample_node[k][h_neighbors_serial_no[counter]], h_neighbors_sq_distance[counter] ) ); 
							
					
							counter++;
						}	
						//cout<<endl<<endl;		
					}
					
					// path find	
					vector<local_planner_node*> sample_local_planner_node=temp_ALL_sample_node[k];
					local_planner_node* start=sample_local_planner_node[init_number_of_samples-2];
					local_planner_node* destination =sample_local_planner_node[init_number_of_samples-1];

					//********************************** find the shortest path ********************************
					clock_t path_find_tStart = clock();
					//cout<<"finding path for n="<<n<<endl;
					if (find_the_shortest_path(start, destination, sample_local_planner_node)){
	 					total_dist_travel_gpu[j]=total_dist_travel_gpu[j]+destination->dist_from_start;
						//write_in_a_file(sample_local_planner_node, start, destination, generate_path_output_file_name(k+p));
				

					}else{
						cout<<"couldn't found the path"<<endl;
						path_not_found_gpu[j]++;
					}
					clock_t path_find_tEnd = clock();
			
					total_time_path_find_gpu[j]=total_time_path_find_gpu[j]+(double)(path_find_tEnd - path_find_tStart)/CLOCKS_PER_SEC;
					
		
				}


			}


		}
		
		clock_t knn_process_tEnd = clock();

		total_time_knn_gpu[j]=total_time_knn_gpu[j]+(double)(knn_process_tEnd - knn_process_tStart)/CLOCKS_PER_SEC -total_time_path_find_gpu[j];
			
		
		cout<<"gpu iteration "<<j<<" completed."<<endl;
	}
	avg=0;
	cout<<"TOTAL TIME FOR KNN-GPU="<<endl;
	for(int j=0;j<iter;j++){
		cout<<total_time_knn_gpu[j]<<"  ";
		avg=avg+total_time_knn_gpu[j];
	}
	cout<<endl<<"mean="<<avg/iter<<endl;
	avg=0;
	cout<<"TOTAL TIME FOR path find-GPU="<<endl;
	for(int j=0;j<iter;j++){
		cout<<total_time_path_find_gpu[j]<<"  ";
		avg=avg+total_time_path_find_gpu[j];
	}
	cout<<endl<<"mean="<<avg/iter<<endl;
	cout<<"Total path not found GPU = "<<endl;
	for(int j=0;j<iter;j++){
		cout<<path_not_found_gpu[j]<<"  ";
	}
	cout<<endl;
	cout<<"Total dist travel GPU = "<<endl;
	for(int j=0;j<iter;j++){
		cout<<total_dist_travel_gpu[j]<<"  ";
	}
	cout<<endl;


	clock_t tEnd = clock();

	//cout<<"	total_time_sample_generation= "<<total_time_sample_generation<<"sec"<<endl;
	printf("At the end: Total Time taken: %.6fs\n", (double)(tEnd - tStart)/CLOCKS_PER_SEC);

	//cout<<"ALL_sample_local_planner_node.size() = "<< ALL_sample_local_planner_node.size()<<endl;

	//obstacle volume
	
	return 0;

}

