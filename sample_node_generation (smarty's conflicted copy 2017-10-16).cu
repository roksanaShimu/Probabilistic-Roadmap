#include<iostream>
#include <stdint.h>
#include <inttypes.h>
#include <cmath>
#include <sstream>

#include "predefined_variables.h"
#include "data_structures.h"
#include "milestones.h"
//#include "update_z_number.h"
//#include "update_k_neighbors.h"
//#include "ann_update_k_neighbors.h"
#include "find_shortest_path.h"
#include "device_BitonicSort_prm.cuh"
#include "device_codes.cuh"
#include "function_codes.cuh"
#include "obstacle_generation_file.cuh"

// we need these includes for CUDA's random number stuff 
#include <curand.h>
#include <curand_kernel.h>

using namespace std;

#define N (init_number_of_samples-2)* number_of_waypoints 

__global__ void init(unsigned int seed, curandState_t* states) {

  	/* we have to initialize the state */
  	curand_init(seed, /* the seed can be the same for each core, here we pass the time in from the CPU */
        blockIdx.x, /* the sequence number should be different for each core (unless you want all
                             cores to get the same sequence of numbers for some reason - use thread id! */
        0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
           &states[blockIdx.x]);
}

__global__ void sample_gpu_calculation(float *d_input_n, float *d_obstacles, float *d_xyz_limit, int *d_sample_grid_size, curandState_t* states, float * d_result){
	
	int id= blockIdx.x*(init_number_of_samples-2)+threadIdx.x;
	__syncthreads();
	float s_x=d_input_n[blockIdx.x*3];
	float s_y=d_input_n[blockIdx.x*3+1];
	float s_z=d_input_n[blockIdx.x*3+2];

	float d_x=d_input_n[(blockIdx.x+1)*3];
	float d_y=d_input_n[(blockIdx.x+1)*3+1];
	float d_z=d_input_n[(blockIdx.x+1)*3+2];


	float x_start=d_xyz_limit[blockIdx.x*6] ; float x_end= d_xyz_limit[blockIdx.x*6+1] ;
	float y_start=d_xyz_limit[blockIdx.x*6+2] ; float y_end= d_xyz_limit[blockIdx.x*6+3] ;
	float z_start=d_xyz_limit[blockIdx.x*6+4] ; float z_end= d_xyz_limit[blockIdx.x*6+5] ;
	int nodes_in_x=d_sample_grid_size[blockIdx.x*3];
	int nodes_in_y=d_sample_grid_size[blockIdx.x*3+1];
	int nodes_in_z=d_sample_grid_size[blockIdx.x*3+2];
	__syncthreads();
	float x,y,z;

	if (threadIdx.x<(nodes_in_x*nodes_in_y*nodes_in_z)){
		int z_d=threadIdx.x/(nodes_in_x*nodes_in_y);
		z=z_start+z_d* (z_end-z_start)/(nodes_in_z-1);
		
		int new_id=threadIdx.x% (nodes_in_x* nodes_in_y);
		int y_d=new_id/nodes_in_x;
		y = y_start + y_d*(y_end-y_start)/(nodes_in_y-1);
		
		int x_d= new_id% nodes_in_x;
		x = x_start + x_d*(x_end-x_start)/(nodes_in_x-1);
		bool collide=false;
		for(int j=0;j<number_of_obstacles;j++){
			collide=d_doesItCollide(x,y,z,d_obstacles[j*4], d_obstacles[j*4+1], d_obstacles[j*4+2], d_obstacles[j*4+3] );
			if (collide==true){
				break;
			}

		}
		while (collide==true){
			//generate random node
			x=curand_uniform(&states[id])*(x_end-x_start)+x_start;
			y=curand_uniform(&states[id])*(y_end-y_start)+y_start;
			z=curand_uniform(&states[id])*(z_end-z_start)+z_start;
			for(int j=0;j<number_of_obstacles;j++){
				collide=d_doesItCollide(x,y,z,d_obstacles[j*4], d_obstacles[j*4+1], d_obstacles[j*4+2], d_obstacles[j*4+3] );
				if (collide==true){
					break;
				}

			}
		}
		
		
	}else{
		bool collide=true;
		while (collide==true){
			//generate random node
			x=curand_uniform(&states[id])*(x_end-x_start)+x_start;
			y=curand_uniform(&states[id])*(y_end-y_start)+y_start;
			z=curand_uniform(&states[id])*(z_end-z_start)+z_start;
			for(int j=0;j<number_of_obstacles;j++){
				collide=d_doesItCollide(x,y,z,d_obstacles[j*4], d_obstacles[j*4+1], d_obstacles[j*4+2], d_obstacles[j*4+3] );
				if (collide==true){
					break;
				}

			}
		}
	}
	__syncthreads();
	d_result[id*3]=x;d_result[id*3+1]=y;d_result[id*3+2]=z;
	
	__syncthreads();
}


void parallel_generate_grid_samples(vector<float> xyz_limit, float gap[], vector<int> sample_grid_size, vector<Sphere> obstacles, vector<local_planner_node*>& sample_local_planner_node){

	position k;

	for( int id=0; id< (sample_grid_size[0]* sample_grid_size[1]* sample_grid_size[2]); id++){

		int z_d=id/(sample_grid_size[0]*sample_grid_size[1]);
		k.z=xyz_limit[4]+z_d* gap[2];
		
		int new_id=id% (sample_grid_size[0]* sample_grid_size[1]);
		int y_d=new_id/sample_grid_size[0];
		k.y = xyz_limit[2] + y_d*gap[1];
		
		int x_d= new_id% sample_grid_size[0];
		k.x = xyz_limit[0] + x_d*gap[0];
		
		//cout<<"id="<<id<<"  z_d="<<z_d<< "  z="<<k.z<<"  y_d="<<y_d<<"  y="<<k.y<<"  x_d="<<x_d<<"  x="<<k.x<<endl;

		if( collides_with_obstacle(k,obstacles)==false){
			local_planner_node* s=new local_planner_node();
			s->p=k;
			sample_local_planner_node.push_back(s); 
		}else{ //generate radomly

		}

	}





	
}

void parallel_generate_random_samples(vector<local_planner_node*>& sample_local_planner_node,vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, int j, vector <float> xyz_limit){
	position k; 
	for (int i=j;i<init_number_of_samples-2;i++){
		local_planner_node* s=new local_planner_node(); 
		
		
		k.x = ( (float)rand()/(float)(RAND_MAX/xyz_limit[1]) )+xyz_limit[0];
		k.y = ( (float)rand()/(float)(RAND_MAX/xyz_limit[3]) )+xyz_limit[2];
		k.z = ( (float)rand()/(float)(RAND_MAX/xyz_limit[5]) )+xyz_limit[4];
		while (not_in_map_limit(k) | collides_with_obstacle(k,obstacles)){
			//cout<<"either exceeded limit. or collides  x= "<<k.x<<",   y= "<<k.y<<",   z= "<<k.z<<endl;    
			k.x = ( (float)rand()/(float)(RAND_MAX/xyz_limit[1]) )+xyz_limit[0];
			k.y = ( (float)rand()/(float)(RAND_MAX/xyz_limit[3]) )+xyz_limit[2];
			k.z = ( (float)rand()/(float)(RAND_MAX/xyz_limit[5]) )+xyz_limit[4];
		}

		//cout<<k.x<<"     "<<k.y<<"     "<<k.z<<endl;
		s->p=k;
		sample_local_planner_node.push_back(s); 
		 
	}
	sample_local_planner_node.push_back(start); sample_local_planner_node.push_back(destination);



}



vector<local_planner_node*> parallel_generate_grid_milestone_sample_local_planner_node(vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, vector<int> sample_grid_size, vector <float> xyz_limit){

	
	vector<local_planner_node*> sample_local_planner_node;


	calculate_the_cordinate_limit(xyz_limit);

	
	float gap[3];
	find_the_gap(xyz_limit, gap, sample_grid_size);

	//cout<<"xyz_limits and gaps and sample_grid_size:"<<endl;
	//cout<< "x: "<<xyz_limit[0]<<" "<< xyz_limit[1]<<"      & "<<gap[0]<<"      & "<<sample_grid_size[0]<<endl; 
	//cout<< "y: "<<xyz_limit[2]<<" "<< xyz_limit[3]<<"      & "<<gap[1]<<"      & "<<sample_grid_size[1]<<endl;
	//cout<< "z: "<<xyz_limit[4]<<" "<< xyz_limit[5]<<"      & "<<gap[2]<<"      & "<<sample_grid_size[2]<<endl;
	parallel_generate_grid_samples(xyz_limit, gap, sample_grid_size, obstacles, sample_local_planner_node);

	parallel_generate_random_samples(sample_local_planner_node, obstacles, start, destination, sample_local_planner_node.size(), xyz_limit);

	/*cout<<sample_local_planner_node.size()<<endl;
	for(int i=0; i<sample_local_planner_node.size();i++){
		cout<<sample_local_planner_node[i]->p.x<<"  "<<sample_local_planner_node[i]->p.y<<"  "<< sample_local_planner_node[i]->p.z<<endl;
	}*/
	

	return sample_local_planner_node;

}





int main(){

	vector<position> input_nodes=read_input_nodes();
	if (input_nodes.size()<=1){
		cout<<"Error: Not enough nodes to find path"<<endl;
		return 0;
	} 
	

	//****************************read obstacles************************************************
	//generate_obstacle(input_nodes);
	vector<Sphere>obstacles; obstacles.resize(number_of_obstacles);
	read_the_obstacles(obstacles);
	

	

	clock_t sample_gen_tStart=clock();
	//obstacles
	float h_obstacles[number_of_obstacles*4]; copy_obstacles(h_obstacles,obstacles);
	float *d_obstacles; cudaMalloc((void**)&d_obstacles, sizeof(float)*number_of_obstacles*4) ; // device
		
	
	float h_input_n[input_nodes.size()*3]; 
	for (int i=0; i<input_nodes.size(); i++){
		h_input_n[i*3]=input_nodes[i].x; 
		h_input_n[i*3+1]=input_nodes[i].y;
		h_input_n[i*3+2]=input_nodes[i].z;

	}
	
	float *d_input_n; cudaMalloc((void**)&d_input_n, sizeof(float)*input_nodes.size()*3) ; // device
		
	/* CUDA's random number library uses curandState_t to keep track of the seed value
     		we will store a random state for every thread  */
  		curandState_t* states;

  	/* allocate space on the GPU for the random states */
  	cudaMalloc((void**) &states, 2*N * sizeof(curandState_t));

  	/* invoke the GPU to initialize all of the random states */
 	init<<<N, 1>>>(time(0), states);

	//generate the xyz limits
	//generate the gaps

	float h_xyz_limit[(input_nodes.size()-1)*6]; 
	float *d_xyz_limit; cudaMalloc((void**)&d_xyz_limit, sizeof(float)*(input_nodes.size()-1)*6) ; // device
		
	
	int h_sample_grid_size[(input_nodes.size()-1)*3];
	int *d_sample_grid_size; cudaMalloc((void**)&d_sample_grid_size, sizeof(int)*(input_nodes.size()-1)*3) ; // device
		
	
	for(int n=0; n<input_nodes.size()-1; n++){
		//******************initialize the start and the destination location***********************
		local_planner_node* start= new local_planner_node();
		start->set_local_planner_node(input_nodes[n].x, input_nodes[n].y, input_nodes[n].z);
		cout<<"start: " <<start->p.x<<"  "<<start->p.y<<"   "<<start->p.z<<endl;
		start->dist_from_start=0;

		local_planner_node* destination = new local_planner_node();
		destination->set_local_planner_node(input_nodes[n+1].x, input_nodes[n+1].y, input_nodes[n+1].z);
		cout<<"destination: "<<destination->p.x<<"  "<<destination->p.y<<"   "<<destination->p.z<<endl;

		vector<float> xyz_limit(6);		
		vector<int> sample_grid_size =find_the_sample_grid_size(start->p, destination->p, xyz_limit);
		
		h_xyz_limit[n*6]=xyz_limit[0]; h_xyz_limit[n*6+1]=xyz_limit[1];
		h_xyz_limit[n*6+2]=xyz_limit[2]; h_xyz_limit[n*6+3]=xyz_limit[3];
		h_xyz_limit[n*6+4]=xyz_limit[4]; h_xyz_limit[n*6+5]=xyz_limit[5];

		h_sample_grid_size[n*3]=sample_grid_size[0];	
		h_sample_grid_size[n*3+1]=sample_grid_size[1];
		h_sample_grid_size[n*3+2]=sample_grid_size[2];

	}
	
	float h_result[(init_number_of_samples-2)*(input_nodes.size()-1)*3];
	float *d_result; cudaMalloc((void**)&d_result, sizeof(float)*((init_number_of_samples-2)*(input_nodes.size()-1)*3)) ; // device


	cudaMemcpy(d_input_n, h_input_n, sizeof(float)*input_nodes.size()*3, cudaMemcpyHostToDevice);
	cudaMemcpy(d_obstacles, h_obstacles, sizeof(float)*number_of_obstacles*4, cudaMemcpyHostToDevice);
	cudaMemcpy(d_xyz_limit, h_xyz_limit, sizeof(float)*(input_nodes.size()-1)*6, cudaMemcpyHostToDevice);
	cudaMemcpy(d_sample_grid_size, h_sample_grid_size, sizeof(int)*(input_nodes.size()-1)*3, cudaMemcpyHostToDevice);



	cudaDeviceSynchronize();
	int blocks=input_nodes.size()-1;
	int threads_per_block_sample_gen=init_number_of_samples-2;
	sample_gpu_calculation<<< blocks, threads_per_block_sample_gen >>>(d_input_n, d_obstacles, d_xyz_limit, d_sample_grid_size, states, d_result);
	cudaDeviceSynchronize();


	cudaMemcpy(h_result, d_result, sizeof(float)*(init_number_of_samples-2)*(input_nodes.size()-1)*3, cudaMemcpyDeviceToHost);

	clock_t sample_gen_tEnd=clock();

//*********************************************************************************************************
	double total_time_sample_generation=0;
	vector<vector<local_planner_node*> > ALL_sample_local_planner_node;
	for(int n=0; n<input_nodes.size()-1; n++){
		

		//******************initialize the start and the destination location***********************
		local_planner_node* start= new local_planner_node();
		start->set_local_planner_node(input_nodes[n].x, input_nodes[n].y, input_nodes[n].z);
		//cout<<"start: " <<start->p.x<<"  "<<start->p.y<<"   "<<start->p.z<<endl;
		start->dist_from_start=0;

		local_planner_node* destination = new local_planner_node();
		destination->set_local_planner_node(input_nodes[n+1].x, input_nodes[n+1].y, input_nodes[n+1].z);
		//cout<<"destination: "<<destination->p.x<<"  "<<destination->p.y<<"   "<<destination->p.z<<endl;

		
		//************************boundary conditions***********************************************
		//check if the start and destination location collides with any obstacles
		if(!boundary_condition_satisfied(start, destination, obstacles)){
			cout<<"boundary condition doesn't satisfied either for start or destination"<<endl;
			//return 0;
		}

		
		//******************* find the grid size for putting the samples****************************

		vector<float> xyz_limit(6);		
		vector<int> sample_grid_size =find_the_sample_grid_size(start->p, destination->p, xyz_limit);
		//cout<<" nodes_in_x = "<<sample_grid_size[0]<<" nodes_in_y = "<<sample_grid_size[1]<<" nodes_in_z = "<<sample_grid_size[2]<<endl;

		//cout<<"x_start="<<xyz_limit[0]<<", x_end="<<xyz_limit[1]<<", y_start="<<xyz_limit[2]<<", y_end="<<xyz_limit[3]<<", z_start="<<xyz_limit[4]<<", z_end="<<xyz_limit[5]<<endl;



		//**************************generate the sample locations***********************************
		vector<local_planner_node*> sample_local_planner_node;
				
		clock_t sample_generation_tStart = clock();
		
		sample_local_planner_node=parallel_generate_grid_milestone_sample_local_planner_node(obstacles, start, destination, sample_grid_size, xyz_limit );
		clock_t sample_generation_tEnd = clock();
		total_time_sample_generation= total_time_sample_generation+(double)(sample_generation_tEnd - sample_generation_tStart)/CLOCKS_PER_SEC;
		
		ALL_sample_local_planner_node.push_back(sample_local_planner_node);

		//cout<<generate_sample_nodes_file_name(n)<<endl;

		//write_the_sample_nodes_in_a_file(sample_local_planner_node, generate_sample_nodes_file_name(n));
	}

	printf("At the end: sample gen time taken: %.6fs\n", (double)(sample_gen_tEnd - sample_gen_tStart)/CLOCKS_PER_SEC);


	return 0;
}
