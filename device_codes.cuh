#ifndef device_codes
#define device_codes


#include<iostream>
#include <stdint.h>
#include <inttypes.h>

#include "predefined_variables.h"
#include "data_structures.h"
//#include "data_structures_NEW.cuh"
//#include "milestones.h"
//#include "update_z_number.h"
//#include "update_k_neighbors.h"
//#include "ann_update_k_neighbors.h"
//#include "find_shortest_path.h"
#include "device_BitonicSort_prm.cuh"



void copy_obstacles(float h_obstacles[],vector<Sphere>obstacles){

	int counter=0;
	for(int i=0;i<number_of_obstacles;i++){
		h_obstacles[counter]=obstacles[i].p.x; counter++;
		h_obstacles[counter]=obstacles[i].p.y; counter++;
		h_obstacles[counter]=obstacles[i].p.z; counter++;
		h_obstacles[counter]=obstacles[i].radius; counter++;
	}

}		


void copy_the_coordinates(float xyz[], vector<vector<local_planner_node*> > ALL_sample_local_planner_node){
	for(int j=0;j<ALL_sample_local_planner_node.size();j++){
		for(int i=0;i<ALL_sample_local_planner_node[j].size();i++){
			int group = i/warp_size;
			int offset = ((int) ceil((double)init_number_of_samples/warp_size)*warp_size )*3* j;
			int x_index=group*warp_size*3 + i%warp_size + offset;
			int y_index= x_index+ warp_size*1;
			int z_index= x_index+ warp_size*2;
		
			xyz[x_index]=ALL_sample_local_planner_node[j][i]->p.x;
			xyz[y_index]=ALL_sample_local_planner_node[j][i]->p.y;
			xyz[z_index]=ALL_sample_local_planner_node[j][i]->p.z;

		}
	}
	
}

__device__ float distanceSquared(float x, float y, float z, float o_x, float o_y, float o_z) {

	return  (o_x - x)* (o_x - x) + (o_y - y)*(o_y - y) + (o_z - z)*(o_z - z);
}

__device__ bool d_doesItCollide(float x, float y, float z, float o_x, float o_y, float o_z, float o_radius){

	float rSquared = UAV_radius + o_radius;
    	rSquared *= rSquared;
    	
	return distanceSquared(x, y, z, o_x, o_y, o_z) < rSquared;

}

__device__ bool d_path_can_be_build(float s_x, float s_y, float s_z, float e_x, float e_y, float e_z, float *d_obstacles){
	

	for(float t=0.1;t<1;t=t+0.1){
		//cout<<t<< "    "<<i<<endl;
		float inter_x = s_x + (e_x - s_x)*t;
		float inter_y = s_y + (e_y - s_y)*t;
		float inter_z = s_z + (e_z - s_z)*t;
		//local_sample[i].radius=UAV_radius;
		for(int j=0;j<number_of_obstacles ;j++){
			
			if(d_doesItCollide(inter_x, inter_y, inter_z, d_obstacles[j*4], d_obstacles[j*4+1], d_obstacles[j*4+2], d_obstacles[j*4+3] )){
				//cout<<"collides with obstacle x: "<<obstacles[j].p.x<<" y: "<<obstacles[j].p.y<<" z: "<<obstacles[j].p.z<<" radius: "<<obstacles[j].radius<<" and the UAV location is x: "<< local_sample[i].p.x<<" y: "<<local_sample[i].p.y<<" z: "<<local_sample[i].p.z<<endl<<endl;
				return false;
			}
		}
	}


	return true;
}

__global__ void gpu_calculation(float *d_xyz, float *d_obstacles,  int *d_neighbors_serial_no, float *d_neighbors_sq_distance ){

	int ith_data_set= blockIdx.x/number_of_blocks;	
	int local_block_id=blockIdx.x%number_of_blocks;
	int group_no=local_block_id/warp_size;
	int offset=((int) ceil((double)init_number_of_samples/warp_size)*warp_size )*3* ith_data_set;
	
	int x_index=group_no*warp_size*3 +local_block_id%warp_size + offset;

	float x=d_xyz[x_index];
	float y=d_xyz[x_index+warp_size];
	float z=d_xyz[x_index+warp_size*2];

	__syncthreads();

	__shared__ float sq_distance[init_number_of_samples]; 
	__shared__ int serial_no[init_number_of_samples]; 
		
	for(int i=threadIdx.x; i<init_number_of_samples; i=i+threads_per_block){
		//if(blockIdx.x==0)printf("i=%d\n",i);
		if(i!=local_block_id){
			group_no=i/warp_size; x_index=group_no*warp_size*3 + i%warp_size + offset;

			float e_x=d_xyz[x_index]; float e_y=d_xyz[x_index+warp_size]; float e_z=d_xyz[x_index+warp_size*2];

			//if (blockIdx.x==0)printf("i=%d  x=%.2f, y=%.2f and z=%.2f\n", i, e_x, e_y, e_z);

			//if a path can be built
			if(d_path_can_be_build(x, y, z, e_x, e_y, e_z, d_obstacles)){                        
				//calculate the distance
				sq_distance[i]=sqrt(distanceSquared(x, y, z, e_x, e_y, e_z));
				//printf("squared distance=%.3f\n", distanceSquared(x, y, z, e_x, e_y, e_z));
				serial_no[i]=i;

			}else{//else put max float number as distance

				sq_distance[i]=FLT_MAX; 
				serial_no[i]=i;
			}
		}else{
			sq_distance[i]=FLT_MAX; 
			serial_no[i]=i;
		}

	}

	__syncthreads();

	/*if(blockIdx.x==1){
		if(threadIdx.x==0){
			for(int i=0; i<init_number_of_samples; i++){
				//printf("threadIdx.x =%d, i= %d, serial_no=%d\n",threadIdx.x, i, serial_no[i]);
				printf("i= %d, serial_no=%d, sq_distance=%.3f \n", i, serial_no[i], sq_distance[i]);
			}
		}
	//}

	__syncthreads();*/

	bitonic_sort(sq_distance, serial_no);

	__syncthreads();

	/*if(blockIdx.x==12){
		if(threadIdx.x==0){
			for(int i=0; i<init_number_of_samples; i++){
				//printf("threadIdx.x =%d, i= %d, serial_no=%d\n",threadIdx.x, i, serial_no[i]);
				printf("after: block=%d,  i= %d, serial_no=%d, sq_distance=%.3f \n", blockIdx.x, i, serial_no[i], sq_distance[i]);
			}
		}
	//}
	__syncthreads();*/

	int offset_output=ith_data_set*number_of_elements_for_knn*init_number_of_samples;

	if(threads_per_block >= 2*number_of_elements_for_knn){
		if(threadIdx.x<number_of_elements_for_knn){
			d_neighbors_sq_distance[local_block_id*number_of_elements_for_knn+threadIdx.x +offset_output] = sq_distance[threadIdx.x];
		}else if(threadIdx.x>=number_of_elements_for_knn && threadIdx.x<2*number_of_elements_for_knn){
			unsigned int t_id=threadIdx.x-number_of_elements_for_knn;
			d_neighbors_serial_no[local_block_id*number_of_elements_for_knn+t_id+offset_output] = serial_no[t_id];
		}
		

	}else if (threads_per_block >= number_of_elements_for_knn){

		if(threadIdx.x<number_of_elements_for_knn){
			d_neighbors_sq_distance[local_block_id*number_of_elements_for_knn+threadIdx.x+offset_output] = sq_distance[threadIdx.x];
			d_neighbors_serial_no[local_block_id*number_of_elements_for_knn+threadIdx.x+offset_output] =  serial_no[threadIdx.x];

		}

	}else{
		for(unsigned int t_id=threadIdx.x; t_id<number_of_elements_for_knn; t_id=t_id+threads_per_block){
			d_neighbors_sq_distance[local_block_id*number_of_elements_for_knn+t_id+offset_output] = sq_distance[t_id];
			d_neighbors_serial_no[local_block_id*number_of_elements_for_knn+t_id+offset_output] =  serial_no[t_id];

		}

	}


	__syncthreads();
}










#endif
