#include<stdio.h>
#include<iostream>
#include<fstream>
#include<stdlib.h>
#include<cstdlib>
#include <time.h>
#include<cuda.h>
#include <cassert>
#include <curand.h>
#include<string>

#include "predefined_variables.h"
using namespace std;


#ifndef device_BitonicSort_prm
#define device_BitonicSort_prm

__device__ void bitonic_sort(float sq_distance[], int serial_no[]){
	//----------------------------------------------------------	
	//---------------------Bitonic Sort-------------------------
	//----------------------------------------------------------
	// Sorts the elements of array sq_distance ascendingly. The serial no includes the coresponding node numbers. 
	
	short unsigned int partition;
 	
	
	short unsigned int i,j,k,r, sml,lrg;
	float temp; int temp2;

	for(i=2;i<=init_number_of_samples;i=i*2){   //  threads_per_block;i=i*2){
		j=log2((float)i);
		sml=j;
		partition=1;
		while(sml>=1){
			for(unsigned int t_id=threadIdx.x; t_id<init_number_of_samples; t_id=t_id+threads_per_block){

				if(sml==j){
					r=t_id % i;
					if(r<i/2){
						
						lrg=t_id/i;
						unsigned int p=t_id+(i/2+i*(lrg)-t_id)*2-1;

						if(p<init_number_of_samples){
							if (sq_distance[t_id]>sq_distance[p]){
								temp=sq_distance[t_id];
								sq_distance[t_id]=sq_distance[p];
								sq_distance[p]=temp;

								temp2=serial_no[t_id];
								serial_no[t_id]=serial_no[p];
								serial_no[p]=temp2;
							}
						}
					}
								

				}else{
					k=i/(partition*2);
					r=t_id%k;
					if(r<k/2){
						unsigned int p=t_id+k/2;
						if(sq_distance[t_id]>sq_distance[p]){
							temp=sq_distance[t_id];
							sq_distance[t_id]=sq_distance[p];
							sq_distance[p]=temp;

							temp2=serial_no[t_id];
							serial_no[t_id]=serial_no[p];
							serial_no[p]=temp2;
						}
					}

					if((t_id+threads_per_block)>=init_number_of_samples){
						partition=partition*2;
			

					}
				}
			}
			
			sml--;
			__syncthreads();
				
		}
	}//end of Bitonic Sort

	__syncthreads();
	
	
}




/*  // the following codes works when the number of threads per block is the same as the number of elements to be sorted
__device__ void bitonic_sort(int64_t sq_distance[], int serial_no[]){
	//----------------------------------------------------------	
	//---------------------Bitonic Sort-------------------------
	//----------------------------------------------------------
	// Sorts the elements of array sq_distance ascendingly. The serial no includes the coresponding node numbers. 
	
	short unsigned int partition;
 	
	
	short unsigned int i,j,k,r, sml,lrg;
	int64_t temp; int temp2;

	for(i=2;i<=threads_per_block;i=i*2){
		j=log2((float)i);
		sml=j;
		partition=1;
		while(sml>=1){
			for(unsigned int t_id=threadIdx.x; t_id<init_number_of_samples; t_id=t_id+threads_per_block){

				if(sml==j){
					
					r=threadIdx.x % i;
					
					if(r<i/2){
						lrg=threadIdx.x/i;
						unsigned int p=threadIdx.x+(i/2+i*(lrg)-threadIdx.x)*2-1;

						if(p<init_number_of_samples){
							if (sq_distance[threadIdx.x]>sq_distance[p]){
								temp=sq_distance[threadIdx.x];
								sq_distance[threadIdx.x]=sq_distance[p];
								sq_distance[p]=temp;

								temp2=serial_no[threadIdx.x];
								serial_no[threadIdx.x]=serial_no[p];
								serial_no[p]=temp2;
							}
						}
						
					}
								

				}else{
					partition=partition*2;
					
					r=threadIdx.x%k;
					
					if(r<k/2){
						unsigned int p=threadIdx.x+k/2;
						if(sq_distance[threadIdx.x]>sq_distance[p]){
							temp=sq_distance[threadIdx.x];
							sq_distance[threadIdx.x]=sq_distance[p];
							sq_distance[p]=temp;

							temp2=serial_no[threadIdx.x];
							serial_no[threadIdx.x]=serial_no[p];
							serial_no[p]=temp2;
						}
					}
									}
			}
			
			sml--;
			__syncthreads();
				
		}
	}//end of Bitonic Sort

	__syncthreads();
	
	
}




*/




#endif
