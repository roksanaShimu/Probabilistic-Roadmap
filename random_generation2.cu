#include <unistd.h>
#include <stdio.h>
#include <iostream>

/* we need these includes for CUDA's random number stuff */
#include <curand.h>
#include <curand_kernel.h>

using namespace std;

#define N 5

#define MAX 100

/* this GPU kernel function is used to initialize the random states */
__global__ void init(unsigned int seed, curandState_t* states) {

  /* we have to initialize the state */
  curand_init(seed, /* the seed can be the same for each core, here we pass the time in from the CPU */
              blockIdx.x, /* the sequence number should be different for each core (unless you want all
                             cores to get the same sequence of numbers for some reason - use thread id! */
              0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
              &states[blockIdx.x]);
}

/* this GPU kernel takes an array of states, and an array of ints, and puts a random int into each */
__global__ void randoms(curandState_t* states, float* numbers) {
  /* curand works like rand - except that it takes a state as a parameter */
	numbers[blockIdx.x] = curand_uniform(&states[blockIdx.x])*100;//% 100;
	numbers[blockIdx.x+N] = curand_uniform(&states[blockIdx.x])*(50-20)+20;
}

int main( ) {
  /* CUDA's random number library uses curandState_t to keep track of the seed value
     we will store a random state for every thread  */
  curandState_t* states;

  /* allocate space on the GPU for the random states */
  cudaMalloc((void**) &states, 2*N * sizeof(curandState_t));

  /* invoke the GPU to initialize all of the random states */
  init<<<N, 1>>>(time(0), states);

  /* allocate an array of unsigned ints on the CPU and GPU */
  float cpu_nums[2*N];
  float* gpu_nums;
  cudaMalloc((void**) &gpu_nums, 2*N * sizeof(float));

  /* invoke the kernel to get some random numbers */
  randoms<<<N, 1>>>(states, gpu_nums);

  /* copy the random numbers back */
  cudaMemcpy(cpu_nums, gpu_nums, 2* N * sizeof(float), cudaMemcpyDeviceToHost);

  /* print them out */
  for (int i = 0; i < 2*N; i++) {
   // printf("%u\n", cpu_nums[i]);
	cout<<cpu_nums[i]<<endl;
  }

  /* free the memory we allocated for the states and numbers */
  cudaFree(states);
  cudaFree(gpu_nums);

  return 0;
}


