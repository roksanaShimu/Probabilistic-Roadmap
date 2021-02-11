#include <stdio.h>
#include <curand.h>
#include <curand_kernel.h>
#include <math.h>
#include <assert.h>
#define MIN 2
#define MAX 7
#define ITER 10000000

__global__ void setup_kernel(curandState *state){

  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  curand_init(1234, idx, 0, &state[idx]);
}


__global__ void generate_kernel(curandState *my_curandstate, float *result){

  	int idx = threadIdx.x + blockDim.x*blockIdx.x;

  	int count = 0;
  	while (count < 100){
    		float myrandf = curand_uniform(my_curandstate+idx);
		printf("idx=%d and myrandf=%f\n", idx,myrandf);
		result[count]=myrandf;
		count++;
	}
   
}

int main(){
	curandState *d_state;
  	cudaMalloc(&d_state, sizeof(curandState));
  	float *d_result, *h_result;
  	
  	cudaMalloc(&d_result, (100) * sizeof(float));
  	h_result = (float *)malloc((100)*sizeof(float));
  	
  	
  	setup_kernel<<<1,1>>>(d_state);

  	
  	generate_kernel<<<1,1>>>(d_state, d_result);
  	cudaMemcpy(h_result, d_result, (100) * sizeof(float), cudaMemcpyDeviceToHost);
  	/*printf("Bin:    Count: \n");
  	for (int i = MIN; i <= MAX; i++)
    		printf("%d    %d\n", i, h_result[i-MIN]);*/

  	return 0;
}
