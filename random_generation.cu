
#include <unistd.h>
#include <stdio.h>

#include <curand.h>
#include <curand_kernel.h>

#define N 1024

__global__ void setup_kernel ( curandState * state, unsigned long seed )
{
    int id = threadIdx.x;
    curand_init ( seed, id, 0, &state[id] );
} 

__global__ void generate( curandState* globalState ) 
{
    int ind = threadIdx.x;
    curandState localState = globalState[ind];
    float RANDOM = curand_uniform( &localState );
 printf("ind= %d, and number =\n",ind);
    globalState[ind] = localState; 
}

int main( int argc, char** argv) 
{
    dim3 tpb(N,1,1);
    curandState* devStates;
    cudaMalloc ( &devStates, N*sizeof( curandState ) );
    
    // setup seeds
    setup_kernel <<< 1, tpb >>> ( devStates, time(NULL) );

    // generate random numbers
    generate <<< 1, tpb >>> ( devStates );

    return 0;
}


