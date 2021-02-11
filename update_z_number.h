#ifndef update_z_number
#define update_z_number

#include <stdint.h>
#include<vector>


#include "predefined_variables.h"

using namespace std;


/*
uint64_t splitBy3(uint32_t a){
	uint64_t x = a & 0x1fffff; // we only look at the first 21 bits
    	x = (x | x << 32) & 0x1f00000000ffff;  // shift left 32 bits, OR with self, and  00011111000000000000000000000000000000001111111111111111
    	x = (x | x << 16) & 0x1f0000ff0000ff;  // shift left 32 bits, OR with self, and 00011111000000000000000011111111000000000000000011111111
    	x = (x | x << 8) & 0x100f00f00f00f00f; // shift left 32 bits, OR with self, and 0001000000001111000000001111000000001111000000001111000000000000
    	x = (x | x << 4) & 0x10c30c30c30c30c3; // shift left 32 bits, OR with self, and 0001000011000011000011000011000011000011000011000011000100000000
    	x = (x | x << 2) & 0x1249249249249249;
    	return x;

}

void calZnumber(local_planner_node* S){
	
	uint32_t xpos=round(S->p.x);
	uint32_t ypos=round(S->p.y);
	uint32_t zpos=round(S->p.z);
	//cout<<"xpos="<<xpos<<"   ypos="<<ypos<<"   zpos="<<zpos;
	
	S->z_number=0;
	S->z_number |= splitBy3(xpos) | splitBy3(ypos) << 1 | splitBy3(zpos) << 2;
	//cout<<"   z_number="<<S.z_number<<endl;		
	
}

void update_z_numbers(vector<local_planner_node*>& sample_local_planner_node){
	
	for(int i=0;i<sample_local_planner_node.size();i++){

		calZnumber(sample_local_planner_node[i]);

	}

}
*/

void update_z_numbers(vector<local_planner_node*>& sample_local_planner_node){
	
	for(int i=0;i<sample_local_planner_node.size();i++){

		sample_local_planner_node[i]->update_Znumber();

	}

}

void swap(vector<local_planner_node*>& S, int a, int b){

	local_planner_node* D=S[a];
	S[a]=S[b];
	S[b]=D;
}

void Quick_sort(vector<local_planner_node*>& S, int begin, int end){
	//cout<<"begin: "<<begin<<"  end: "<<end<<endl;
	
	if(begin>=end) return;

	// generate a random number between begin and end
	int r=rand()%(end-begin+1)+begin;	
	
	swap(S, r, end);

	uint64_t t=S[end]->z_number;
	
	int i=begin-1; 
	
	for(int j=begin;j<end;j++){
		if(t>S[j]->z_number){
			i++;
			swap(S, i,j);	
		}
	}

	i++;
	swap(S, i, end);
	
	Quick_sort(S, begin, i-1);
	Quick_sort(S, i+1, end);

	
	
}
void sort_according_to_z_number(vector<local_planner_node*>& sample_local_planner_node){
	if (sample_local_planner_node.size()==0) return;

	Quick_sort(sample_local_planner_node, 0, sample_local_planner_node.size()-1);
}


void print_z_numbers(vector<local_planner_node*> sample_local_planner_node){
	for(int i =0; i<sample_local_planner_node.size();i++){
		cout<<"i= "<<i<<"  x= "<<sample_local_planner_node[i]->p.x<<"  y= "<<sample_local_planner_node[i]->p.y<<"  z= "<<sample_local_planner_node[i]->p.z<<"  z_no= "<<sample_local_planner_node[i]->z_number<<endl; 
	}
}

#endif
