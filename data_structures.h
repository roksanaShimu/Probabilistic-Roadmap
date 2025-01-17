#ifndef data_structures
#define data_structures

#include <stdint.h>
#include <vector>
#include <cfloat>
#include <cmath>
#include <cstdlib>


#include "predefined_variables.h"

using namespace std;

struct position{
	float x, y, z;
};


class local_planner_node{
private:

	uint64_t splitBy3(uint32_t a){
		uint64_t x = a & 0x1fffff; // we only look at the first 21 bits
    		x = (x | x << 32) & 0x1f00000000ffff;  // shift left 32 bits, OR with self, and  00011111000000000000000000000000000000001111111111111111
    		x = (x | x << 16) & 0x1f0000ff0000ff;  // shift left 32 bits, OR with self, and 00011111000000000000000011111111000000000000000011111111
    		x = (x | x << 8) & 0x100f00f00f00f00f; // shift left 32 bits, OR with self, and 0001000000001111000000001111000000001111000000001111000000000000
    		x = (x | x << 4) & 0x10c30c30c30c30c3; // shift left 32 bits, OR with self, and 0001000011000011000011000011000011000011000011000011000100000000
    		x = (x | x << 2) & 0x1249249249249249;
    		return x;

	}
public:
	position p;

	int index;  //temporary variable for debugging
	float radius;
    	uint64_t z_number;

	bool visited;
	local_planner_node* source;
	float dist_from_start;

	bool in_the_min_heap;

	typedef pair<local_planner_node*, float> apair;
	vector<pair<local_planner_node*, float> >knn;    // nearest neighbors and corresponding distances
	
	local_planner_node(){    //constructor
		p.x=0.0; p.y=0.0; p.z=0.0; radius=UAV_radius; z_number=0;
		visited=false; dist_from_start=FLT_MAX;//999999999;//FLT_MAX;
		in_the_min_heap=false; source=this;
	}

	void set_local_planner_node(float a, float b, float c){
		p.x=a; p.y=b; p.z=c;
	}

	
	void update_Znumber(){
	
		uint32_t xpos=round(p.x);
		uint32_t ypos=round(p.y);
		uint32_t zpos=round(p.z);
		//cout<<"xpos="<<xpos<<"   ypos="<<ypos<<"   zpos="<<zpos;
	
		z_number=0;
		z_number |= splitBy3(xpos) | splitBy3(ypos) << 1 | splitBy3(zpos) << 2;
		//cout<<"   z_number="<<z_number<<endl;*/		
	}


	

	

};


struct Sphere {
    	position p;
    	float radius;
};



class min_heap{
private:
	vector<local_planner_node*> mheap;

	void up_heappify(int j){
		//cout<<"in the up_heappify and j= "<<j<<endl;	
		if(j==0) return;

		int i;
		if(j%2==0){
			i=(j-2)/2;
		}else{
			i=(j-1)/2;
		}
		
		if (mheap[i]->dist_from_start > mheap[j]->dist_from_start){
			//swap i, j
			local_planner_node* temp=mheap[i];
			mheap[i]=mheap[j];
			mheap[j]=temp;
			up_heappify(i);
		}		
	}

	int find_min(int k, int m){
		if(mheap[k]->dist_from_start < mheap[m]->dist_from_start){
			return k;
		}else{
			return m;
		}
	}

	void down_heappify(int i){
		//cout<<"in the down_heappify and i= "<<i<<endl;	
		if(2*i+1 > mheap.size()-1) return;
		int minimum;
		if(2*i+2 <= mheap.size()-1){ //two children exists
			minimum=find_min(i, find_min(2*i+1, 2*i+2));
		}else{				// one child only
			minimum=find_min(i, 2*i+1);
		}
		if(minimum!=i){
			//swap i and min
			local_planner_node* temp=mheap[i];
			mheap[i]=mheap[minimum];
			mheap[minimum]=temp;
			down_heappify(minimum);
		}
		return;
	}
public:
	min_heap(){   // constructor;
		
	}
	int size(){
		return mheap.size();
	}

	void add_node(local_planner_node*& n){
		n->in_the_min_heap=true;
		mheap.push_back(n);
		
		if (mheap.size()>1){
			up_heappify(mheap.size()-1);
		}
	}

	void print_the_vector(){
		for(int i=0; i<mheap.size(); i++){
			cout<<mheap[i]->dist_from_start<<"  ";
		}
		cout<<endl;
	}

	local_planner_node* get_node(){
		//cout<<mheap.size()<<endl;
		
		if(mheap.size()==0){
			//cout<<"mheap is empty"<<endl;
			return NULL;
		}

		mheap[0]->in_the_min_heap=false;
		local_planner_node* n=mheap[0];

		mheap[0]=mheap[mheap.size()-1];
		mheap.erase(mheap.begin() + (mheap.size()-1));
		if(mheap.size()>1){ 
			down_heappify(0);
		}

		return n;
	}

	void update_the_heap(local_planner_node* n){  // if a shorter path is found, the heap will be updated accordingly
		
		 
		for(int i=0;i<mheap.size();i++){
			if(mheap[i]==n){
				//cout<<"found at i= "<<i<<endl;
				up_heappify(i);
				break;
			}
		}
		

	}
	bool isempty(){
		if(mheap.size()==0)return true;
		else return false;
	}

	
};






#endif
