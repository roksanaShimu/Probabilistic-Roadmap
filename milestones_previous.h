#ifndef milestones_previous
#define milestones_previous

#include <stdint.h>
#include<vector>
#include<fstream>
#include <cstdlib>

#include "predefined_variables.h"
#include "milestones.h"

//#define padding_for_grid_sample UAV_radius
using namespace std;

//***********************************************************************************************



//************************************************************************************************


//within the xyz limit find the grid size for placing the sample nodes



vector<int> find_the_sample_grid_size_previous(){
	vector<int> sample_grid_size;
	float p=(float)(init_number_of_samples-2) /((map_x-map_x_start) * (map_y-map_y_start) * (map_z-map_z_start)) ;
	p=pow(p,(1.0/3));

	cout<<"p = "<<p<<endl;

	int nodes_in_x, nodes_in_y, nodes_in_z;

	if (round((map_x-map_x_start)*p) * round((map_y-map_y_start)*p) * round((map_z-map_z_start)*p) <= (init_number_of_samples-2)){

		nodes_in_x = round((map_x-map_x_start)*p);
		nodes_in_y = round((map_y-map_y_start)*p); 
		nodes_in_z = round((map_z-map_z_start)*p);
	}else if(round((map_x-map_x_start)*p) * round((map_y-map_y_start)*p) * floor((map_z-map_z_start)*p) <= (init_number_of_samples-2)){
		nodes_in_x = round((map_x-map_x_start)*p);
		nodes_in_y = round((map_y-map_y_start)*p); 
		nodes_in_z = floor((map_z-map_z_start)*p);

	}else if(round((map_x-map_x_start)*p) * floor((map_y-map_y_start)*p) * round((map_z-map_z_start)*p) <= (init_number_of_samples-2)){
		nodes_in_x = round((map_x-map_x_start)*p);
		nodes_in_y = floor((map_y-map_y_start)*p); 
		nodes_in_z = round((map_z-map_z_start)*p);

	}else if(floor((map_x-map_x_start)*p) * round((map_y-map_y_start)*p) * round((map_z-map_z_start)*p) <= (init_number_of_samples-2)){
		nodes_in_x = floor((map_x-map_x_start)*p);
		nodes_in_y = round((map_y-map_y_start)*p); 
		nodes_in_z = round((map_z-map_z_start)*p);
	
	}else if(round((map_x-map_x_start)*p) * floor((map_y-map_y_start)*p) * floor((map_z-map_z_start)*p) <= (init_number_of_samples-2)){
		nodes_in_x = round((map_x-map_x_start)*p);
		nodes_in_y = floor((map_y-map_y_start)*p); 
		nodes_in_z = floor((map_z-map_z_start)*p);
	
	}else if(floor((map_x-map_x_start)*p) * round((map_y-map_y_start)*p) * floor((map_z-map_z_start)*p) <= (init_number_of_samples-2)){
		nodes_in_x = floor((map_x-map_x_start)*p);
		nodes_in_y = round((map_y-map_y_start)*p); 
		nodes_in_z = floor((map_z-map_z_start)*p);
	}else if(floor((map_x-map_x_start)*p) * floor((map_y-map_y_start)*p) * round((map_z-map_z_start)*p) <= (init_number_of_samples-2)){
		nodes_in_x = floor((map_x-map_x_start)*p);
		nodes_in_y = floor((map_y-map_y_start)*p); 
		nodes_in_z = round((map_z-map_z_start)*p);
	
	}else{
		nodes_in_x=floor((map_x-map_x_start)*p);
		nodes_in_y=floor((map_y-map_y_start)*p);
		nodes_in_z=floor((map_z-map_z_start)*p);
	}

	sample_grid_size.push_back(nodes_in_x); sample_grid_size.push_back(nodes_in_y); sample_grid_size.push_back(nodes_in_z);
	return sample_grid_size;
}



//************************************************************************************************



//*****************************************************************************************************



//************************************************************************************************


void generate_random_samples_previous(vector<local_planner_node*>& sample_local_planner_node,vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, int j){
	position k; 
	for (int i=j;i<init_number_of_samples-2;i++){
		local_planner_node* s=new local_planner_node(); 
		k.x = (float)rand()/(float)(RAND_MAX/map_x);
		k.y = (float)rand()/(float)(RAND_MAX/map_y);
		k.z = (float)rand()/(float)(RAND_MAX/map_z);
		while (not_in_map_limit(k) | collides_with_obstacle(k,obstacles)){
			//cout<<"either exceeded limit. or collides  x= "<<k.x<<",   y= "<<k.y<<",   z= "<<k.z<<endl;    
			k.x = (float)rand()/(float)(RAND_MAX/map_x);
			k.y = (float)rand()/(float)(RAND_MAX/map_y);
			k.z = (float)rand()/(float)(RAND_MAX/map_z);
		}

		//cout<<k.x<<"     "<<k.y<<"     "<<k.z<<endl;
		s->p=k;
		sample_local_planner_node.push_back(s); 
		 
	}
	sample_local_planner_node.push_back(start); sample_local_planner_node.push_back(destination);



}

//************************************************************************************************


vector<local_planner_node*> generate_milestone_sample_local_planner_node_previous(vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, vector <int> sample_grid_size){


	
	vector<local_planner_node*> sample_local_planner_node;

	
	generate_random_samples_previous(sample_local_planner_node, obstacles, start, destination, 0);

	/*cout<<sample_local_planner_node.size()<<endl;
	for(int i=0; i<sample_local_planner_node.size();i++){
		cout<<sample_local_planner_node[i]->p.x<<"  "<<sample_local_planner_node[i]->p.y<<"  "<< sample_local_planner_node[i]->p.z<<endl;
	}*/


	return sample_local_planner_node;

}



//********************************************************************************************************************************

void calculate_the_cordinate_limit_previous(float cord_limit[]){
	cord_limit[0]=(float)map_x_start + padding_for_grid_sample;
	cord_limit[1]=(float)map_x - padding_for_grid_sample;

	cord_limit[2]=(float)map_y_start + padding_for_grid_sample;
	cord_limit[3]=(float)map_y - padding_for_grid_sample;
	
	cord_limit[4]=(float)map_z_start + padding_for_grid_sample;
	cord_limit[5]=(float)map_z - padding_for_grid_sample;
}


void find_the_gap_previous(float cord_limit[], float gap[], vector<int> sample_grid_size){
	int m[3];
	//m[0]=grid_nodes_in_x; m[1]=grid_nodes_in_y; m[2]=grid_nodes_in_z;
	m[0]=sample_grid_size[0]; m[1]=sample_grid_size[1]; m[2]=sample_grid_size[2];

	if(m[0]*m[1]*m[2]>(init_number_of_samples-2)){
		cout<<" error in grid_nodes distribution. Please update grid_nodes_in_x,y,z. The multiplication of grid_nodes_in_x,y&z should not exceed init_number_of_samples-2 and each value should be at least 2"<<endl;
		exit(0);
	}

	//int m=(int)pow((double)(init_number_of_samples-2), double(1/3.0));
	//cout<<"m= "<<m<<endl;
	if(m[0]<2 || m[1]<2 || m[2]<2 ){
		cout<<"Too less samples for grid distribution"<<endl;
		exit(0);
	}

	gap[0]=(cord_limit[1] - cord_limit[0])/(m[0]-1);
	gap[1]=(cord_limit[3] - cord_limit[2])/(m[1]-1);
	gap[2]=(cord_limit[5] - cord_limit[4])/(m[2]-1);

	//cout<<"gap_x= "<<gap[0]<<"    gap_y= "<<gap[1]<<"     gap_z= "<<gap[2]<<endl;
	


}
//*****************************************************************************************************************

void generate_grid_samples_previous(float cord_limit[], float gap[], vector<Sphere> obstacles, vector<local_planner_node*>& sample_local_planner_node){

	position k;
	int d=1;
	float z_curr=cord_limit[4]; float y_curr; float x_curr;
	while(z_curr<map_z){
		if(d%2==0){
			y_curr=cord_limit[2]+(gap[1]/2);
		}else{
			y_curr=cord_limit[2];
		}
		
		while(y_curr<map_y){
			if(d%2==0){
				x_curr=cord_limit[0]+(gap[0]/2);
			}else{
				x_curr=cord_limit[0];
			}
			
			
			while(x_curr<map_x){
				k.x=x_curr;k.y=y_curr;k.z=z_curr;
				if( collides_with_obstacle(k,obstacles)==false){
					local_planner_node* s=new local_planner_node();
					s->p=k;
					sample_local_planner_node.push_back(s); 
				}
				//cout<<x_curr<<", "<<y_curr<<", "<<z_curr<<endl;

				x_curr=x_curr+gap[0];
			}
			//cout<<endl;	
			y_curr=y_curr+gap[1];			
		}
		//cout<<endl;
		z_curr=z_curr+gap[2];
		d++;
	}

	
}




vector<local_planner_node*> generate_grid_milestone_sample_local_planner_node_previous(vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, vector<int> sample_grid_size){

	
	vector<local_planner_node*> sample_local_planner_node;

	float cord_limit[6];
	calculate_the_cordinate_limit_previous(cord_limit);

	
	float gap[3];
	find_the_gap_previous(cord_limit, gap, sample_grid_size);


	generate_grid_samples_previous(cord_limit, gap, obstacles, sample_local_planner_node);

	generate_random_samples_previous(sample_local_planner_node, obstacles, start, destination, sample_local_planner_node.size());

	/*cout<<sample_local_planner_node.size()<<endl;
	for(int i=0; i<sample_local_planner_node.size();i++){
		cout<<sample_local_planner_node[i]->p.x<<"  "<<sample_local_planner_node[i]->p.y<<"  "<< sample_local_planner_node[i]->p.z<<endl;
	}*/
	

	return sample_local_planner_node;

}



//***************************************************************************************************************

void generate_exact_grid_samples_previous(float cord_limit[], float gap[], vector<Sphere> obstacles, vector<local_planner_node*>& sample_local_planner_node){

	position k;
	float z_curr=cord_limit[4]; float y_curr; float x_curr;
	while(z_curr<map_z){
		y_curr=cord_limit[2];
		
		
		while(y_curr<map_y){
			x_curr=cord_limit[0];
			
			
			while(x_curr<map_x){
				k.x=x_curr;k.y=y_curr;k.z=z_curr;
				if( collides_with_obstacle(k,obstacles)==false){
					local_planner_node* s=new local_planner_node();
					s->p=k;
					sample_local_planner_node.push_back(s); 
				}
				//cout<<x_curr<<", "<<y_curr<<", "<<z_curr<<endl;

				x_curr=x_curr+gap[0];
			}
			//cout<<endl;	
			y_curr=y_curr+gap[1];			
		}
		//cout<<endl;
		z_curr=z_curr+gap[2];
	}

	
}




vector<local_planner_node*> generate_exact_grid_milestone_sample_local_planner_node_previous(vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, vector<int> sample_grid_size){

	
	vector<local_planner_node*> sample_local_planner_node;

	float cord_limit[6];
	calculate_the_cordinate_limit_previous(cord_limit);

	
	float gap[3];
	find_the_gap_previous(cord_limit, gap, sample_grid_size);


	generate_exact_grid_samples_previous(cord_limit, gap, obstacles, sample_local_planner_node);

	generate_random_samples_previous(sample_local_planner_node, obstacles, start, destination, sample_local_planner_node.size());

	/*cout<<sample_local_planner_node.size()<<endl;
	for(int i=0; i<sample_local_planner_node.size();i++){
		cout<<sample_local_planner_node[i]->p.x<<"  "<<sample_local_planner_node[i]->p.y<<"  "<< sample_local_planner_node[i]->p.z<<endl;
	}*/
	

	return sample_local_planner_node;

}







#endif
