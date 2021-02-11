#ifndef milestones
#define milestones

#include <stdint.h>
#include<vector>
#include<fstream>
#include <cstdlib>

#include "predefined_variables.h"


#define padding_for_grid_sample UAV_radius
using namespace std;

//***********************************************************************************************

void read_the_obstacles(vector<Sphere>& obstacles){
		
	// read the obstacles from the file
		
	ifstream infile;
	infile.open(obstacle_file_name);
	
	position k; float r;

	for(int i=0;i<number_of_obstacles ;i++){
		infile>>k.x>>k.y>>k.z>>r;
		obstacles[i].p=k; obstacles[i].radius=r;
	}
	//cout<<obstacles[i-1].position.x<<endl;
	infile.close();

}

//************************************************************************************************
//finds the limit of x y z where the sample nodes should be placed. its like finding a cubic area arround 
//start and destination where the sample nodes will be placed
void find_xyz_limit(position start, position end, vector<float>& xyz_limit){


	float x_start, x_end;

	float x_small, x_big;
	if(start.x < end.x){
		x_small=start.x; x_big=end.x;
	}else{
		x_small=end.x; x_big=start.x;
	}
	if((x_small-UAV_radius*spacing_for_sample)>=map_x_start){
		x_start=x_small-UAV_radius*spacing_for_sample;
	}else{
		x_start=map_x_start;
	}

	if((x_big+UAV_radius*spacing_for_sample)<=map_x){
		x_end=x_big+UAV_radius*spacing_for_sample;
	}else{
		x_end=map_x;
	}
	//cout<< "x_start = "<<x_start<<", x_end = "<<x_end<<endl;
		
	float y_start, y_end;
	float y_small, y_big;
	if(start.y < end.y){
		y_small=start.y; y_big=end.y;
	}else{
		y_small=end.y; y_big=start.y;
	}
	if((y_small-UAV_radius*spacing_for_sample)>=map_y_start){
		y_start=y_small-UAV_radius*spacing_for_sample;
	}else{
		y_start=map_y_start;
	}

	if((y_big+UAV_radius*spacing_for_sample)<=map_y){
		y_end=y_big+UAV_radius*spacing_for_sample;
	}else{
		y_end=map_y;
	}
	//cout<< "y_start = "<<y_start<<", y_end = "<<y_end<<endl;
	

	float z_start, z_end;
	float z_small, z_big;
	if(start.z < end.z){
		z_small=start.z; z_big=end.z;
	}else{
		z_small=end.z; z_big=start.z;
	}
	if((z_small-UAV_radius*spacing_for_sample)>=map_z_start){
		z_start=z_small-UAV_radius*spacing_for_sample;
	}else{
		z_start=map_z_start;
	}

	if((z_big+UAV_radius*spacing_for_sample)<=map_z){
		z_end=z_big+UAV_radius*spacing_for_sample;
	}else{
		z_end=map_z;
	}
	//cout<< "z_start = "<<z_start<<", z_end = "<<z_end<<endl;
	
	//vector<float> xyz_limit(6);

	xyz_limit[0] = x_start; xyz_limit[1] = x_end;
	xyz_limit[2] = y_start; xyz_limit[3] = y_end;
	xyz_limit[4] = z_start; xyz_limit[5] = z_end;

		

}

int find_max(int a, int b){
	if(a>b){
		return a;
	}else{
		return b;
	}
}


//within the xyz limit find the grid size for placing the sample nodes
vector<int> find_the_sample_grid_size(position start, position end, vector<float>& xyz_limit){
	vector<int> sample_grid_size;
	
	find_xyz_limit(start, end, xyz_limit);

	//cout<<"xstart="<<xyz_limit[0] <<" xend="<< xyz_limit[1]<<"   ystart="<< xyz_limit[2] <<" yend="<< xyz_limit[3]<<"   zstart="<<xyz_limit[4] <<" zend="<< xyz_limit[5]<<endl;
	float x = xyz_limit[1] - xyz_limit[0];
	float y = xyz_limit[3] - xyz_limit[2];
	float z = xyz_limit[5] - xyz_limit[4];

	
	if(x<0)x=x*(-1); if(y<0)y=y*(-1);if(z<0)z=z*(-1);
	
	//cout<<"x= "<<x<<"   y="<<y<<"  z="<<z<<endl;

	float p=(float)(init_number_of_samples-2) /(x * y * z) ;
	//cout<<"p = "<<p<<endl;
	p=pow(p,(1.0/3));

	//cout<<"p = "<<p<<endl;

	int nodes_in_x, nodes_in_y, nodes_in_z;
	

		
	vector<int> x_s; vector<int>y_s; vector<int>z_s;
	if(round(x*p)>=2)x_s.push_back(round(x*p));
	if(floor(x*p)>=2 && floor(x*p)!=round(x*p) )x_s.push_back(floor(x*p));
	

	if(round(y*p)>=2)y_s.push_back(round(y*p));
	if(floor(y*p)>=2 && floor(y*p)!=round(y*p) )y_s.push_back(floor(y*p));
	

	if(round(z*p)>=2)z_s.push_back(round(z*p));
	if(floor(z*p)>=2 && floor(z*p)!=round(z*p) )z_s.push_back(floor(z*p));
	
	int number_of_sample_nodes = init_number_of_samples-2;
	//bool found=false;


	if(x_s.size()==0 && y_s.size()==0 && z_s.size()==0){
		//invalid
	}else if (x_s.size()==0 && y_s.size()==0 && z_s.size()!=0){
		x_s.push_back(2); y_s.push_back(2);  number_of_sample_nodes= number_of_sample_nodes/4;
		p=(float)(number_of_sample_nodes) /(z) ;
		z_s.erase(z_s.begin(), z_s.begin()+z_s.size()); 
		if(round(z*p)>=2 && round(z*p)*4<=(init_number_of_samples-2) ){
			z_s.push_back(round(z*p));
		}else if(floor(z*p)>=2 && floor(z*p)*4<=(init_number_of_samples-2)  ){
			z_s.push_back(floor(z*p));
		}else{
			z_s.push_back(floor(z*p)-1);			
			
			if(z_s[0]*6<=(init_number_of_samples-2)){
				x_s[0]=3;
			} 

		}
	

	}else if (x_s.size()==0 && y_s.size()!=0 && z_s.size()==0){
		x_s.push_back(2); z_s.push_back(2);  number_of_sample_nodes= number_of_sample_nodes/4;
		p=(float)(number_of_sample_nodes) /(y) ;
		y_s.erase(y_s.begin(), y_s.begin()+y_s.size()); 
		if(round(y*p)>=2 && round(y*p)*4<=(init_number_of_samples-2) ){
			y_s.push_back(round(y*p));
		}else if(floor(y*p)>=2 &&  floor(y*p)*4 <=(init_number_of_samples-2) ){
			y_s.push_back(floor(y*p));
		}else{
			y_s.push_back(floor(y*p)-1);
			if(y_s[0]*6<=(init_number_of_samples-2)){
				z_s[0]=3;
			} 


		}

	}else if (x_s.size()!=0 && y_s.size()==0 && z_s.size()==0){

		y_s.push_back(2); z_s.push_back(2);  number_of_sample_nodes= number_of_sample_nodes/4;
		p=(float)(number_of_sample_nodes) /(x) ;
		x_s.erase(x_s.begin(), x_s.begin()+x_s.size()); 
		if(round(x*p)>=2 && round(x*p)*4<=(init_number_of_samples-2) ){
			x_s.push_back(round(x*p));
		}else if(floor(x*p)>=2 && floor(x*p)*4 <=(init_number_of_samples-2) ){
			x_s.push_back(floor(x*p));
		}else{
			x_s.push_back(floor(x*p)-1);
			if(x_s[0]*6<=(init_number_of_samples-2)){
				y_s[0]=3;
			} 

		}

	
	}else if (x_s.size()==0 && y_s.size()!=0 && z_s.size()!=0){
		x_s.push_back(2);  number_of_sample_nodes= number_of_sample_nodes/2;
		p=(float)(number_of_sample_nodes) /(y * z) ;
		p=pow(p,(1.0/2));
		y_s.erase(y_s.begin(), y_s.begin()+y_s.size()); 
		z_s.erase(z_s.begin(), z_s.begin()+z_s.size());
		if(round(y*p)>=2)y_s.push_back(round(y*p));
		if(floor(y*p)>=2 && floor(y*p)!=round(y*p) )y_s.push_back(floor(y*p));
	
		if(round(z*p)>=2)z_s.push_back(round(z*p));
		if(floor(z*p)>=2 && floor(z*p)!=round(z*p) )z_s.push_back(floor(z*p));

		if(y_s.size()==0){
			y_s.push_back(2);number_of_sample_nodes= number_of_sample_nodes/2;
			p=(float)(number_of_sample_nodes) /(z) ;
			z_s.erase(z_s.begin(), z_s.begin()+z_s.size()); 
			if(round(z*p)>=2  && round(z*p)*4 <=(init_number_of_samples-2)){
				z_s.push_back(round(z*p));
			}else if(floor(z*p)>=2 && floor(z*p)*4 <=(init_number_of_samples-2) ){
				z_s.push_back(floor(z*p));
			}else{
				z_s.push_back(floor(z*p)-1);
				if(z_s[0]*6<=(init_number_of_samples-2)){
					x_s[0]=3;
				} 
			}

		}else if(z_s.size()==0){
			z_s.push_back(2);number_of_sample_nodes= number_of_sample_nodes/2;
			p=(float)(number_of_sample_nodes) /(y) ;
			y_s.erase(y_s.begin(), y_s.begin()+y_s.size()); 
			if(round(y*p)>=2 && round(y*p)*4 <=(init_number_of_samples-2) ){
				y_s.push_back(round(y*p));
			}else if(floor(y*p)>=2 && floor(y*p)*4 <=(init_number_of_samples-2) ){
				y_s.push_back(floor(y*p));
			}else{
				y_s.push_back(floor(y*p)-1);
				if(y_s[0]*6<=(init_number_of_samples-2)){
					z_s[0]=3;
				} 
			}
			
		}



	}else if (x_s.size()!=0 && y_s.size()==0 && z_s.size()!=0){

		y_s.push_back(2);  number_of_sample_nodes= number_of_sample_nodes/2;
		p=(float)(number_of_sample_nodes) /(x * z) ;
		p=pow(p,(1.0/2));
		x_s.erase(x_s.begin(), x_s.begin()+x_s.size()); 
		z_s.erase(z_s.begin(), z_s.begin()+z_s.size());
		if(round(x*p)>=2)x_s.push_back(round(x*p));
		if(floor(x*p)>=2 && floor(x*p)!=round(x*p) )x_s.push_back(floor(x*p));
	
		if(round(z*p)>=2)z_s.push_back(round(z*p));
		if(floor(z*p)>=2 && floor(z*p)!=round(z*p) )z_s.push_back(floor(z*p));

		if(x_s.size()==0){
			x_s.push_back(2);number_of_sample_nodes= number_of_sample_nodes/2;
			p=(float)(number_of_sample_nodes) /(z) ;
			z_s.erase(z_s.begin(), z_s.begin()+z_s.size()); 
			if(round(z*p)>=2  && round(z*p)*4 <=(init_number_of_samples-2)){
				z_s.push_back(round(z*p));
			}else if(floor(z*p)>=2 && floor(z*p)*4 <=(init_number_of_samples-2) ){
				z_s.push_back(floor(z*p));
			}else{
				z_s.push_back(floor(z*p)-1);
				if(z_s[0]*6<=(init_number_of_samples-2)){
					x_s[0]=3;
				} 
			}

		}else if(z_s.size()==0){
			z_s.push_back(2);number_of_sample_nodes= number_of_sample_nodes/2;
			p=(float)(number_of_sample_nodes) /(x) ;
			x_s.erase(x_s.begin(), x_s.begin()+x_s.size()); 
			if(round(x*p)>=2 && round(x*p)*4 <=(init_number_of_samples-2)){
				x_s.push_back(round(x*p));
			}else if(floor(x*p)>=2 && floor(x*p)*4 <=(init_number_of_samples-2)){
				x_s.push_back(floor(x*p));
			}else{
				x_s.push_back(floor(x*p)-1);
				if(x_s[0]*6<=(init_number_of_samples-2)){
					y_s[0]=3;
				} 
			}
		}


	}else if (x_s.size()!=0 && y_s.size()!=0 && z_s.size()==0){
	

		z_s.push_back(2);  number_of_sample_nodes= number_of_sample_nodes/2;
		p=(float)(number_of_sample_nodes) /(x*y) ;
		p=pow(p,(1.0/2));
		y_s.erase(y_s.begin(), y_s.begin()+y_s.size()); 
		x_s.erase(x_s.begin(), x_s.begin()+x_s.size());
		if(round(y*p)>=2)y_s.push_back(round(y*p));
		if(floor(y*p)>=2 && floor(y*p)!=round(y*p) )y_s.push_back(floor(y*p));
	
		if(round(x*p)>=2)x_s.push_back(round(x*p));
		if(floor(x*p)>=2 && floor(x*p)!=round(x*p) )x_s.push_back(floor(x*p));

		if(x_s.size()==0){
			x_s.push_back(2);number_of_sample_nodes= number_of_sample_nodes/2;
			p=(float)(number_of_sample_nodes) /(y) ;
			y_s.erase(y_s.begin(), y_s.begin()+y_s.size()); 
			if(round(y*p)>=2 && round(y*p)*4 <=(init_number_of_samples-2) ){
				y_s.push_back(round(y*p));
			}else if(floor(y*p)>=2 && floor(y*p)*4 <=(init_number_of_samples-2) ){
				y_s.push_back(floor(y*p));
			}else{
				y_s.push_back(floor(y*p)-1);
				if(y_s[0]*6<=(init_number_of_samples-2)){
					z_s[0]=3;
				} 
			}
				
	

		}else if(y_s.size()==0){
			y_s.push_back(2);number_of_sample_nodes= number_of_sample_nodes/2;
			p=(float)(number_of_sample_nodes) /(x) ;
			x_s.erase(x_s.begin(), x_s.begin()+x_s.size()); 
			if(round(x*p)>=2 && round(x*p)*4 <=(init_number_of_samples-2)){
				x_s.push_back(round(x*p));
			}else if(floor(x*p)>=2 && floor(x*p)*4 <=(init_number_of_samples-2)){
				x_s.push_back(floor(x*p));
			}else{
				x_s.push_back(floor(x*p)-1);
				if(x_s[0]*6<=(init_number_of_samples-2)){
					y_s[0]=3;
				} 
			}
	
		}

	}

	if(x_s.size()==0 || y_s.size()==0 || z_s.size()==0) {
		cout<<"*****************************"<<endl;
		cout<<"after all scaling still problem with grid"<<endl;
		cout<<"******************"<<endl;

	}
	
	bool found=false;
	for(int i=0;i<x_s.size();i++){
		nodes_in_x=x_s[i];
		for(int j=0; j<y_s.size(); j++){
			nodes_in_y= y_s[j];
			for(int k=0; k<z_s.size(); k++){
				nodes_in_z=z_s[k];
				if((nodes_in_x*nodes_in_y*nodes_in_z) <=(init_number_of_samples-2)){
					found=true;
					break;
				}
			}
			if(found) break;
		}
		if(found) break;
	}

	if(found){
		//cout<<"found solution"<<"  x="<<nodes_in_x<<"   y="<<nodes_in_y<<"  z="<<nodes_in_z<<endl;
	}else{
		cout<<"not found solution"<<"  x="<<nodes_in_x<<"   y="<<nodes_in_y<<"  z="<<nodes_in_z<<endl;

		while(!found){
			cout<<"inside while"<<endl;
			int n_max=find_max(find_max(nodes_in_x,nodes_in_y),nodes_in_z);
			if(n_max==nodes_in_x){
				nodes_in_x=nodes_in_x-1;
			}else if(n_max==nodes_in_y){
				nodes_in_y=nodes_in_y-1;
			}else{
				nodes_in_z=nodes_in_z-1;
			}
			if(nodes_in_x*nodes_in_y*nodes_in_z <=(init_number_of_samples-2) ){
				found=true;
			}
		}
		cout<<"updated solution "<<"  x="<<nodes_in_x<<"   y="<<nodes_in_y<<"  z="<<nodes_in_z<<endl;
	} 

	
	sample_grid_size.push_back(nodes_in_x); sample_grid_size.push_back(nodes_in_y); sample_grid_size.push_back(nodes_in_z);
	return sample_grid_size;
}



//************************************************************************************************

bool not_in_map_limit(position k){
	if( ((k.x+UAV_radius)>map_x) || ((k.x-UAV_radius)<0) || ((k.y+UAV_radius)>map_y) || ((k.y-UAV_radius)<0) || ((k.z+UAV_radius)>map_z) || ((k.z-UAV_radius)<0)){
		return true;
	}
	return false; 
}

//*****************************************************************************************************

position deduct(const position& v1, const position& v2) {
    position r;
    r.x = v1.x - v2.x;
    r.y = v1.y - v2.y;
    r.z = v1.z - v2.z;
    return r;
}

double dotProduct(const position& v1, const position& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}




int distanceSquared(const position& v1, const position& v2) {
    position delta = deduct(v2, v1);
    return dotProduct(delta, delta);
}


bool doesItCollide(const position& s1, const Sphere& s2) {
    int rSquared = UAV_radius + s2.radius;
    rSquared *= rSquared;
    return distanceSquared(s1, s2.p) < rSquared;
}

bool collides_with_obstacle(position k,vector<Sphere> obstacles){

	for(int j=0;j<number_of_obstacles ;j++){
		if(doesItCollide(k, obstacles[j])){
			//cout<<"colliding obstacle: "<<obstacles[j].p.x<<"   "<<obstacles[j].p.y<<"   "<<obstacles[j].p.z<<"   "<<obstacles[j].radius<<endl;
			return true;
		}
	}
	return false;	
}



//************************************************************************************************

void generate_random_samples(vector<local_planner_node*>& sample_local_planner_node,vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, int j, vector <float> xyz_limit){
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



//************************************************************************************************

vector<local_planner_node*> generate_milestone_sample_local_planner_node(vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, vector <int> sample_grid_size, vector <float> xyz_limit){


	
	vector<local_planner_node*> sample_local_planner_node;

	
	generate_random_samples(sample_local_planner_node, obstacles, start, destination, 0, xyz_limit);

	/*cout<<sample_local_planner_node.size()<<endl;
	for(int i=0; i<sample_local_planner_node.size();i++){
		cout<<sample_local_planner_node[i]->p.x<<"  "<<sample_local_planner_node[i]->p.y<<"  "<< sample_local_planner_node[i]->p.z<<endl;
	}*/


	return sample_local_planner_node;

}



void calculate_the_cordinate_limit(vector<float>& xyz_limit){
	xyz_limit[0]=xyz_limit[0] + padding_for_grid_sample;
	xyz_limit[1]=xyz_limit[1] - padding_for_grid_sample;

	xyz_limit[2]=xyz_limit[2] + padding_for_grid_sample;
	xyz_limit[3]=xyz_limit[3] - padding_for_grid_sample;
	
	xyz_limit[4]=xyz_limit[4] + padding_for_grid_sample;
	xyz_limit[5]=xyz_limit[5] - padding_for_grid_sample;
}

void find_the_gap(vector<float> xyz_limit, float gap[], vector<int> sample_grid_size){
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

	gap[0]=(xyz_limit[1] - xyz_limit[0])/(m[0]-1);
	gap[1]=(xyz_limit[3] - xyz_limit[2])/(m[1]-1);
	gap[2]=(xyz_limit[5] - xyz_limit[4])/(m[2]-1);

	//cout<<"gap_x= "<<gap[0]<<"    gap_y= "<<gap[1]<<"     gap_z= "<<gap[2]<<endl;
	


}


void generate_grid_samples(vector<float> xyz_limit, float gap[], vector<Sphere> obstacles, vector<local_planner_node*>& sample_local_planner_node){

	position k;
	int d=1;
	float z_curr=xyz_limit[4]; float y_curr; float x_curr;
	while(z_curr<xyz_limit[5]){
		if(d%2==0){
			y_curr=xyz_limit[2]+(gap[1]/2);
		}else{
			y_curr=xyz_limit[2];
		}
		
		while(y_curr<xyz_limit[3]){
			if(d%2==0){
				x_curr=xyz_limit[0]+(gap[0]/2);
			}else{
				x_curr=xyz_limit[0];
			}
			
			
			while(x_curr<xyz_limit[1]){
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


vector<local_planner_node*> generate_grid_milestone_sample_local_planner_node(vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, vector<int> sample_grid_size, vector <float> xyz_limit){

	
	vector<local_planner_node*> sample_local_planner_node;


	calculate_the_cordinate_limit(xyz_limit);

	
	float gap[3];
	find_the_gap(xyz_limit, gap, sample_grid_size);


	generate_grid_samples(xyz_limit, gap, obstacles, sample_local_planner_node);

	generate_random_samples(sample_local_planner_node, obstacles, start, destination, sample_local_planner_node.size(), xyz_limit);

	/*cout<<sample_local_planner_node.size()<<endl;
	for(int i=0; i<sample_local_planner_node.size();i++){
		cout<<sample_local_planner_node[i]->p.x<<"  "<<sample_local_planner_node[i]->p.y<<"  "<< sample_local_planner_node[i]->p.z<<endl;
	}*/
	

	return sample_local_planner_node;

}





//***************************************************************************************************************
void generate_exact_grid_samples(vector<float> xyz_limit, float gap[], vector<Sphere> obstacles, vector<local_planner_node*>& sample_local_planner_node){

	position k;
	float z_curr=xyz_limit[4]; float y_curr; float x_curr;
	while(z_curr<xyz_limit[5]){
		y_curr=xyz_limit[2];
		
		
		while(y_curr<xyz_limit[3]){
			x_curr=xyz_limit[0];
			
			
			while(x_curr<xyz_limit[1]){
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

vector<local_planner_node*> generate_exact_grid_milestone_sample_local_planner_node(vector<Sphere> obstacles, local_planner_node*  start, local_planner_node* destination, vector<int> sample_grid_size, vector <float> xyz_limit){

	
	vector<local_planner_node*> sample_local_planner_node;

	
	calculate_the_cordinate_limit(xyz_limit);

	
	float gap[3];
	find_the_gap(xyz_limit, gap, sample_grid_size);


	generate_exact_grid_samples(xyz_limit, gap, obstacles, sample_local_planner_node);

	generate_random_samples(sample_local_planner_node, obstacles, start, destination, sample_local_planner_node.size(), xyz_limit);

	/*cout<<sample_local_planner_node.size()<<endl;
	for(int i=0; i<sample_local_planner_node.size();i++){
		cout<<sample_local_planner_node[i]->p.x<<"  "<<sample_local_planner_node[i]->p.y<<"  "<< sample_local_planner_node[i]->p.z<<endl;
	}*/
	

	return sample_local_planner_node;

}






//***************************************************************************************************************

bool boundary_condition_satisfied(local_planner_node* start, local_planner_node* destination, vector<Sphere> obstacles){
	
	position i=start->p; position j=destination->p;
	if( collides_with_obstacle(i,obstacles) | collides_with_obstacle(j,obstacles)){
		return false;
	}
	return true; 
	
}



#endif
