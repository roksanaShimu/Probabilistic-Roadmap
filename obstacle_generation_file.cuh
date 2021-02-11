#ifndef __OBSTACLE_GENERATION_FILE_CUH
#define __OBSTACLE_GENERATION_FILE_CUH


#include<iostream>


#include "predefined_variables.h"
#include "data_structures.h"
#include "milestones.h"

using namespace std;

void generate_obstacle(vector<position> input_nodes){
	ofstream outfile;
	outfile.open(obstacle_file_name);
	Sphere s;
	


	for (int i=0;i<number_of_obstacles;i++){
		bool continue_looping=true;

		while(continue_looping){
			s.p.x = (float)rand()/(float)(RAND_MAX/map_x);
			s.p.y = (float)rand()/(float)(RAND_MAX/map_y);
			s.p.z = (float)rand()/(float)(RAND_MAX/map_z);
			s.radius = (float)rand()/(float)(RAND_MAX/max_radius_of_obstacle);
			bool collides=false;
	
		

			for(int j=0;j<input_nodes.size(); j++){
				
				if(doesItCollide(input_nodes[j], s) ){   //collides
					collides=true;
					break;
				}

			}
			if (!collides){
			
				//add to the obstacle;
				outfile<<s.p.x<<"     "<<s.p.y<<"     "<<s.p.z<<"     "<<s.radius<<endl;
				continue_looping=false;
			}
		}
	}



	

	outfile.close();

}


void generate_input_nodes(){

	ofstream outfile;
	outfile.open("input_nodes.txt");
	

	outfile<<number_of_waypoints<<endl;
	position p;

	for(int i=0; i<number_of_waypoints; i++){

		p.x = (float)rand()/(float)(RAND_MAX/map_x);
		p.y = (float)rand()/(float)(RAND_MAX/map_y);
		p.z = (float)rand()/(float)(RAND_MAX/map_z);
		outfile<<p.x<<" "<<p.y<<" "<<p.z<<endl;
	}


	outfile.close();

}


#endif
