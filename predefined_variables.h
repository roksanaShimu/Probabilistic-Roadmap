#ifndef predefined_variables
#define predefined_variables


//please update the values of grid numbers manually according to the number of samples and map dimension
//Please update grid_nodes_in_x,y,z. The multiplication of grid_nodes_in_x,y&z should not exceed init_number_of_samples-2 and each value should be at least 2
//#define grid_nodes_in_x 5//4
//#define grid_nodes_in_y 5//4
//#define grid_nodes_in_z 2//4



//consider the coordinates as always positive. if negative then the float z-numbers sorting for NN won't work
#define map_x_start 0
#define map_y_start 0
#define map_z_start 0
#define map_x 100
#define map_y 100
#define map_z 50
#define UAV_radius 1.0

#define obstacle_file_name "obstacle_list.txt"
#define number_of_obstacles 500
#define max_radius_of_obstacle 5

#define number_of_waypoints 10

#define spacing_for_sample 6

#define number_of_elements_for_knn 40//100//100   //get it as multiple of 4

#define input_file_name "input_nodes.txt"


#define file_location "/home/roksana/Dropbox/PRM_parallel_2017_multi_point/Result/"
//#define file_location "/home/shimu/Dropbox/PRM_parallel_2017_multi_point/Result/"

#define sample_file_location "/home/roksana/Dropbox/PRM_parallel_2017_multi_point/SampleNodes/"

#define path_debug_file_name "path_debug.txt"
#define sample_node_debug_file_name "sample_nodes.txt"


#define init_number_of_samples 64// don't go beyond 8200, it causes segmentation error // at least 10, if less than 10 it won't work
#define paths_in_each_loop 100

#define warp_size 32//4//32
#define threads_per_block 32//128//128//8//16//128////8//16;//256;   //number of threads
#define number_of_blocks init_number_of_samples//1024





#endif
