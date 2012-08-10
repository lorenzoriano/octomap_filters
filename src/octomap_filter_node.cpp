#include <octomap_filter.h>


int main(int argc, char** argv) {

	ros::init(argc, argv, "octomap_filter_node");
	OctomapFilter filter;

	ros::AsyncSpinner spinner(1); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();

}

