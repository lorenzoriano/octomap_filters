#include <ros/ros.h>
#include <octomap_filter.h>


int main(int argc, char** argv) {

	ros::init(argc, argv, "test_octomap_filter");

	if (argc != 2) {
		ROS_ERROR("Usage: %s [mapfilename.bt]", argv[0]);
		return 1;
	}

	octomap::OcTree tree(0.02);
	if (not tree.readBinary(argv[1])) {
		ROS_ERROR("Error while loading octree from %s", argv[1]);
		return 1;
	}
	OctomapFilter filter(&tree);

	ROS_INFO("Tree memory: %d", tree.memoryUsage());

	octomap::point3d p(4.5, 0, 0.9);
	ROS_INFO_STREAM("Searching for point "<<p);

	octomap::OcTree::NodeType* n = tree.search(p);
	if (n != NULL)
		ROS_INFO("Node found! ");
	else {
		ROS_INFO("Node not found, getting the closest");
		octomap::point3d key = filter.find_closest_node(p);
		ROS_INFO_STREAM("Closest node has coords: "<<key);

		n = tree.search(key);
	}

	ROS_INFO("Node has value: %f", n->getValue());
	ROS_INFO("Node has occupancy: %f", n->getOccupancy());

	for (unsigned int i=0; i<8; i++) {
		if (n->childExists(i)) {
			ROS_INFO("Child %d exists!", i);
		}
	}


    return 0;

}
