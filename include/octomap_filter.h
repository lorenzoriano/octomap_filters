#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <vector>
#include <string>

#include <octomap_filters/FilterDefine.h>
#include <octomap_filters/QueryFilter.h>
#include <tf/transform_listener.h>


struct _InternalFilter {

	_InternalFilter() {};
	_InternalFilter(const octomap_filters::FilterDefine::Request& req, bool enabled) {
		this->req = req;
		this->enabled = enabled;
	}

	octomap_filters::FilterDefine::Request req;
	bool enabled;
};

class OctomapFilter {

public:
	OctomapFilter();
	OctomapFilter(octomap::OcTree* tree);
	~OctomapFilter();


	const octomap::point3d find_closest_node(octomap::point3d pos);

	void publishCollisionMap(const std::vector<geometry_msgs::Point>& pointlist, const std_msgs::Header &header, ros::Publisher &pub);


	octomap::OcTree* getTree() const {
		return tree_;
	}
	void setTree(octomap::OcTree* tree) {
		if (tree_ != NULL)
			delete tree_;
		tree_ = new octomap::OcTree(*tree);
	}

	void getOccupiedPoints(std::vector<geometry_msgs::Point>& pointlist) const;
	void updateNodesInBBX(const octomap::point3d& min, const octomap::point3d& max, bool occupied);
	void ask_octomap(const ros::TimerEvent& e);
	void apply_filters();

protected:
    tf::TransformListener listener_;
	octomap::OcTree* tree_;
	ros::NodeHandle nh_;
	ros::ServiceClient get_octomap_;

	std::string octomap_frame_id_;
	ros::Publisher cmap_publisher_;

	ros::Timer caller_timer_;
	std::map<std::string, _InternalFilter> filters_;
	ros::ServiceServer filter_srv_;

	ros::ServiceServer get_filter_info_srv;

	bool filter_cb(octomap_filters::FilterDefine::Request& request, octomap_filters::FilterDefine::Response& response);
	bool new_filter(octomap_filters::FilterDefine::Request& request, octomap_filters::FilterDefine::Response& response);
	bool enable_filter(octomap_filters::FilterDefine::Request& request, octomap_filters::FilterDefine::Response& response);
	bool disable_filter(octomap_filters::FilterDefine::Request& request, octomap_filters::FilterDefine::Response& response);
	bool delete_filter(octomap_filters::FilterDefine::Request& request, octomap_filters::FilterDefine::Response& response);
	bool get_filter_info(octomap_filters::QueryFilter::Request& request, octomap_filters::QueryFilter::Response& response);
};
