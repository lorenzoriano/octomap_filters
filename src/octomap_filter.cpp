#include <octomap_filter.h>
#include <ros/ros.h>
#include <arm_navigation_msgs/CollisionMap.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/GetOctomapResponse.h>
#include <octomap_msgs/GetOctomap.h>

#include <octomap_filters/FilterDefine.h>
#include <octomap_filters/QueryFilter.h>
#include <string>

OctomapFilter::OctomapFilter() {
	tree_ = new octomap::OcTree(0.02);
    octomap_frame_id_ = "/odom_combined";
	std::string octomap_service_name = "/octomap_binary";
	get_octomap_ =  nh_.serviceClient<octomap_msgs::GetOctomap> (octomap_service_name);
	ROS_INFO("Waiting for service %s", octomap_service_name.c_str());

	get_octomap_.waitForExistence();
	ROS_INFO("connected to %s", octomap_service_name.c_str());

	cmap_publisher_ = nh_.advertise<arm_navigation_msgs::CollisionMap>("filtered_collision_map_out", 1, true);
	caller_timer_ = nh_.createTimer(ros::Duration(0.1), &OctomapFilter::ask_octomap, this);

	filter_srv_ = nh_.advertiseService("create_filter", &OctomapFilter::filter_cb, this);
	get_filter_info_srv = nh_.advertiseService("get_filter_info", &OctomapFilter::get_filter_info, this);

}

OctomapFilter::OctomapFilter(octomap::OcTree *tree) {

	tree_ = NULL;
	setTree(tree);
}

OctomapFilter::~OctomapFilter() {
	delete tree_;
}

const octomap::point3d OctomapFilter::find_closest_node(octomap::point3d pos) {
	octomap::point3d res = tree_->begin().getCoordinate();
	double dist = 10e9;

	for (octomap::OcTree::iterator i = tree_->begin(); i != tree_->end(); i++) {
		double newdist = pos.distance(i.getCoordinate());
		if (newdist < dist) {
			dist = newdist;
			res = i.getCoordinate();
		}
	}

	return res;
}

void OctomapFilter::publishCollisionMap(const std::vector<geometry_msgs::Point>& pointlist,
		const std_msgs::Header& header, ros::Publisher& pub) {
	if(pointlist.size() <= 1)
		return;

	arm_navigation_msgs::CollisionMap cmap;
	cmap.header = header;

	arm_navigation_msgs::OrientedBoundingBox box;
	box.extents.x = box.extents.y = box.extents.z = tree_->getResolution();
	box.axis.x = box.axis.y = 0.0; box.axis.z = 1.0;
	box.angle = 0.0;
	cmap.boxes.reserve(pointlist.size());

	for (std::vector<geometry_msgs::Point>::const_iterator it = pointlist.begin(); it != pointlist.end(); ++it) {
		box.center.x = it->x;
		box.center.y = it->y;
		box.center.z = it->z;
		cmap.boxes.push_back(box);
	}
	pub.publish(cmap);
}

void OctomapFilter::updateNodesInBBX(const octomap::point3d& min,
		const octomap::point3d& max, bool occupied) {

	float logodds = tree_->getClampingThresMaxLog() - tree_->getClampingThresMinLog();
	if (!occupied)
		logodds *= -1;

	for(octomap::OcTree::leaf_bbx_iterator it = tree_->begin_leafs_bbx(min,max),
			end=tree_->end_leafs_bbx(); it!= end; ++it){
		tree_->updateNode(it.getKey(), logodds);
	}

}

void OctomapFilter::getOccupiedPoints(std::vector<geometry_msgs::Point>& pointlist) const {
	{
		pointlist.reserve(tree_->size() / 2.0);
		for (octomap::OcTree::iterator it = tree_->begin(),
				end = tree_->end(); it != end; ++it){
			if (tree_->isNodeOccupied(*it)){
				geometry_msgs::Point p;
				p.x = it.getX();

				p.y = it.getY();
				p.z = it.getZ();
				pointlist.push_back(p);
			}
		}
	}
}

void OctomapFilter::ask_octomap(const ros::TimerEvent& e) {
	octomap_msgs::GetOctomapRequest req;
	octomap_msgs::GetOctomapResponse res;

	if (not get_octomap_.call(req, res) ){
		ROS_ERROR("Error while getting an octomap");
		return;

	}

	octomap_frame_id_ = res.map.header.frame_id;
	octomap::octomapMsgToMap(res.map, *tree_);
	apply_filters();

	//now publishing
	std::vector<geometry_msgs::Point> pointlist;
	getOccupiedPoints(pointlist);
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = octomap_frame_id_;

	publishCollisionMap(pointlist, header, cmap_publisher_);

}


void OctomapFilter::apply_filters() {
	for (std::map<std::string, _InternalFilter>::iterator i = filters_.begin();
			i != filters_.end(); i++) {
		if (i->second.enabled) {
			octomap_filters::FilterDefine::Request& f = i->second.req;

			octomap::point3d min;
            min.x() = f.min.point.x;
            min.y() = f.min.point.y;
            min.z() = f.min.point.z;
			octomap::point3d max;
            max.x() = f.max.point.x;
            max.y() = f.max.point.y;
            max.z() = f.max.point.z;
			updateNodesInBBX(min, max, false);
		}
	}
}

bool OctomapFilter::filter_cb(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {

    octomap_filters::FilterDefine::Request newreq;
    listener_.waitForTransform(octomap_frame_id_, request.min.header.frame_id, ros::Time(0),
                               ros::Duration(10));
    listener_.transformPoint(octomap_frame_id_, request.min, newreq.min);
    listener_.waitForTransform(octomap_frame_id_, request.max.header.frame_id, ros::Time(0),
                               ros::Duration(10));
    listener_.transformPoint(octomap_frame_id_, request.max, newreq.max);

	switch (request.operation) {
	case octomap_filters::FilterDefine::Request::CREATE:
        return new_filter(newreq, response);
	case octomap_filters::FilterDefine::Request::DISABLE:
        return disable_filter(newreq, response);
	case octomap_filters::FilterDefine::Request::ENABLE:
        return enable_filter(newreq, response);
	case octomap_filters::FilterDefine::Request::DELETE:
        return delete_filter(newreq, response);
	default:
		ROS_ERROR("Unknown request!");
		return false;
	}

}

bool OctomapFilter::new_filter(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {

	ROS_INFO("Creating new filter, name %s", request.name.c_str());
	filters_[request.name] = _InternalFilter(request, true);
	return true;
}

bool OctomapFilter::enable_filter(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {

    ROS_INFO("Enabling filter, name %s", request.name.c_str());
	std::map<std::string, _InternalFilter>::iterator i;
	i = filters_.find(request.name);
	if (i == filters_.end()) {
		ROS_ERROR("No filter named %s", request.name.c_str());
		return false;
	}

	i->second.enabled = true;
	return true;
}

bool OctomapFilter::disable_filter(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {
	std::map<std::string, _InternalFilter>::iterator i;

    ROS_INFO("Disabling filter, name %s", request.name.c_str());
	i = filters_.find(request.name);
	if (i == filters_.end()) {
		ROS_ERROR("No filter named %s", request.name.c_str());
		return false;
	}

	i->second.enabled = false;
	return true;
}

bool OctomapFilter::delete_filter(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {

    ROS_INFO("Deleting filter, name %s", request.name.c_str());
	std::map<std::string, _InternalFilter>::iterator i;
	i = filters_.find(request.name);
	if (i == filters_.end()) {
		ROS_ERROR("No filter named %s", request.name.c_str());
		return false;
	}

	filters_.erase(i);
	return true;
}

bool OctomapFilter::get_filter_info(
		octomap_filters::QueryFilter::Request& request,
		octomap_filters::QueryFilter::Response& response) {

	std::map<std::string, _InternalFilter>::iterator i;
	i = filters_.find(request.name);
	if (i == filters_.end()) {
		ROS_ERROR("No filter named %s", request.name.c_str());
		return false;
	}

	response.max = i->second.req.max;
	response.min = i->second.req.min;
	return true;
}




