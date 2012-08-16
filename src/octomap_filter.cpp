#include <octomap_filter.h>
#include <ros/ros.h>
#include <arm_navigation_msgs/CollisionMap.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/GetOctomapResponse.h>
#include <octomap_msgs/GetOctomap.h>

#include <octomap_filters/FilterDefine.h>
#include <octomap_filters/QueryFilter.h>

#include <visualization_msgs/Marker.h>
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
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("filters_markers", 1);

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
    int id =0;
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

            publish_filter_markers(i->second.req, id);
            id++;
		}
	}
}

void OctomapFilter::publish_filter_markers(const octomap_filters::FilterDefine::Request& req, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = octomap_frame_id_;
    marker.header.stamp = ros::Time::now();

    marker.ns = req.name;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;

    float x = (req.max.point.x - req.min.point.x)/2.;
    float dim_x = (req.max.point.x - req.min.point.x);
    float y = (req.max.point.y - req.min.point.y)/2.;
    float dim_y = (req.max.point.y - req.min.point.y);
    float z = (req.max.point.z - req.min.point.z)/2.;
    float dim_z = (req.max.point.z - req.min.point.z);

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = dim_x;
    marker.scale.y = dim_y;
    marker.scale.z = dim_z;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(0.1);

    marker_pub_.publish(marker);
}

bool OctomapFilter::filter_cb(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {

	switch (request.operation) {
	case octomap_filters::FilterDefine::Request::CREATE:
        return new_filter(request, response);
    case octomap_filters::FilterDefine::Request::DISABLE:
        return disable_filter(request, response);
	case octomap_filters::FilterDefine::Request::ENABLE:
        return enable_filter(request, response);
	case octomap_filters::FilterDefine::Request::DELETE:
        return delete_filter(request, response);
	default:
		ROS_ERROR("Unknown request!");
		return false;
	}

}

bool OctomapFilter::new_filter(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {

    ROS_INFO_STREAM("Creating new filter, name "<<request.name);

    listener_.waitForTransform(octomap_frame_id_, request.min.header.frame_id, ros::Time(0),
                               ros::Duration(10));

    geometry_msgs::PointStamped newmin;
    listener_.transformPoint(octomap_frame_id_, request.min, newmin);

    geometry_msgs::PointStamped newmax;
    listener_.waitForTransform(octomap_frame_id_, request.max.header.frame_id, ros::Time(0),
                               ros::Duration(10));
    listener_.transformPoint(octomap_frame_id_, request.max, newmax); 

    octomap_filters::FilterDefine::Request newrequest = request;
    newrequest.min = newmin;
    newrequest.max = newmax;
    ROS_INFO("Newrequest min: %s (%f, %f, %f)", newrequest.min.header.frame_id.c_str(), newrequest.min.point.x, newrequest.min.point.y, newrequest.min.point.z);

    filters_[request.name] = _InternalFilter(request, true);
	return true;
}

bool OctomapFilter::enable_filter(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {

    ROS_INFO_STREAM("Enabling filter, name "<<request.name);
	std::map<std::string, _InternalFilter>::iterator i;
	i = filters_.find(request.name);
	if (i == filters_.end()) {
        ROS_ERROR_STREAM("No filter named "<<request.name);
		return false;
	}

	i->second.enabled = true;
	return true;
}

bool OctomapFilter::disable_filter(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {
	std::map<std::string, _InternalFilter>::iterator i;

    ROS_INFO_STREAM("Disabling filter, name "<< request.name);
	i = filters_.find(request.name);
	if (i == filters_.end()) {
        ROS_ERROR_STREAM("No filter named "<< request.name);
		return false;
	}

	i->second.enabled = false;
	return true;
}

bool OctomapFilter::delete_filter(octomap_filters::FilterDefine::Request& request,
		octomap_filters::FilterDefine::Response& response) {

    ROS_INFO_STREAM("Deleting filter, name "<< request.name);
	std::map<std::string, _InternalFilter>::iterator i;
	i = filters_.find(request.name);
	if (i == filters_.end()) {
        ROS_ERROR_STREAM("No filter named "<< request.name);
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
        ROS_ERROR_STREAM("No filter named "<<  request.name);
		return false;
	}

	response.max = i->second.req.max;
	response.min = i->second.req.min;
	return true;
}





