#! /usr/bin/python
import roslib
roslib.load_manifest("octomap_filters")
import rospy
from octomap_filters.srv import FilterDefine, QueryFilter
from octomap_filters.srv import FilterDefineRequest, QueryFilterRequest

import sys

if __name__ == "__main__":
    rospy.init_node("test_octomap_filters")
    rospy.wait_for_service("create_filter")
    
    
    create_filter = rospy.ServiceProxy("create_filter", FilterDefine)
    
    req = FilterDefineRequest()
    if len(sys.argv) == 10:
        frame_id = sys.argv[9]
    else:
        frame_id = "/base_link"
    req.min.header.frame_id = frame_id
    req.max.header.frame_id = frame_id
    if len(sys.argv) == 1:
        req.operation = req.CREATE
    else:        
        if sys.argv[1].lower() == "create":
            req.operation = req.CREATE
        elif sys.argv[1].lower() == "delete":
            req.operation = req.DELETE
        elif sys.argv[1].lower() == "enable":
            req.operation = req.ENABLE
        elif sys.argv[1].lower() == "disable":
            req.operation = req.DISABLE
        if len(sys.argv) >= 3:
            req.name = sys.argv[2]            

    if len(sys.argv) >= 9: #test_filters action name min_x, min_y, min_z, max_x, max_y, max_z frame_id
        req.min.point.x = float(sys.argv[3])
        req.min.point.y = float(sys.argv[4])
        req.min.point.z = float(sys.argv[5])
        req.max.point.x = float(sys.argv[6])
        req.max.point.y = float(sys.argv[7])
        req.max.point.z = float(sys.argv[8])
    else:
        req.min.point.x = 0
        req.min.point.y = 0
        req.min.point.z = 0
        req.max.point.x = 10
        req.max.point.y = 10
        req.max.point.z = 10
    
    rospy.loginfo("Sending request:\n%s",req) 
    create_filter(req)
    rospy.loginfo("Done")
    
