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

    if len(sys.argv) == 9: #test_filters action name min_x, min_y, min_z, max_x, max_y, max_z
        req.min.x = float(sys.argv[3])
        req.min.y = float(sys.argv[4])
        req.min.z = float(sys.argv[5])
        req.max.x = float(sys.argv[6])
        req.max.y = float(sys.argv[7])
        req.max.z = float(sys.argv[8])
    else:
        req.min.x = 0
        req.min.y = 0
        req.min.z = 0
        req.max.x = 10
        req.max.y = 10
        req.max.z = 10
    
    rospy.loginfo("Sending request:\n%s",req) 
    create_filter(req)
    rospy.loginfo("Done")
    