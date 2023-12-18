#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import SegmentList, Segment
import threading
from visualization_msgs.msg import Marker, MarkerArray



class ShowMapNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ShowMapNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.sub_seg=rospy.Subscriber('~lineseglist_in',SegmentList, self.cbLineSegList, queue_size=1)
        self.pub_map = rospy.Publisher("~lane_map", MarkerArray, queue_size=1)
        self.seg_list = []
        self.marker_map = MarkerArray()
        self.x = 0.0
        self.y = 0.0
        self.num_marker = 0

    def cbLineSegList(self, lineseglist):
        # Start a daemon thread to process the line segment list
        thread = threading.Thread(target = self.storeSegments, args=(lineseglist,))
        thread.setDaemon(True)
        thread.start()
        # Returns right away

    def storeSegments(self,in_segs):
        rec_seg_list = SegmentList()
        rec_seg_list.header = in_segs.header
        for in_seg in in_segs.segments:
            new_seg = Segment()
            new_seg.points[0].x = in_seg.points[0].x
            new_seg.points[0].y = in_seg.points[0].y
            new_seg.points[0].z = 0.0
            new_seg.points[1].x = in_seg.points[1].x
            new_seg.points[1].y = in_seg.points[1].y
            new_seg.points[1].z = 0.0
            new_seg.color = in_seg.color
            rec_seg_list.segments.append(new_seg)
        self.seg_list.append(rec_seg_list)
        self.pubMap()

    def pubMap(self):
        segments = self.seg_list[-1]

        for seg in segments.segments:
            self.num_marker = self.num_marker+1
            marker = Marker()
            marker.header.frame_id = '/duck'
            marker.header.stamp=rospy.Time.now()
            marker.id = self.num_marker
            marker.type = marker.LINE_LIST
            marker.action = marker.ADD
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.a = 1.0
            if seg.color == seg.WHITE:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            elif seg.color == seg.YELLOW:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            marker.color.r = 1.0
            marker.points = seg.points
            self.marker_map.markers.append(marker)
        print(len(self.marker_map.markers))
        self.pub_map.publish(self.marker_map)



if __name__ == '__main__':
    rospy.init_node('show_map', anonymous=True)
    show_map_node = ShowMapNode(node_name='map')
    rospy.spin()
