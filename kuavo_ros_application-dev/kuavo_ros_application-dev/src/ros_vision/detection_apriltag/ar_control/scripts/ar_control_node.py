#! /usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs

class ARControlNode(object):
    def __init__(self):
        rospy.init_node('ar_control_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.publisher = rospy.Publisher('/robot_tag_info', AprilTagDetectionArray, queue_size=10)
        self.subscription = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

    def tag_callback(self, msg):
        new_msg = AprilTagDetectionArray()
        new_msg.header.stamp = rospy.Time.now()
        new_msg.header.frame_id = 'base_link'
    
        for detection in msg.detections:
            # Make sure the detection's pose has a valid frame_id
            if detection.pose.header.frame_id == '':
                detection.pose.header.frame_id = 'camera_color_optical_frame'
    
            # 转换
            if self.tf_buffer.can_transform('torso', detection.pose.header.frame_id, detection.pose.header.stamp, rospy.Duration(1.0)):
                transform = self.tf_buffer.lookup_transform('torso', detection.pose.header.frame_id, detection.pose.header.stamp)
            else:
                rospy.logwarn("Cannot transform from {} to {} at time {}".format(detection.pose.header.frame_id, 'torso', detection.pose.header.stamp))
                continue
            
            # 注意这里需要使用 detection.pose.pose.pose 来获取实际的 Pose 对象
            transformed_pose = tf2_geometry_msgs.do_transform_pose(detection.pose.pose, transform)
        
            # 转换后的目标进行赋值
            detection.pose.pose.pose.position = transformed_pose.pose.position  # position赋值
            detection.pose.pose.pose.orientation = transformed_pose.pose.orientation  # orientation 赋值
            detection.pose.header.frame_id = 'torso' # 重新该标签是基于base_link的坐标
    
            # 添加至队尾
            new_msg.detections.append(detection)
    
            # Broadcast transform to tf
            self.broadcast_transform(transformed_pose, detection.id)
    
        # 发布消息
        self.publisher.publish(new_msg)

    def broadcast_transform(self, pose, tag_id):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = 'torso'
        transform_stamped.child_frame_id = 'tag_origin_' + str(tag_id)
        transform_stamped.transform.translation.x = pose.pose.position.x
        transform_stamped.transform.translation.y = pose.pose.position.y
        transform_stamped.transform.translation.z = pose.pose.position.z
        transform_stamped.transform.rotation = pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform_stamped)

def main():
    try:
        ar_control_node = ARControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
