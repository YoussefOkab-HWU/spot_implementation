#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetLinkProperties, GetJointProperties
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import tf2_ros
import tf.transformations

rospy.init_node('odom_pub')
odom_pub = rospy.Publisher('/my_odom', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

tf_broadcaster = tf2_ros.TransformBroadcaster()
#joint_state_subscriber = rospy.Subscriber('/spot1/joint_states', JointState, joint_state_callback)

odom = Odometry()
header = Header()
header.frame_id = 'my_odom'

model = GetModelStateRequest()
model.model_name = 'spot1'

previous_transforms = {}
def joint_state_callback(msg):
    # Extract the joint names, positions, velocities, and efforts from the received message
    joint_names = msg.name
    joint_positions = msg.position
    joint_velocities = msg.velocity
    joint_efforts = msg.effort

    # Process each joint individually
    for name, position in zip(joint_names, joint_positions):
        # Check if the joint name matches any of the specified joints
        if name in ['front_left_hip_x', 'front_left_hip_y', 'front_left_knee',
                    'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
                    'rear_left_hip_x', 'rear_left_hip_y', 'rear_left_knee',
                    'rear_right_hip_x', 'rear_right_hip_y', 'rear_right_knee']:
            # Print the joint name and position
            #print(f"Joint '{name}' position: {position}")
            pass
joint_state_subscriber = rospy.Subscriber('/spot1/joint_states', JointState, joint_state_callback)

while not rospy.is_shutdown():
    result = get_model_srv(model)
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist
    header.stamp = rospy.Time.now()
    odom.header = header

    # Publish odometry

    # Broadcast the transform for each link
    link_names = ['marker','visuilization_marker','world','lase_frame','my_odom','base_link','front_rail','rear_rail','body', 'front_right_hip', 'front_right_upper_leg',
                  'front_right_lower_leg', 'front_left_hip', 'front_left_upper_leg', 'front_left_lower_leg',
                  'rear_right_hip', 'rear_right_upper_leg', 'rear_right_lower_leg', 'rear_left_hip',
                  'rear_left_upper_leg', 'rear_left_lower_leg']
    # laser_frame link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/body"
    transform_stamped.child_frame_id = "/laser_frame"
    transform_stamped.transform.translation.x = 0.05
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = 0.1
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
     # laser_frame link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/laser_frame"
    transform_stamped.child_frame_id = "/world"
    transform_stamped.transform.translation.x = 0.05
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = 0.1
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    # marker link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/body"
    transform_stamped.child_frame_id = "/marker"
    transform_stamped.transform.translation.x = 0.05
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = 0.1
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    transform_stamped = TransformStamped()
    #visuilisation_marker
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/marker"
    transform_stamped.child_frame_id = "/visuilization_marker"
    transform_stamped.transform.translation.x = 0.05
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = 0.1
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    
    # Body link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/my_odom"
    transform_stamped.child_frame_id = "/base_link"
    transform_stamped.transform.translation.x =  result.pose.position.x
    transform_stamped.transform.translation.y =  result.pose.position.y
    transform_stamped.transform.translation.z =  result.pose.position.z
    transform_stamped.transform.rotation.x = result.pose.orientation.x
    transform_stamped.transform.rotation.y = result.pose.orientation.y
    transform_stamped.transform.rotation.z = result.pose.orientation.z
    transform_stamped.transform.rotation.w = result.pose.orientation.w
    tf_broadcaster.sendTransform(transform_stamped)
        # base_link link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/base_link"
    transform_stamped.child_frame_id = "/body"
    transform_stamped.transform.translation.x =  result.pose.position.x
    transform_stamped.transform.translation.y =  result.pose.position.y
    transform_stamped.transform.translation.z =  result.pose.position.z
    transform_stamped.transform.rotation.x = result.pose.orientation.x
    transform_stamped.transform.rotation.y = result.pose.orientation.y
    transform_stamped.transform.rotation.z = result.pose.orientation.z
    transform_stamped.transform.rotation.w =  result.pose.orientation.w
    tf_broadcaster.sendTransform(transform_stamped)
        # front_rail link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/body"
    transform_stamped.child_frame_id = "/front_rail"
    transform_stamped.transform.translation.x = 0.223
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = 0.04505
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
        # rear_rail link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/body"
    transform_stamped.child_frame_id = "/rear_rail"
    transform_stamped.transform.translation.x = -0.223
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = 0.04505
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    # Front left hip link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/body"
    transform_stamped.child_frame_id = "/front_left_hip"
    transform_stamped.transform.translation.x = 0.29785
    transform_stamped.transform.translation.y = 0.05500
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    #Front left upper leg link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/front_left_hip"
    transform_stamped.child_frame_id = "/front_left_upper_leg"
    transform_stamped.transform.translation.x = 0
    transform_stamped.transform.translation.y = 0.110945
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    #Front left lower leg link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/front_left_upper_leg"
    transform_stamped.child_frame_id = "/front_left_lower_leg"
    transform_stamped.transform.translation.x = 0.025
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = -0.3205
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
     # Front right hip link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/body"
    transform_stamped.child_frame_id = "/front_right_hip"
    transform_stamped.transform.translation.x = 0.29785
    transform_stamped.transform.translation.y = -0.055
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    #Front right upper leg link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/front_right_hip"
    transform_stamped.child_frame_id = "/front_right_upper_leg"
    transform_stamped.transform.translation.x = 0
    transform_stamped.transform.translation.y = -0.110945
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    #Front right lower leg link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/front_right_upper_leg"
    transform_stamped.child_frame_id = "/front_right_lower_leg"
    transform_stamped.transform.translation.x = 0.025
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = -0.3205
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)

    # Rear left hip link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/body"
    transform_stamped.child_frame_id = "/rear_left_hip"
    transform_stamped.transform.translation.x = -0.29785
    transform_stamped.transform.translation.y = 0.05500
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    #Rear left upper leg link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/rear_left_hip"
    transform_stamped.child_frame_id = "/rear_left_upper_leg"
    transform_stamped.transform.translation.x = 0
    transform_stamped.transform.translation.y = 0.110945
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    #rear left lower leg link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/rear_left_upper_leg"
    transform_stamped.child_frame_id = "/rear_left_lower_leg"
    transform_stamped.transform.translation.x = 0.025
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = -0.3205
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    # rear right hip link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/body"
    transform_stamped.child_frame_id = "/rear_right_hip"
    transform_stamped.transform.translation.x = -0.29785
    transform_stamped.transform.translation.y = -0.055
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    #Front right upper leg link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/rear_right_hip"
    transform_stamped.child_frame_id = "/rear_right_upper_leg"
    transform_stamped.transform.translation.x = 0
    transform_stamped.transform.translation.y = -0.110945
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)
    #Front right lower leg link
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "/rear_right_upper_leg"
    transform_stamped.child_frame_id = "/rear_right_lower_leg"
    transform_stamped.transform.translation.x = 0.025
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = -0.3205
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform_stamped)

    odom_pub.publish(odom)

    rospy.sleep(0.1)

rospy.spin()