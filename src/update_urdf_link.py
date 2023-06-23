import roslaunch
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import xml.etree.ElementTree as ET
import subprocess

def update_link_position(xacro_file, link_name, new_position, new_rotation):
    # Initialize ROS node
    rospy.init_node('link_position_updater')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_desc = roslaunch.parent.ROSLaunchParent(uuid, ["/opt/ros/melodic/share/rviz/launch/rviz.launch"])


    # # Initialize TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    tree = ET.parse(xacro_file)
    root = tree.getroot()

    # Find the link with the given name
    link_element = root.find(".//link[@name='{}']".format(link_name))
    if link_element is None:
        print("Link '{}' not found in the URDF file.".format(link_name))
        return

    # Find the visual element within the link
    visual_element = link_element.find("visual")
    if visual_element is None:
        print("No visual element found for link '{}'.".format(link_name))
        return

    # Find the origin element within the visual element
    origin_element = visual_element.find("origin")
    if origin_element is None:
        print("No origin element found for visual of link '{}'.".format(link_name))
        return

    # Update the position attributes of the origin element
    origin_element.set("xyz", " ".join(str(coord) for coord in new_position))

    # Update the position attributes of the origin element
    origin_element.set("rpy", " ".join(str(coord) for coord in new_rotation))


    # Save the modified URDF file
    tree.write(xacro_file)
   
    # Start the RViz launch
    launch_desc.start()
    # Wait for the RViz node to initialize
    rospy.sleep(2.0)
    # Shutdown the RViz launch
    launch_desc.shutdown()


    # # Create a new transform
    # transform = TransformStamped()
    # transform.header.frame_id = 'table_link'  # Set the reference frame
    # transform.child_frame_id = link_name    # Set the link frame

    # # Update the translation
    # transform.transform.translation.x = new_position[0]
    # transform.transform.translation.y = new_position[1]
    # transform.transform.translation.z = new_position[2]

    # transform.transform.rotation.x = new_rotation[0]
    # transform.transform.rotation.y = new_rotation[1]
    # transform.transform.rotation.z = new_rotation[2]
    # transform.transform.rotation.w = new_rotation[3]

    # rate = rospy.Rate(10)  # Publish rate (adjust as needed)

    # while not rospy.is_shutdown():
    #     # Update the timestamp
    #     transform.header.stamp = rospy.Time.now()

    #     # Publish the transform
    #     tf_broadcaster.sendTransform(transform)

    #     rate.sleep()

# Example usage
link_name = "target_cup"
xacro_file_path = "/home/andre/catkin_ws/src/ros_kortex/kortex_description/arms/gen3_lite/6dof/urdf/gen3_lite_macro_modified.xacro"
# xacro_file_path = "/home/andre/catkin_ws/src/ros_kortex/kortex_description/robots/kortex_robot_modified.xacro"
new_position = [0.0, 0.0, 0.0625]
new_rotation = [0, 0.7068252, 0, 0.7073883 ]
update_link_position(xacro_file_path,link_name, new_position, new_rotation)