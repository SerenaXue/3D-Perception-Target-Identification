#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Exercise-1
# Voxel Grid filter
def voxel_grid_downssampling(pcl_data, leaf_size):
    # Create a VoxelGrid filter object for our input point cloud
    vox = pcl_data.make_voxel_grid_filter()
    # Set the voxel (or leaf) size
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    return vox.filter()

# PassThrough filter
def passthrough(pcl_data, filter_axis, axis_min, axis_max):
    # Create a PassThrough filter object.
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

# RANSAC plane segmentation
def ransac_plane_segmentation(pcl_data, pcl_sac_model_plane, pcl_sac_ransac, max_distance):
    # Create the segmentation object
    seg = pcl_data.make_segmenter()
    # Set the model you wish to fit
    seg.set_model_type(pcl_sac_model_plane)
    seg.set_method_type(pcl_sac_ransac)
    seg.set_distance_threshold(max_distance)
    return seg

# Extract inliers/outliers
def extract_cloud_objects_and_cloud_table(pcl_data, ransac_segmentation):
    inliers, coefficients = ransac_segmentation.segment()
    cloud_table = pcl_data.extract(inliers, negative=False)
    cloud_objects = pcl_data.extract(inliers, negative=True)
    return cloud_table,cloud_objects

# Outlier Removal Filter
def statistical_outlier_filtering(pcl_data, mean_k, tresh):
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

# Euclidean Clustering (perform a DBSCAN cluster search)
def euclidean_clustering(white_cloud):
    tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.03)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(10000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    # now contains a list of indices for each cluster (a list of lists). In the next step, 
    # you'll create a new point cloud 
    # to visualize the clusters by assigning a color to each of them.
    
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    # Create new cloud containing all clusters, each with unique color
    # In order to visualize the results in RViz, you need to create one final point cloud, 
    # let's call it "cluster_cloud" of type PointCloud_PointXYZRGB.
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud,cluster_indices

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    cloud = statistical_outlier_filtering(cloud, 10, 0.001)
    
    # TODO: Statistical Outlier Filtering
    cloud = statistical_outlier_filtering(cloud, 10, 0.001)
    
    # TODO: Voxel Grid Downsampling
    LEAF_SIZE = 0.01
    cloud = voxel_grid_downssampling(cloud, LEAF_SIZE)

    # TODO: PassThrough Filter
    filter_axis ='z'
    axis_min = 0.50
    axis_max = 0.90
    cloud = passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'x'
    axis_min = 0.30
    axis_max = 1.0
    cloud = passthrough(cloud, filter_axis, axis_min, axis_max)

    # TODO: RANSAC Plane Segmentation
    ransac_segmentation = ransac_plane_segmentation(cloud, pcl.SACMODEL_PLANE, pcl.SAC_RANSAC, 0.01)

    # TODO: Extract inliers and outliers
    cloud_table, cloud_objects = extract_cloud_objects_and_cloud_table(cloud, ransac_segmentation)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    cluster_cloud,cluster_indices = euclidean_clustering(white_cloud)

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    # write a for loop to cycle through each of the segmented clusters
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects), detected_objects_labels))
    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    objects_map = {}  # name -> object
    for recognized_object in object_list:
        objects_map[recognized_object.label] = recognized_object

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    groups_map = {}  # object -> bin
    for object_param in object_list_param:
        groups_map[object_param['name']] = object_param['group']

    dropbox_map = {}  # group -> tuple arm, drop position (x, y, z)
    for dropbox in dropbox_param:
        dropbox_map[dropbox['group']] = (dropbox['name'], dropbox['position'])

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    dict_list = []

    # TODO: Loop through the pick list
    for object_param in object_list_param:

    	print(len(object_list_param))

        if object_param['name'] in objects_map:
            pick_object = objects_map[object_param['name']]
        else:
            print('Object ' + object_param['name'] + ' not found!')

        labels = []
        labels.append(pick_object.label)

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        # Use tuples for input
        points_arr = ros_to_pcl(pick_object.cloud).to_array()
        centroids = [] # to be list of tuples (x, y, z)
        centroid = np.mean(points_arr, axis=0)[:3]
        centroids.append(centroid)

        # TODO: Create 'place_pose' for the object
        test_scene_num = Int32()
        test_scene_num_data = 2
        test_scene_num.data = int(test_scene_num_data)

        # Initialize a variable
        object_name = String()
        object_name.data = str(pick_object.label)

        # TODO: Assign the arm to be used for pick_place
        group = groups_map[pick_object.label]
        arm = dropbox_map[group][0]

        which_arm = String()
        which_arm.data = str(arm)

        # initialize an empty pose message
        pick_pose = Pose()
        pick_pose.position.x = float(centroid[0])
        pick_pose.position.y = float(centroid[1])
        pick_pose.position.z = float(centroid[2])

        place_pose = Pose()
        place_pose.position.x = float(dropbox_map[group][1][0])
        place_pose.position.y = float(dropbox_map[group][1][1])
        place_pose.position.z = float(dropbox_map[group][1][2])

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        # Populate various ROS messages
        yaml_dict = make_yaml_dict(test_scene_num, which_arm, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, which_arm, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    yaml_filename = "output_" + str(test_scene_num.data) + ".yaml"
    send_to_yaml(yaml_filename, dict_list)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("pcl_cluster", PointCloud2, queue_size=1)

    # Create Publishers
    # TODO: here you need to create two publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

