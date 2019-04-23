## Project3 - 3D Perception

# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

---
### Note
#### 1. Explain where the files are. 

1) Exercise 1-3 Python codes are under project folder.
2) `project_template.py` is the project final code, which is under `/RoboND-Perception-Project/pr2_robot/scripts`.
3) All the output `.yaml` files are under the same folder with `project_template.py`.
4) All the screenshot images are under `/RoboND-Perception-Project/images`.


### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

The pipeline consists of compiling these filters/algorithms:

1) pre-processing: use Point Cloud filters to remove additional data points
- Voxel Grid filter (downsampling): to derive a point cloud that has fewer points but should still do a good job of representing the input point cloud as a whole

![alt text][voxGrid]

- PassThrough filter (cropping): to crop any given 3D point cloud by specifying an axis with cut-off values along that axis

![alt text][passThrough]

2) segmentaion: isolate objects in the scene
- RANSAC plane segmentation: to identify points in dataset that belong to a particular model(a plane, a cylinder, a box, or any other common shape); used along with other techniques to obtain a subset of points, which is ExtractIndices Filter
- Extract inliers (run PCD): to extract points from a point cloud by providing a list of indices; extracted the subset of points corresponding to the table itself

![alt text][extracted_inliers]

- Extract outliers (run PCD): to retrieve the subset of points for everything that did not fit the RANSAC model, which are the objects on top of the table

![alt text][extracted_outliers]


#### 2. Complete Exercise 2 steps. Pipeline including clustering for segmentation implemented (segment the remaining points into individual objects).

1) Create publishers to publish post-processed point cloud data as ROS message; Create Subscriber to subscribe node to the `/pr2/world/points` topic to get ROS message which is raw point cloud data.

![alt text][code]

2) Use PCL's Euclidean Clustering algorithm (only supports k-d trees) to segment the remaining points into individual objects

![alt text][segmentation]

#### 3. Complete Exercise 3 Steps. Features extracted and SVM trained. Object recognition implemented.

1）Generate features

![alt text][generate features]

2) Train SVM after improve model

![alt text][SVM_result]

![alt text][f1_color]

![alt text][f2_normal]


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

1) Generate features and train an SVM classifier on labeled set of features
Inside `capture_feature.py` file, I change the `for loop` number to 30, which means I let the camera to capture 30 different view for each object. In fact, I use 30 for both world 1 and world 3, and use 50 for world 2. World 1 is the simplest environment, 30 is enough to have a workable model. For world 2, I raise the number to 50 to have a better model campared to using 30 in world 2. I also want to use 50 for world 3, but my VM cannot affordable such heavy computation so I turn it back to 30.

![alt text][world1_SVM_results]

![alt text][world2_SVM_results]

![alt text][world3_SVM_results]

![alt text][world3_f1_color]

![alt text][world3_f1_normal]

2) Let PR2 work for object recognition and pick & place task
I correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 100% of items from `pick_list_2.yaml` for `test2.world` and 87.5% of items from `pick_list_3.yaml` in `test3.world`. The results can be found in each output file.

![alt text][world2_output]

![alt text][world3_output]

##### Results analysis:

I asked PR2 to finished all pick & place task in world 1 but not for world 2. For world 3, I firstly let PR2 to do pick and place task but it stuck at somewhere and all the remain target items fall to the floor. So I just disable pick and place functionality finally by comment the related code. Here are some screenshots for its achievements.

![alt text][all things]

![alt text][world3_recognition]

![alt text][pick and place]


##### Issues analysis:

1) Two biscuits
There are two biscuits recognized in world 1 although just one here.

![alt text][two biscuits]

2) PR2 become `crazy`
The robot's gripper getting stucked for picking up one of the target in world 3. And sometimes it gets the item to the bin, but eventually the object just dropped on the floor (the object reached the edge of bin).

3) Not 100% recognition for world 3
The model cannot perfectly recognize all the 8 items in world 3 because it regards glue as soap2.

##### Further Enhancements:

1) Improve the accuracy of the model
2) Make the movement of robot's arm more accurate with reasonable speed.

