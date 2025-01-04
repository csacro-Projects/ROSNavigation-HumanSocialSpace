# Navigating without Intruding Humans' Social Space: A Cost-based Approach
The contents of this repository were created in the scope of the project "Autonomous Systems" at Ulm University.

## Repository Structure

### Package: socialspace_navigation_layer
This is the main contribution of the project.  
It contains the implementation of the modular cost function for modelling humans' social space along with the jupyter notebook playground.  
Moreover, the employment of the cost function as the social space navigation layer is in this package.

### Package: humans
msg: contains the messages  
src: contains "mock" publishers for humans messages, there is a file for each scenario

### Package: people_publisher
src: contains "mock" publishers for people messages, there is a file for each scenario

### Package: navgoal_publisher
src: contains a node to publish navigation goals to the robot

### worlds_for__tiago_simulation__tiago_gazebo__worlds
contains worlds for the respective scenarios with the TIAGo robot, need to be placed in the following directory/package `tiago_simulation/tiago_gazebo/worlds`

### Package: navigation_baseline
launch: contains a file for launching the navigation with "mock" publishers for people messages and the respective social_navigation_layers ([Proxemic and Passing Layer](http://wiki.ros.org/social_navigation_layers))  
config: contains the respective config files for the navigation

### Package: navigation_socialspace
launch: contains a file for launching the navigation with "mock" publishers for human messages and the respective socialspace_navigation_layer (our SocialSpace Layer from above)  
config: contains the respective config files for the navigation


## How to Use
Make sure that all `.py` (and to be on the save side all `.launch`) files have execute permissions.  
In order to so, execute the following in this directory
- `chmod +x humans/src/*.py people_publisher/src/*.py socialspace_navigation_layer/src/*.py navgoal_publisher/src/*.py`
- `chmod +x navigation_baseline/launch/*.launch navigation_baseline/launch/*.launch`

### Navigation Layer only
You can use the `socialspace_navigation_layer` along with the `humans` package on its own.
It can be used in the costmap as a plugin. See [ROS-Plugins_Specification](http://wiki.ros.org/costmap_2d/Tutorials/Configuring%20Layered%20Costmaps#Step_3:_Plugins_Specification) for more details.
You may also use the packages `navigation_baseline` and `navigation_socialspace` as a reference for integration (see `config/base/common/` directory for plugin usage).

In order to select the hyperparemters of the navigation layer, you may want to make use of [rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure) via: `rosrun rqt_reconfigure rqt_reconfigure`.  
If you want to navigate the robot to a specific destination (without making use of the rviz UI), you may want to run: `rosrun navgoal_publisher navgoal.py <dest_x> <dest_y> <dest_yaw>`. 

### with TIAGo
Install ROS with TIAGo according to the following description with the deviations noted below: http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS
- before running rosdep:
	- place all packages of this repository into the `src` directory of the TIAGo workspace
	- place the files from the `worlds_for__tiago_simulation__tiago_gazebo__worlds` directory into the directory/package `tiago_simulation/tiago_gazebo/worlds`
	- if you also want to run the baseline, clone the following github repository into the `src` directory: https://github.com/wg-perception/people.git (branch: melodic)

Even when you have TIAGo already installed, it is required that you perform the above named steps and execute rosdep and catkin again as described in the above-linked TIAGo installation tutorial.

### Running the Experiments with TIAGo
baseline: `roslaunch navigation_baseline tiago_navigation.launch public_sim:=true scenario:=scenario#` (replace # with the respective scneario number)  
socialspace: `roslaunch navigation_socialspace tiago_navigation.launch public_sim:=true scenario:=scenario#` (replace # with the respective scneario number)

In order to manipulate the hyperparemters of the navigation layer, you may want to make use of [rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure) via: `rosrun rqt_reconfigure rqt_reconfigure`.  
If you want to navigate the robot to a specific destination (without making use of the rviz UI), you may want to run: `rosrun navgoal_publisher navgoal.py <dest_x> <dest_y> <dest_yaw>`. 
