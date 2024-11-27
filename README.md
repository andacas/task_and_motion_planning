# Task_and_motion_planning

in this repository we explain how to use kautham to analize task and motion planning algorithms for a new robot and a custom scenario, we will go through all the necesary steps and considerations that need to be taken into account. 

For our case we are going to be using the the Mobile Anthropomorphic Dual-Arm Robot [MADAR](https://upcommons.upc.edu/handle/2117/371727) develop by the UPC they provided the [robot description](https://gitioc.upc.edu/robots/madar_description/-/tree/gazebo_implementation?ref_type=heads) for simulating in gazebo, we are going to modify it so we can only use a single urdf file containing all the information of the robot removing xacro functionalities and also removing gazebo specific tags.


## Problem description

The [MADAR](https://upcommons.upc.edu/handle/2117/371727) is going to be place in a sceneario were he is a bartender and has to serve multiple drinks to people at differents locations in the bar 

we are going to analize 2 specific situations in this sceneario to test the capability of the robot:
- 4 drinks requested in different tables
- 3 drinks requested in different tables 


## Creating sceneario in kautham

The first thing is making the 3d models that kautham can load (.obj , .dae) we have used blender to create this models and export them in ktmpb/ktmpb_interfaces/demos/models/obstacles/madar_bar/obj_name.dae

- [bar](ktmpb/ktmpb_interfaces/demos/models/obstacles/madar_bar/madar_case.dae)
- [glass](ktmpb/ktmpb_interfaces/demos/models/obstacles/madar_bar/Glass.dae)
- [bottle](ktmpb/ktmpb_interfaces/demos/models/obstacles/madar_bar/Bottle.dae)

Once the models were created we need to add them to the scenario as urdf files so we create an urdf file for all the objects in the scene  the urdf are located in the ktmpb/ktmpb_interfaces/demos/models/obstacles/madar_bar 

structure of a urdf file that the kautham can read 
```
<?xml version="1.0"?>
<robot name="OBJ">
	<link name="base">
	<inertial>
       </inertial>
	<visual>
		<origin xyz="0 0 0" rpy="0 0 0" />
		<geometry>
       			 <mesh filename="mesh.dae" scale="0 0 0"/>
      		</geometry>
		<material>
			<color rgba="1 1 1 1" />
		</material>
		</visual>
  	 <collision>
		<origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
       			 <mesh filename="mesh.dae" scale="0 0 0"/>
            </geometry>
	    <material>
			<color rgba="1 1 1 1" />
	    </material>
        </collision>
	</link>
</robot>
```

URDF Models :

- [bar](ktmpb/ktmpb_interfaces/demos/models/obstacles/madar_bar/madar_case.urdf)
- [glass](ktmpb/ktmpb_interfaces/demos/models/obstacles/madar_bar/glass1.urdf)
- [bottle](ktmpb/ktmpb_interfaces/demos/models/obstacles/madar_bar/bottle_R.urdf)

we modify the xacro from the madar description to be a single urdf that contains all the parts of the MADAR and also removed all the gazebo specific tags that are no require  thus creating a single URDF file called [madar.urdf](ktmpb/ktmpb_interfaces/demos/models/robots/madar/madar.urdf)

then under ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/controls  define the control set of the robot all the posible joints we can rotate we can define and initial offset and the joints we want to move right now, here some of the created controllers: 

- [controller just for moving the platform ](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/controls/madar_R2.cntr)
- [controller all the joints at the same time](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/controls/madar.cntr)


finally we can add all the files to the problem.xml adding all the urdf to create the sceneario, the robot and the controller and also defining the type of planner with and initial and goal joint configuration , we created multiple of these files in order to benchmark them and select the best for our sceneario.

- [LazyRRT move from home to zone 3](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_LazyRRT_madar_case_move_a_b.xml)
- [PRM GAUSSIAN](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_PRM_madar_case_move_a_b_g.xml)
- [PRM H](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_PRM_madar_case_move_a_b_h.xml)
- [PRM r](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_PRM_madar_case_move_a_b_r.xml)
- [PRM sdk](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_PRM_madar_case_move_a_b_sdk.xml)
- [RRTconnect](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_RRTconnect_madar_case_move_a_b.xml)



we will have to do also motion planning and bench mark for analizing the arm behaviour 

- [LazyRRT move from home to zone 3](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_LazyRRT_madar_case_move_a_b.xml)
- [PRM GAUSSIAN](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_PRM_madar_case_move_a_b_g.xml)
- [PRM H](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_PRM_madar_case_move_a_b_h.xml)
- [PRM r](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_PRM_madar_case_move_a_b_r.xml)
- [PRM sdk](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_PRM_madar_case_move_a_b_sdk.xml)
- [RRTconnect](ktmpb/ktmpb_interfaces/demos/OMPL_geo_demos/madar_bar/OMPL_RRTconnect_madar_case_move_a_b.xml)

add figure of the MADAR at the Bar


after that we had to define all the waypoints in the scene moving by hand the slider in the kautham-gui and loading the problem with the move controller 

| zone name     | X             | Y             | Z     |
| ------------- |:-------------:|:-------------:| -----:|
| HOME   | 0.5 |0.5   | 0.5  |
| P1     | 0.5 |0.25  | 0.75 |
| P2     | 0.25 |0.25  | 0.5  |
| P3     | 0.25 |0.75  | 0.5  |
| P4     | 0.5 |0.75  | 0.25 |
| PREP   | 0.8 |0.3   | 0.0  |
| SHELF  | 0.75 |0.75   | 0.25  |


we will have to do also motion planning and bench mark for analizing the arm behaviour
| zone name     | j1             | j2            | j3     |j4             | j5            | j6     |
| ------------- |:-------------:|:-------------:| -----:|:-------------:|:-------------:| -----:|
| P1-left     | 0.5 |0.25  | 0.75 |0.5 |0.25  | 0.75 |
| P1-right    | 0.2 |0.25  | 0.5  |0.5 |0.25  | 0.75 |
| P1-center   | 0.2 |0.75  | 0.5  |0.5 |0.25  | 0.75 |
| P4     | 0.5 |0.75  | 0.25 |0.5 |0.25  | 0.75 |
| PREP   | 0.8 |0.3   | 0.0  |0.5 |0.25  | 0.75 |
| SHELF  | 0.75 |0.75   | 0.25  |0.5 |0.25  | 0.75 |

now that the motion planner has been decided 

we proceed to create the pddl files 

the pddl domain file: downward_ros/downward_interfaces/pddl/madar_world.pddl
the pddl problem file: downward_ros/downward_interfaces/pddl/madar_problem


---

## Road map 

- [X] Define the problem
- [X] Create kautham sceneario (models, urdf, and xml files)
- [X] Define zones and waypoints the robot can travel 
- [X] Task planning create domain.pddl
- [X] task planning create problem.pddl 
- [ ] Benchmark task planners to select the best solution
- [ ] combine task and motion planner 