# Task_and_motion_planning

here i have to do a short description of the repo 

## Problem description

the Mobile Anthropomorphic Dual-Arm Robot (MADAR) is going to be place in a sceneario were he is a bartender and has to serve multiple drinks to people at differents locations in the bar 

we are going to analize 2 specific situations in this sceneario to test the capability of the robot:
- 4 drinks requested in different tables
- 3 drinks requested in different tables 

## Creating sceneario in kautham

The first thing is making the 3d models that kautham can load (.obj , .dae) we have used blender to create this models and export them in .dae 

bar: done 

glass: done 

bottle: done

once the models were created we need to add them to the scenario as urdf files so we create an urdf file for all the objects in the scene  the urdf are located in the ktmpb_interfaces/demos/models/obstacles 

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


we modify the xacro from the madar description to be a single urdf that contains all the parts of the MADAR and also removed all the gazebo specific tags that are no require  


then under OMPL_geo_demos/madar_bar/controls  define the control set of the robot all the posible joints we can rotate we can define and initial offset and the joints we want to move right now 

finally we can add all the files to the problem.xml adding all the urdf to create the sceneario the robot and the controller and also defining the planner with and initial joint configuration and ending configuration 

add figure of the MADAR at the Bar


---

## Road map 

- [X] Define the problem
- [X] Create kautham sceneario (models, urdf, and xml files)
- [ ] Define zones and waypoints the robot can travel 
- [X] Task planning create domain.pddl
- [X] task planning create problem.pddl 
- [ ] Benchmark task planners to select the best solution
- [ ] combine task and motion planner 