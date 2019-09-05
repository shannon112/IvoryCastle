# IvoryCastle
<img src="https://static.newmobilelife.com/wp-content/uploads/2017/12/pokemon-go-hoenn-slaking_00.jpg" width="400"/>  <img src="http://res.pokemon.name/common/pokemon/pgl/288.00.png" width="200"/><img src="https://tw.portal-pokemon.com/play/resources/pokedex/img/pm/25ecd635b6ac9803e574229c886ede9a2f1fbd38.png" width="200"/>

```sh
# Main control loop
scorpio_arm_ros_control/src/hardware_transmission_common.cpp
void HwTmIntf::update()

# Testing: calibrate translation and rotation of base
roslaunch pmc_application sghero_testing.launch 

# Testing: Simple get end point, and make arm to move to assigned point or init pose (w/ tf_prefix)
roslaunch pmc_application sghero_bringup_real_controlonly.launch 
rosrun scorpio_bringup getcurrentpose.py
rosrun scorpio_bringup simple_moving.py 
rosrun scorpio_bringup init_move.py
rosrun scorpio_bringup zero_move.py	

# Testing: Using moveit gui to move (w/o tf_prefix)
roslaunch pmc_application sghero_bringup_real_armball.launch

# Integration demo
roslaunch pmc_application sghero_bringup_real_amir.launch 
(roslaunch pmc_application sghero_bringup_real_amr.launch)
(roslaunch pmc_application sghero_init_pose.launch)
roslaunch pmc_application sghero_functions.launch 
```
<img src=https://pre00.deviantart.net/3d42/th/pre/i/2019/041/b/4/one_punch_man_class_s_by_vegito5001-dczfcdn.png width="840"/>

## Dependences

neronbot:  
leg_detector: https://github.com/Herobrixx/people.git -b ros1  
neronbot: https://github.com/willie5588912/neuronbot  -b  multi-bots  
ira_factory: https://github.com/willie5588912/ira_factory  -b  multibots  
ros_aiml: https://github.com/jkllbn2563/ros_aiml master

scorpio: (using the newest code before 2019.08.23)  
IvoryCastle: https://github.com/shannon112/IvoryCastle -b pmccontest_default  
scorpio: https://github.com/willie5588912/scorpio -b pmc_sghero_merge (simulation with scorpio_ethercat removed)  
ira_factory: https://github.com/willie5588912/ira_factory  -b  pmc_sghero (scorpio)  
image_caption_PMC: https://github.com/jkllbn2563/image_caption_PMC -b pmc_sghero  
robot_arm_PMC: https://github.com/jkllbn2563/robot_arm_PMC -b pmc_sghero  
leg_detector: https://github.com/Herobrixx/people -b ros1  

## icps demo world
```
roslaunch icps_gazebo iceira_icps_demo.launch
```

<img src="https://github.com/shannon112/IvoryCastle/blob/master/icps_gazebo/result/icps_demo_world.png" width="400">

## icps application
spawn two robot one camera at once with gmapping or amcl
```js
roslaunch icps_application icps_demo_1Cam2Robot.launch
roslaunch icps_application icps_demo_nav_amcl.launch
roslaunch icps_application icps_demo_nav_gampping.launch
```

<img src="https://github.com/shannon112/IvoryCastle/blob/master/icps_application/result/1cam2robots_gazebo.png" width="400"><img src="https://github.com/shannon112/IvoryCastle/blob/master/icps_application/result/1cam2robots_rviz.png" width="400">

## pmc demo world
```js
roslaunch icps_gazebo iceira_pmc_demo.launch
```

<img src="https://github.com/shannon112/IvoryCastle/blob/master/pmc_gazebo/result/pmc_demo_world2.png" width="600">

## pmc application
ira_factory dynamically generate robots (and its task nodes)
```js
roslaunch pmc_application pmc_spawn.launch
roslaunch pmc_application project_spawn.launch
```

<img src="https://github.com/shannon112/IvoryCastle/blob/master/pmc_application/result/script_generator_result.png" width="800">

standalone robot generator launch w/o task nodes
```js
roslaunch pmc_application neuron_bringup_sim.launch
roslaunch pmc_application neuron_init_pose.launch       #(optional)
roslaunch pmc_application neuron_functions_sim.launch
```
<img src="https://github.com/shannon112/IvoryCastle/blob/master/pmc_application/result/isolated_generator_result_carto.png" width="800">
