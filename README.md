# IvoryCastle
<img src="https://static.newmobilelife.com/wp-content/uploads/2017/12/pokemon-go-hoenn-slaking_00.jpg" width="400"/>

## Dependences
neronbot:  
leg_detector: https://github.com/Herobrixx/people.git -b ros1  
neronbot: https://github.com/willie5588912/neuronbot  -b  multi-bots  
ira_factory: https://github.com/willie5588912/ira_factory  -b  multibots  
ros_aiml: https://github.com/jkllbn2563/ros_aiml master

scorpio:  
IvoryCastle: https://github.com/shannon112/IvoryCastle.git -b pmccontest  
scorpio: https://github.com/willie5588912/scorpio -b pmc_sghero (simulation with scorpio_ethercat removed)  
neronbot: https://github.com/willie5588912/neuronbot  -b  pmc_sghero (multi-bots with move_back removed)  
ira_factory: https://github.com/willie5588912/ira_factory  -b  pmc_sghero (scorpio)  
image_caption_PMC: https://github.com/jkllbn2563/image_caption_PMC -b pmc_sghero  
leg_detector: https://github.com/Herobrixx/people.git -b ros1  

scorpio pmc_sghero_tmp ```run seperately on arm or base with tf_prefix but no moveit ball```  
scorpio pmc_sghero_merge ```run togather with tf_prefix but no moveit ball```  
```sh
roslaunch scorpio_bringup arm_bringup_real.launch  #with tf prefix
roslaunch scorpio_bringup arm_partial.launch       #without tf prefix 
```

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
