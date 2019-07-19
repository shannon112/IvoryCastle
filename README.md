# IvoryCastle
## icps demo world
<img src="https://github.com/shannon112/IvoryCastle/blob/master/icps_gazebo/result/icps_demo_world.png" width="600">

## pmc demo world
<img src="https://github.com/shannon112/IvoryCastle/blob/master/pmc_gazebo/result/pmc_demo_world2.png" width="600">

## pmc application
ira_factory dynamically generate robots (and its task nodes)
```
roslaunch pmc_application pmc_spawn.launch
roslaunch pmc_application project_spawn.launch
```

<img src="https://github.com/shannon112/IvoryCastle/blob/master/pmc_application/result/script_generator_result.png" width="800">

standalone robot generator launch w/o task nodes
```
roslaunch pmc_application amir_bringup_sim.launch
roslaunch pmc_application amir_init_pose.launch
roslaunch pmc_application amir_functions_sim.launch
```
<img src="https://github.com/shannon112/IvoryCastle/blob/master/pmc_application/result/isolated_generator_result_carto.png" width="800">
