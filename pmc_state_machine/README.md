runing
```
roslaunch pmc_state_machine pmc_state_machine_test.launch
```
/Intent topic
```
rostopic pub /Intent std_msgs/String "data: 'IntentFind'"
rostopic pub /Intent std_msgs/String "data: 'IntentDelivery'"
rostopic pub /Intent std_msgs/String "data: 'Others'"
rostopic pub /Intent std_msgs/String "data: 'IntentWhat'"
rostopic pub /Intent std_msgs/String "data: 'Clear'"
```

/triggerCaption rosservice server  
/triggerGrasping rosservice server  
/triggerPlacing rosservice server  
/triggerFetching rosservice server  
/triggerStacking rosservice server  
```
please checkout script/fake_server.py
```
