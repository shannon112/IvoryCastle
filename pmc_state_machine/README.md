runing
```
rosrun pmc_state_machine state_machine_nesting2.py
rosrun smach_viewer smach_viewer.py
```
trigger Intent
```
rostopic pub /Intent std_msgs/String "Others" -r 10
rostopic pub /Intent std_msgs/String "IntentFind" -r 10
rostopic pub /Intent std_msgs/String "IntentDelivery" -r 10
rostopic pub /Intent std_msgs/String "IntentWhat" -r 10
rostopic pub /Intent std_msgs/String "Clear" -r 10
```
