# Communication
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

# Our setting
1. 已知pose: ```物料區原料的中心*3, 車上籃子的中心*1, 籃子四格的中心*4, 加工區桌子放置籃子中心的點*1, 加工區桌子出貨的區域中心*1```
2. 需要硬體: ```物料區桌子電工膠帶分區,車上用的四格籃子,固定四格籃子的機構,```
3. 需要技術: ```不規則物與方形的夾點判斷```

# Grasping 
1. 桌上用電工膠帶貼,有分三方形區: USB線,相機,腳架. 
   物品隨機放在區域內, 三方形區中心pose已知
2. 使用夾點判斷, 依序對三區內的物品夾取
3. 放入已知位置的四格籃子
