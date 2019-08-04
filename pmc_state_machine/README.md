# Conversation
IntentFind
```
Q
1.Let's assemble the camera pack.  (assemble/camera)  
2.Let's do the camera pack. (camera/pack)  
3.Please give me the material of camera. (material/camera)備用  
A
1.Sure, let's do this.
2.Ok, I will collect all the parts for you.
3.No problem, I'm going to do it.
```
IntentDelivery
```
Q
1.It's OK to deliver. (to deliver)  
2.It’s ready to shipping area. (ready to)  
3.The camera have been finished. (have been finished)備用  
A
1.Sure, let's do this.
2.Ok, I will deliver them to shipping area.
3.No problem, I'm going to deliver them.
```
Others
```
Q： What are you doing?
A： I'm finding the components of camera.
A： I'm delivering the material of camera.

A: I'm delivering the completed products which are cameras.
A: I'm delivering the packaged camera products.

A: I'm waiting for your command.
```
IntentWhat
```
Q： Please tell me what happened.
A: (image caption response)
```

# Communication
runing
```
roslaunch pmc_state_machine pmc_state_machine_test.launch
```
state machine trigger topic ```/Intent```
```
rostopic pub /Intent std_msgs/String "data: 'IntentFind'"
rostopic pub /Intent std_msgs/String "data: 'IntentDelivery'"
rostopic pub /Intent std_msgs/String "data: 'Others'"
rostopic pub /Intent std_msgs/String "data: 'IntentWhat'"
rostopic pub /Intent std_msgs/String "data: 'Clear'"
```
natural language sentence input ```/chatter```  
```
rostopic pub /chatter std_msgs/String "data: 'hi '"
```
natural language or anything voice output ```/response```  
```
rostopic pub /response std_msgs/String "data: 'hi '"
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
1. 已知pose: 
```
物料區原料類別的中心*3, 
車上籃子的中心*1, 籃子四格的中心*4, 車上放箱子區中心*1
加工區桌子放置籃子中心的點*1, 加工區桌子待出貨的區域中心*1
出貨區產品類別的中心*3 
```
2. 需要硬體: 
```
物料區桌子電工膠帶匡三區,
車上用的四格籃子,車上固定四格籃子的機構, 車上放箱子區的止滑墊
加工區桌子電工膠框出待出貨區, 加工區桌子上固定四格籃子的機構
出貨區桌子電工膠帶匡三區
```
3. 需要技術:  
```不規則物與方形的夾點判斷```

# Grasping 
1. 依序針對原料三區（USB線,相機,腳架）取影像  
   物品隨機放在區域內, 三方形區中心pose已知  
2. 使用夾點判斷, 取三區內的物體
3. 放入車上已知位置的四格籃子

# Placing 
1. 夾取已知中心位置的籃子
2. 放到已知位置的加工區桌子

# Fetching
1. 針對待出貨區取影像
   盒子隨機放在區域內
2. 使用夾點判斷, 取待出貨區的盒子
3. 放到車上已知的放箱子區

# Stacking
1. 針對車上已知的放箱子區取影像
2. 使用夾點判斷, 取放箱子區的盒子, 抓在手上
3. 針對指定要放入的出貨區該區取影像  
   已有盒子放在該區域內
4. 使用框出位置
5. 將盒子疊在該位置上
