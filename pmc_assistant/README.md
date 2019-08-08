# ros_aiml

original from: https://github.com/jkllbn2563/ros_aiml/tree/pmc_sghero

### dependence
```
pip install gtts
pip install aiml
pip install PyAudio
```
### text input
```
python aiml_client.py

```
### start the aiml_server, the google tts,and the text process
```
python aiml_server.py

```
### start the tts
```
python aiml_tts_client.py

```

### start the stt
```
python robot.py
```
### start the text process and will publish the trigger to the topic /Intent,and the newest version is merged to the aiml_server.py

```
python text_process.py

```
### if you want to change to data of chatbot

```
cd ../data
vim PMC_E.aiml

```
### if you want to use the ros tts
```

git clone https://github.com/ros-drivers/audio_common.git

roscd audio_common
cd sound_play
roslaunch soundplay_node.launch


```
