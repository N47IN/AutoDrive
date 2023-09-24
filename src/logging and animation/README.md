clone the directory into your wokrspace
open the world file as it is in your device on webots   
make sure both controllers are set to extern 
add export WEBOTS_HOME=/home/username/webots to .bashrc  
execute webots-controller.exe --robot-name=vehicle path/to/controller/file/ego_pd_v1.py  
execute webots-controller.exe --robot-name=boo path/to/controller/file/lead_teleop_v1.py  
Rewind the sim to finsih it and generate "diagnostics.csv in path/to/controller/file/ and a data plot
