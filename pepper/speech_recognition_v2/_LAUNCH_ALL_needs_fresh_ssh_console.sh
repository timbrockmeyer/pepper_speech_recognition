/bin/bash 1_LAUNCH_NAOQI_SCRIPTS.sh;
source /home/nao/System/setup_ros1_pepper.bash;
/bin/bash 2_LAUNCH_ROSCORE_ON_PEPPER.sh;
sleep 15;
/bin/bash 3_LAUNCH_ROSPY_SCRIPTS.sh;
python scripts/utils/golden_pepper.py
