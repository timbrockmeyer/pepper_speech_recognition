source /home/nao/System/setup_ros1_pepper.bash;
python ~/speech_recognition_v2/scripts/speech_status_bridge_rospy.py &
python ~/speech_recognition_v2/scripts/speech_bridge_rospy.py &
python ~/speech_recognition_v2/scripts/set_microphone_volume.py & 
