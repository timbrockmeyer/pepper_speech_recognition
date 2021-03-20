#Inputs are normally provided by the Speech Recognition engine and the event system of the robot. A call to this function will stimulate the dialog engine with the given input as if this input had been given by the ASR engine
from naoqi import ALProxy
tts = ALProxy("ALTextToSpeech", "192.168.1.100", 9559)
print("starting TTS... we want this to be blocking anyway...")
tts.say("Witness my final form: I am now: Golden! Pepper!")
print("done")
