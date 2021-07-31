Code from SS 2019 to allow the UOS Pepper/Lou robot to use an external respeaker microphone XXXmodelXXX + raspberry pi for cloud and offline speech recognition.

This repository is split into two parts: the part of the code that needs to be run on pepper in the "pepper" directory and the other half that needs to be (for example) run from the provided companion raspberry pi + respeaker microphone in the "raspberry_pi" directory.

It is strongly advised that you use our provided raspberry pi or recreate its sd card image via the provided ISO, otherwise you will have to reinstall our code, ROS, rospy, respeaker libraries, IBM watson's libraries, google's cloud libraries, and possibly snips and MQTT/mosquitto, which can be quite troubling to get working on a single raspbian release. For example, we had to start from scratch with Debian Stretch instead of Buster to get everything working on one installation, and a lot of time was spent troubleshooting installing steps for non-supported version combinations of different libraries.

# Usage TLDR

## Pepper Side:

Connect to your pepper via your favorite console, at the time of writing this would be:

```ssh nao@192.168.1.100```

It is helpful to use a tool like [screen](https://www.gnu.org/software/screen/) or [tmux](https://learnxinyminutes.com/docs/tmux/) to make the connection so that it survives flaky wifi disconnects. On normal ssh connections, your executed programs terminate or break horribly on disconnects.

The contents of this repository's "pepper" directory are lying directly in pepper/lou's home directory /home/nao. You can use our all-in-one launch script to launch all processes in a single terminal via 

```bash /home/nao/speech_recognition_v2/_LAUNCH_ALL_needs_fresh_ssh_console.sh```

This should launch the ROS core on pepper and all python bridge scripts to bridge it to naoqi. Pepper is now ready to communicate with our raspberry speech recognition scripts or any other future ROS work you may come up with.

To shut down the ROS core and all of our pepper scripts, we provide the utility script

```bash /home/nao/speech_recognition_v2/_KILL_ALL.sh```

## Raspberry Side: 

The content of the repository linked in this repository's raspberry_pi directory lie directly in the home directory of the companion raspberry pi in /home/pi

Make sure that the respeaker microphone is connected to the pi - with a USB cable that actually has data lines.

Set pepper as the ROS master:

```bash /home/pi/respeaker_ros/launch_helper_scripts/_SET_PEPPER_AS_ROS_MASTER.sh```
   
We recommend that you use Google live streaming, as it gives the best performance by far. At the time of writing, you get 60 minutes of free audio transcription on the free tier as well. You will have to sign up at Google and create a project and credentials, which will net you a .json credential file, which you should place into the home directory of the pi with the filename: /home/pi/googleCloudSpeechCredentials.json 

Then you can launch the script via: 

```bash /home/pi/respeaker_ros/launch_helper_scripts/google_cloud_livestream.sh```
   
The script should run indefinitely until you kill it via CTRL+C.

In case you simply do not have any cloud credentials, you can run the purely offline snips model instead:

```bash /home/pi/respeaker_ros/launch_helper_scripts/snips.sh```

You can also try Google's public API endpoint, which they might shut down at any time:

``bash /home/pi/respeaker_ros/launch_helper_scripts/google_legacy_single_utterance(free_public_api).sh```
   
# Cloud Credentials (required on the raspberry pi)

For Google: At the time of writing, you get 60 minutes of free audio transcription on the free tier as well. You will have to sign up at Google and create a project and credentials, which will net you a .json credential file, which you should place into the home directory of the pi with the filename: /home/pi/googleCloudSpeechCredentials.json 

For Watson: Sign up and generate your credentials and API key, then set them as an environment variable on the rapsberry pi:

```export IBM_API=XXXyourAPIkeyXXX && source ~/.bashrc```


# Description and Usage

## Pepper Side:

The scripts enable Pepper to run as a standalone ROS core and feed recognized speech data from the recognized_speech node into NaoQi's ALDialog.forceInput() and display the string on Pepper's tablet via a nifty HTML file workaround (the tablet is quite locked down otherwise). It also broadcasts Pepper's NaoQi ALTextToSpeech/Status indicator (whether Pepper is currently speaking itself) to ROS under /pepper_speech_status.

We also mute Pepper's internal microphone via ALSA/amixer so that Pepper's internal microphones are not listening and interfering with everything as well. We further expose a ROS node /microphone_volume by which one can mute and unmute pepper's internal microphones. Our raspberry pi side scripts make use of this automatically. Examples to use it manually with volume values from 0-100, representing mute and max volume respectively:

```
rostopic pub --once /microphone_volume std_msgs/Int32 0
rostopic pub --once /microphone_volume std_msgs/Int32 100
```

Ultimately, this also allows you to easily work with Pepper as a ROS master, come up with your own speech recognition solutions on whatever hardware and publish the results via ROS to /recognized_speech, see whether Pepper is speaking under /pepper_speech_status

While we managed to install ROS (XXX source) on Pepper so it can be used as a ROS core directly, which is quite useful, running ROS requires sourcing of a custom python distribution which does not support NaoQi. Hence you can not have ROS and NaoQi code in the same python script. However, we want to pipe our recognized speech into ALDialog.forceInput() (and conceivably other NaoQi endpoints), so to bridge the gap we built couples of python scripts that communicate with each other via simple and robust localhost sockets on Pepper. One half of the scripts is launched with Pepper's default NaoQi-capable python version, the other with the ROS-capable python version. Our helper scripts (see Usage TLDR) handle the launching for you.

Without our launch helper script, you would have to open up several terminals from your side to run them all, and while we tried to compile and cross-compile screen or tmux on Pepper, it simply did not work out. Hence a flaky wifi disconnect will kill most of the launched processes. It really is a shame that pepper does not come with at least the screen library preinstalled. XXX successful here but gentoo changed XXX

The only functioning workaround to this problem that we could get to work is by using [nohup](https://man7.org/linux/man-pages/man1/nohup.1.html) to launch the processes one by one, although this makes live terminal output more difficult to observe (you have to [tail](https://man7.org/linux/man-pages/man1/tail.1.html) the nohup .out files). Of course you can also just ignore terminal output altogether. This nohup solution is clunky but works, and would be preferred in environments with unreliable wifi.

## Raspberry Side:

The raspberry side of the code uses [furushchev/respeaker_ros](https://github.com/furushchev/respeaker_ros) as a base. It facilitates speech recognition in the cloud (Google or IBM, see Credentials) or offline on the pi with poorer performance via snips. We provide helper scripts to launch the X different modes:

### Google Live Streaming Mode

This outsources all the heavy lifting to Google and gives the best results by far. Audio is continously piped to Google, so the detection of starting and stopping of utterances is left to Google, outperforming the on-board respeaker or snips libraries in this task as well. While Pepper is speaking (detected via ALTextToSpeech/Status on Pepper and bridged via ROS by our code), zeroes are piped into the audio stream in real time to prevent Pepper from hearing itself. At the time of writing, Google's live streaming service is limited to 5 minute continuous sessions at a time, and their provided workaround was not functional. We worked around it by restarting the service every 5 minutes, which results in a tiny downtime every 300 seconds, but worked flawlessly otherwise. This is the recommended mode to use (do not forget credentials) via:

```bash /home/pi/respeaker_ros/launch_helper_scripts/google_cloud_livestream.sh```
   
### Google Single Utterance Mode

Contrary to the Google Live Streaming Mode, here we need to handle cutting the audio according to the start and the end of an utterance ourselves. We use the respeaker library and [Uberi/speech_recognition](https://github.com/Uberi/speech_recognition) for this task, additionally we make a custom cut whenever Pepper starts speaking. This mode might pose a better pricing model or a better solution where internet uplink is not good enough for continuous streaming. Launch it via:

```bash /home/pi/respeaker_ros/launch_helper_scripts/google_cloud_single_utterance.sh```

### Google Single Utterance LEGACY Mode

This uses an older public Google API endpoint, which at the time or writing still works and does not require an API key. It might get rate limited, perform slowly or be completely removed in the future. Otherwise performs like the method above. Launch via:

```bash /home/pi/respeaker_ros/launch_helper_scripts/google_legacy_single_utterance(free_public_api).sh```
   
### IBM Single Utterance

Uses IBM Watson as cloud backend, so you will need credentials (see Credentials). You can probably still get some through the Study Project. Otherwise works like the Google Single Utterance Mode. Launch via:

```bash /home/pi/respeaker_ros/launch_helper_scripts/ibm_single_utterance.sh```

### SNIPS.AI

At the time of writing, there is not a lot to choose from for offline speech recognition that works on small single board ARM computers like the raspberry pi. For example, picovoice/cheetah is built for small RAM footprints but is proprietary, requires a license and only works on x86 architectures. Mozilla Deep Speech is far too big and slow (3 seconds inference time on an 8-core desktop CPU). Pocketsphinx performance on the other hand is very poor. The 500 MB general model from snips.ai seemed like our best bet, as we definitely wanted to include a fallback for offline scenarios, although performance will obviously not be great for a general speech model of this size. 

Under the hood, we disable snips' own voice activation detection via its mosquitto/MQTT API endpoints "hermes/asr/startListening" and "hermes/asr/stopListening" in order to use the same code we use for the cloud single utterance modes. You can run it via:

```bash /home/pi/respeaker_ros/launch_helper_scripts/snips.sh```
   
### SILENT MODE

This runs all the capabilities of the [furushchev/respeaker_ros](https://github.com/furushchev/respeaker_ros/) minus the speech recognition, just in case you ever want it to interface with completely different non-python speech recognition solutions or different versions of python that are incompatible with [furushchev/respeaker_ros](https://github.com/furushchev/respeaker_ros/) - such as most likely IBM's API in the future.

```bash /home/pi/respeaker_ros/launch_helper_scripts/silent.sh```
   
   
Note: We got IBM's Live Streaming Service to run with many workarounds, but performance was worse than Google's and we would not have been able to provide a compatible version on our all-in-one raspberry image.


# Troubleshooting

## Muting Pepper's Microphones

Our scripts mute Pepper's internal microphone by setting the ALSA microphone volume to zero (from a scale of 0-100): 

```amixer sset Capture 0```

After trying around with pulseaudio and undocumented NaoQi C++ endpoints for a long time, this is the one method we finally found as working in order to mute Pepper's microphones. If, after running our scripts and terminating them, you end up with a deaf Pepper, you may have to set them to the default value of 40 again:

On Pepper directly:

```amixer sset Capture 0```

or via our provided ROS endpoint:

```rostopic pub --once /microphone_volume std_msgs/Int32 40```

We found that a reboot always resulted in a functioning Pepper microphone at the default value. It would still be good for everyone working on Pepper to be aware of this in case it ever needs to manually be set due to some bug.


# Installation hints
Future work XXX
