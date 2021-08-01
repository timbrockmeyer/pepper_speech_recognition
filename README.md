Code from SS 2019 to allow the UOS Pepper/Lou robot to use an external microphone [Respeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/) coupled with a raspberry pi for cloud and offline speech recognition instead of Pepper's own internal (very flawed) microphone array.

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li><a href="#introduction">Introduction</a></li>
    <li>
      <a href="#quick-start">Quick Start</a>
      <ul>
        <li><a href="#pepper-side">Pepper Side</a></li>
        <li><a href="#raspberry-side">Raspberry Side</a></li>
      </ul>
    </li>
    <li><a href="#cloud-credentials">Cloud Credentials</a></li>
    <li>
      <a href="#description-and-usage">Description and Usage</a>
      <ul>
        <li><a href="#pepper-side-details">Pepper Side Details</a></li>
        <li><a href="#raspberry-side-details">Raspberry Side Details</a></li>
        <ul>
           <li><a href="#google-live-streaming-mode">Google Live Streaming Mode</a></li>
           <li><a href="#google-single-utterance-mode">Google Single Utterance Mode</a></li>
           <li><a href="#google-single-utterance-legacy-mode">Google Single Utterance LEGACY Mode</a></li>
           <li><a href="#ibm-single-utterance-mode">IBM Single Utterance Mode</a></li>
           <li><a href="#snips-mode">Snips Mode</a></li>
           <li><a href="#silent-mode">Silent Mode</a></li>
        </ul>
      </ul>
    </li>
    <li><a href="#troubleshooting">Troubleshooting</a></li>
  </ol>
</details>

# Introduction

This repository is split into two parts: the part of the code that needs to be run on pepper in the "pepper" directory and the other half that needs to be (for example) run from the provided companion raspberry pi + respeaker microphone in the "raspberry_pi" directory.

It is strongly advised that you use our provided raspberry pi or recreate its sd card image via the provided ISO file, otherwise you will have to reinstall our code, ROS, rospy, respeaker libraries, IBM watson's libraries, google's cloud libraries, and possibly snips and MQTT/mosquitto, which can be quite troubling to get working on a single raspbian release. For example, we had to start from scratch with Debian Stretch instead of Debian Buster to get everything working on one installation, and a lot of time was spent troubleshooting installing steps for non-supported version combinations of different libraries.

# Quick Start

## Pepper Side:

Connect to your pepper from your device via your favorite console, at the time of writing, for UOS Pepper/Lou, this would be:

```ssh nao@192.168.1.100```

It is helpful to use a tool like [screen](https://www.gnu.org/software/screen/) or [tmux](https://learnxinyminutes.com/docs/tmux/) to make the connection so that it survives flaky wifi disconnects. On normal ssh connections, your executed programs terminate or break horribly on disconnects.

The contents of this repository's "pepper" directory are lying directly in pepper/lou's home directory /home/nao. You can use our all-in-one launch script to launch all processes in a single terminal via:

```bash /home/nao/speech_recognition_v2/_LAUNCH_ALL_needs_fresh_ssh_console.sh```

This should launch the ROS core on pepper and all python bridge scripts to bridge ROS and NaoQi within Pepper by running two different python versions. Pepper is now ready to communicate via ROS with our raspberry speech recognition scripts or any other future ROS work you may come up with.

To shut down the ROS core and all of our Naoqi <-> ROS bridge scripts, we provide the following utility script:

```bash /home/nao/speech_recognition_v2/_KILL_ALL.sh```

Although we did not find any trouble, if you encounter any, it might be a good idea to give Pepper a clean reboot before wanting to work with Pepper's internal microphones again.

## Raspberry Side: 

The content of the repository linked in this repository's "raspberry_pi" directory lie directly in the home directory of the companion raspberry pi in /home/pi

Make sure that the respeaker microphone is connected to the pi - with a USB cable that actually has data lines and not only power lines.

Set pepper as the ROS master:

```bash /home/pi/respeaker_ros/launch_helper_scripts/_SET_PEPPER_AS_ROS_MASTER.sh```
   
We recommend that you use the Google live streaming mode, as it gives the best performance by far. At the time of writing, you get a monthly 60 minutes of free audio transcription on the free tier. You will have to sign up at Google Cloud and create a project and credentials, which will net you a .json credential file, which you should place into the home directory of the pi with the filename: /home/pi/googleCloudSpeechCredentials.json 

Then you can launch the script via: 

```bash /home/pi/respeaker_ros/launch_helper_scripts/google_cloud_livestream.sh```
   
The script should run indefinitely until you kill it via CTRL+C.

In case you simply do not have any cloud credentials or internet connection, you can run the purely offline snips model instead:

```bash /home/pi/respeaker_ros/launch_helper_scripts/snips.sh```

You can also try Google's public API endpoint, which does not require credentials but might be shut down by Google at any time in the future:

```bash /home/pi/respeaker_ros/launch_helper_scripts/google_legacy_single_utterance(free_public_api).sh```
   
# Cloud Credentials

Apart from the snips.ai and Google Single Utterance LEGACY modes, you will require cloud credentials for either Google or IBM on the raspberry pi.

For Google: At the time of writing, you get a monthly 60 minutes of free audio transcription on the free tier. You will have to sign up at Google Cloud and create a project and credentials, which will net you a .json credential file, which you should place into the home directory of the pi with the filename: /home/pi/googleCloudSpeechCredentials.json 

For Watson: Sign up and generate your credentials and API key, then set it as an environment variable on the raspberry pi:

```export IBM_API=XXXyourAPIkeyXXX && source ~/.bashrc```

Older versions of Google's APIs required you to set an environment variable "GOOGLE_APPLICATION_CREDENTIALS" containing the path to the .json file, but this is no longer the case.

It might be a good idea to remove your credentials once you are done working with Pepper, just in case. You can edit ~/.bashrc with nano or vim to remove the IBM credentials.


# Description and Usage

## Pepper Side Details:

The scripts enable Pepper to run as a standalone ROS core and feed recognized speech data from the recognized_speech node into NaoQi's ALDialog.forceInput() and display the recognized text on Pepper's tablet via a nifty HTML file workaround (the tablet is quite locked down otherwise). It also broadcasts Pepper's NaoQi ALTextToSpeech/Status indicator (whether Pepper is currently speaking itself) to ROS under /pepper_speech_status.

We also mute Pepper's internal microphone via ALSA/amixer so that Pepper's internal microphones are not listening and interfering with everything as well. We further expose a ROS node /microphone_volume by which one can mute and unmute pepper's internal microphones. Our raspberry pi side scripts make use of this automatically. Examples to use it manually with volume values from 0-100, representing mute and max volume respectively:

```
rostopic pub --once /microphone_volume std_msgs/Int32 0
rostopic pub --once /microphone_volume std_msgs/Int32 100
```

Ultimately, this also allows you to easily work with Pepper as a ROS master, come up with your own speech recognition solutions on whatever hardware and publish the results via ROS to /recognized_speech, see whether Pepper is speaking under /pepper_speech_status, and mute and unmute Pepper's internal microphones whenever you wish. 

While we managed to install ROS (aided by [ProtolabSBRE/pepper_ros_compiled](https://github.com/ProtolabSBRE/pepper_ros_compiled) and [ProtolabSBRE/pepper_ros_compilation](https://github.com/ProtolabSBRE/pepper_ros_compilation) on Pepper so it can be used as a ROS core directly, which is quite useful, running ROS requires the sourcing of a custom python distribution which does not support NaoQi. Hence you can not have ROS and NaoQi code in the same python script. However, we want to pipe our recognized speech into ALDialog.forceInput() (and conceivably other NaoQi endpoints), so to bridge ROS and NaoQi we built couples of python scripts that communicate with each other via simple and robust localhost sockets on Pepper. One half of the scripts is launched using Pepper's default NaoQi-capable python version, the other half with the ROS-capable python version. Our helper scripts (see the Quickstart section) handle the launching of all these bridging scripts for you.

Without our launch helper script, you would have to open up several terminals from your side to run the scripts one by one at the same time, and while we tried to compile and cross-compile screen or tmux on Pepper, it simply did not work out. It looks like [others](https://github.com/LCAS/spqrel_launch/wiki/System-Tools-on-Pepper) were successful in compiling tmux for pepper before, but we tried many different approaches and maybe the gentoo libraries of our time simply were not compatible anymore, we had to give up after spending too much time here. Hence a flaky wifi disconnect will kill most of the launched processes. It really is a shame that pepper does not come with at least the screen library preinstalled. 

The only functioning workaround to this problem that we could get to work is the use of [nohup](https://man7.org/linux/man-pages/man1/nohup.1.html) to launch our scripts one by one, although this makes reading of live terminal output more difficult (you could for example [tail](https://man7.org/linux/man-pages/man1/tail.1.html) the nohup .out files). Of course you can also just ignore terminal output altogether. The nohup solution is clunky but works, and would be preferred in environments with unreliable wifi.

## Raspberry Side Details:

The raspberry side of the code uses [furushchev/respeaker_ros](https://github.com/furushchev/respeaker_ros) as a base (you can find some additional parameters on how to fine-tune the respeaker microphone to different scenarios in that repository's readme). It facilitates speech recognition in the cloud (Google or IBM, see Credentials) or offline on the pi with poorer performance via snips. We provide helper scripts to launch the 6 different modes:

### Google Live Streaming Mode

This outsources all the heavy lifting to Google and gives the best results by far. Audio is continously piped to Google, so the detection of starting and stopping of utterances is left to Google as well, outperforming the on-board respeaker or snips libraries in this task. While Pepper is speaking (detected via NaoQi ALTextToSpeech/Status on Pepper and communicated to the raspberry via ROS by our bridging scripts), zeroes are piped into the audio stream in real time to prevent Pepper from hearing itself. At the time of writing, Google's live streaming service is limited to 5 minute continuous sessions at a time, and their provided workaround was not functional. We worked around it by restarting the service every 5 minutes, which results in a tiny downtime every 300 seconds, but worked flawlessly otherwise. This is the recommended mode to use our work via (do not forget credentials):

```bash /home/pi/respeaker_ros/launch_helper_scripts/google_cloud_livestream.sh```
   
### Google Single Utterance Mode

Contrary to the Google Live Streaming Mode, here we need to handle cutting the audio according to the start and the end of an utterance ourselves. We use the respeaker library and [Uberi/speech_recognition](https://github.com/Uberi/speech_recognition) for this task, additionally we make a custom cut whenever Pepper starts speaking. This mode might pose a better pricing model or a better solution where internet uplink is not good enough for continuous streaming. Launch it via:

```bash /home/pi/respeaker_ros/launch_helper_scripts/google_cloud_single_utterance.sh```

### Google Single Utterance LEGACY Mode

This uses an older public Google API endpoint, which at the time or writing still works and does not require an API key. It might get rate limited, perform slowly or be completely removed in the future. Otherwise performs like the method above. Launch via:

```bash /home/pi/respeaker_ros/launch_helper_scripts/google_legacy_single_utterance(free_public_api).sh```
   
### IBM Single Utterance Mode

Uses IBM Watson as cloud backend, so you will need credentials (see Credentials). You can probably still get some through the Study Project. Otherwise works like the Google Single Utterance Mode. Launch via:

```bash /home/pi/respeaker_ros/launch_helper_scripts/ibm_single_utterance.sh```

### Snips Mode

At the time of writing, there is not a lot to choose from for offline speech recognition that works on small single board ARM computers like the raspberry pi. For example, picovoice/cheetah is built for small RAM footprints but is proprietary, requires a license and only works on x86 architectures. Mozilla Deep Speech is far too big and slow (3 seconds inference time on an 8-core desktop CPU). Pocketsphinx performance on the other hand is very poor. The 500 MB general model from snips.ai seemed like our best bet, as we definitely wanted to include a fallback for offline scenarios, although performance will obviously not be great for a general speech model of this size. 

Under the hood, we disable snips' own voice activation detection via its mosquitto/MQTT API endpoints "hermes/asr/startListening" and "hermes/asr/stopListening" in order to use the same code we use for the cloud single utterance modes. You can run it via:

```bash /home/pi/respeaker_ros/launch_helper_scripts/snips.sh```
   
### Silent Mode

This runs all the capabilities of the [furushchev/respeaker_ros](https://github.com/furushchev/respeaker_ros/) minus the speech recognition, just in case you ever want it to interface with completely different non-python speech recognition solutions or different versions of python that are incompatible with [furushchev/respeaker_ros](https://github.com/furushchev/respeaker_ros/) - such as most likely IBM's API in the future.

```bash /home/pi/respeaker_ros/launch_helper_scripts/silent.sh```
   
   
Note: We got IBM's Live Streaming Service to run with many workarounds, but performance was worse than Google's and we would not have been able to provide a compatible version on our all-in-one raspberry image.


# Troubleshooting

## Muting Pepper's Microphones

Our scripts mute Pepper's internal microphone by setting the microphone volume in ALSA to zero (from a scale of 0-100): 

```amixer sset Capture 0```

After trying around with pulseaudio and undocumented NaoQi C++ endpoints for a long time, this is the one working method we finally found to mute Pepper's microphones. If, after running our scripts and terminating them, you end up with a deaf Pepper, you may have to set them to the default value of 40 again by hand:

On Pepper directly:

```amixer sset Capture 0```

or via our provided ROS endpoint:

```rostopic pub --once /microphone_volume std_msgs/Int32 40```

We found that a reboot always resulted in a functioning Pepper microphone at the default value so this should hardly pose a problem. It would still be good for everyone working on Pepper to be aware of this in case it ever needs to manually be set due to some bug.
