# RustBot
Software tools for the project Sistemas Embarcados de Vistoria (SEV). We refer to the hardware platform as RustBot because the AVT Mako cameras look kind of rusty (sorry for the lack of creativity).

If you want to use windows for connecting to the sensor check out this [repository](https://github.com/carlosmccosta/RustBotCSharp)

## Table of Contents

* [The Robot](#therobot)
* [Quick Start](#quickstart)
* [Installation](#installation)
* [Connecting via ssh](#connectingssh)
* [Usage](#usage)
* [Calibration](#calibration)
* [Tunning Stereo Parameters](#tunningparameters)
* [Saving a Bag File](#savingabagfile)
* [Playing Back Data](#playingbackdata)
* [Stereo and SLAM from a Bagfile](#stereobagfile)
* [Getting a bagfile with only raw data from a complete bagfile](#rawfromcomplete)
* [Publishing Data Using ZMQ](#publishingdatazmq)
* [Finding IP Address of Cameras](#findingcameraip)
* [ZeroMQ + Google Protocol Buffers Tutorial](#zeromqtutorial)
* [Compile Google Protocol Buffers Messages](#compilemessages)
* [Update pen-wifi drivers after kernel uptade] (#penwifi)
* [Copy bag files to local computer (Ubuntu)] (#copybagfilesubuntu)
* [Fix a rosbag file (bags collected before 14-12-2016)] (#fixrosbag)
* [Getting a bagfile with only raw data from a complete bagfile] (#rawfromcomplete)
* [Evaluating Odometry Errors] (#evaluateodometry)

## <a name="therobot"></a>The Robot

The RustyBot is a stereo vision system with two AVT Mako cameras mounted on a plate. The baseline is around 0.1 meters, and the cameras are horizontally aligned. Here's a picture of the system

![robot](https://github.com/miguelriemoliveira/RustBot/blob/master/docs/robot.jpg)


## <a name="quickstart"></a>Quick Start

* Connect all inter-cables (processing unit box <--> gimbal camera box), 2 POE ethernets, 1 USB, and 1 Power Cable 
* Connection of an external PC 
 * Option 1, connection with wifi hotspot
   * Connect the WiFi access point (TP-LINK), if there is available internet by ethernet connect the ethernet cable and then the the power transformer. 
    * Connect your PC to the WiFi Hot-spot with SEVsys name
    * The  processing unit box (NUC pc) has the IP 192.168.0.150 on the SEVsys
 * Option 2, connection with ethernet cable
   * Connect an Ethernet cable from your laptop to the processing unit box (Pay attentio there are two POE ethernet and only one normal ethernet)
    * Set a static IP address (the NUC computer has ip 169.254.4.50, so use 169.254.4.51 for example)
    * Ping the NUC Computer to see if everything is ok, i.e. ```ping 169.254.4.50```
    * Launch the ssh connection ```ssh sev@169.254.4.50``` (the password is written in a sticker on the NUC computer)
* Connect the power cable to the processing unit box.
* Wait until gimbal camera box stop of bipping (attention the gimbal camera box must be keep stable for the gyro calibration)



## <a name="installation"></a>Installation

Install the openssh server

```bash
sudo apt-get install openssh-server
```

Install the ros_imresize ros pakcage
```bash
cd ~/catkin_src/src
git clone https://github.com/miguelriemoliveira/ros_imresize
```
First, the Vimba sdk must be downloaded and compiled. We will refer to the folder where the sdk is installed as **$(Vimba_2_0)**.

You may download this sdk from [https://www.alliedvision.com/en/products/software.html](https://www.alliedvision.com/en/products/software.html). Then follow the installation instructions in **$(Vimba_2_0)/Documentation/ReleaseNotes.txt** to install the sdk.

After that, the ros vimba wrapper at [http://wiki.ros.org/avt_vimba_camera](http://wiki.ros.org/avt_vimba_camera) should work. So install the ros wrapper

```bash
git clone https://github.com/srv/avt_vimba_camera.git
catkin_make
```

Install ZMQ Python

```bash
sudo apt-get install python-zmq
```

and google protocol buffers 

```bash
sudo apt-get install python-protobuf
```
Also you should run 

```bash
sudo pip install --upgrade protobuf
```

To use windows based control, you must install the scripts that windows will use through a non-interactive ssh tunnel. First you need to make sure the [.rosrc](https://github.com/miguelriemoliveira/RustBot/blob/master/scripts/.rosrc) file is propperly configured by changing to the ros distro you have.

Then, run:

```bash
roscd rustbot_bringug && cd ../scripts
bash ./install_scripts.bash
```
If asked to replace say yes.

Install VISO2
```
$ cd src
$ wstool init
$ wstool set viso2 --git git://github.com/srv/viso2.git
$ wstool update

```

Install aerial maps visualization in rviz

https://github.com/gareth-cross/rviz_satellite

Install ekf filter 

```bash
sudo apt-get install ros-indigo-robot-pose-ekf ros-indigo-gps-common ros-indigo-robot-localization
```


## <a name="usage"></a>Usage

To start the complete stereo system, just run 

```bash
roslaunch rustbot_bringup all.launch fps:=4 do_stereo:=true do_slam:=true
```
To launch a single camera use

```bash
roslaunch rustbot_bringup left_camera.launch
```
## <a name="connectingssh"></a>Connecting via ssh

To connect to the remote NUC computer

1. Connect an Ethernet cable from your laptop to the switch inside the sensor's box
2. Set a static IP address (the NUC computer has ip 169.254.4.50, so use 169.254.4.51 for example)
3. Ping the NUC Computer to see if everything is ok, i.e. ```ping 169.254.4.50```
4. Launch the ssh connection ```ssh sev@169.254.4.50``` (the password is written in a sticker on the NUC computer)

Note that whenever your ssh tunnel will execute some visual interface application you should start the ssh connection with the _-X_ option, as seen bellow:

4. Launch the ssh connection ```ssh sev@169.254.4.50 -X``` (the password is written in a sticker on the NUC computer)

## <a name="calibration"></a>Calibration

Start the system without the stero processing

```bash
roslaunch rustbot_bringup all.launch do_stereo:=false
```

then startup de calibration

```bash
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.03 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left --approximate=0.05
```

## <a name="tunningparameters"></a>Tunning Stereo Parameters

Read [this](http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters) tutorial.

To tune the parameters of the stereo algorithm, run

```bash
roslaunch rustbot_bringup all.launch fps:=3 config_stereo:=true 
```

if you are running from a rosbag file:
```bash
roslaunch rustbot_bringup all.launch config_stereo:=true online_stereo:=false do_gps:=false
```

and then

```bash
roslaunch rustbot_bringup visualize.launch 
```

Then you can change the parameters an see the effect they have in the disparity map / point cloud in real time.
If you reached a set of parameters you would like to save, do the following:

```bash
roscd rustbot_calibration/calibration/ && rosparam dump stereo_image_proc.yaml /stereo/stereo_image_proc
```
## <a name="savingabagfile"></a>Saving a Bag File

The following commands should be executed from the NUC computer. Thus, if you want to run from a laptop, use an ssh connection as explained in section [Connecting via ssh](#connectingssh).

To record raw data we must first launch the camera drivers (no need to run stereo processing, since this will be done offline)

```bash
roslaunch rustbot_bringup all.launch fps:=4 do_stereo:=false
```

Then, to record messages, run

```bash
roslaunch rustbot_bringup record_raw.launch
```
After stopping (using Ctrl-X) the recorder node, the bag file can be found on the desktop, with the date and hour, e.g., 
_sev_2016-11-24-14-48-30.bag_.

By default all messages are saved but the rosbags are quite big. If you want to save only raw data, i.e., wihtout stereo + slam processing, run

```bash
roslaunch rustbot_bringup record_raw.launch only_raw_data:=true
```


## <a name="playingbackdata"></a>Playing Back Data

```bash
roslaunch rustbot_bringup playback.launch bag:=/home/sev/Desktop/sev_2016-11-03-12-27-31.bag
```

Note: if your bagfile does not contain odometry you can download a dummy odometry from [here](http://criis-projects.inesctec.pt/attachments/download/4003/GPS_IMU_2016-06-14-17-01-59.bag). Then just publish it in addition to the other bags:

```bash
rosbag play GPS_IMU_2016-06-14-17-01-59.bag -l
```
Note2: if your bagfile does not contain odometry info, you can publish dummy /odom messages by running:

```bash
rostopic pub /odom nav_msgs/Odometry <then press tab tab and the message will be filled> -r 10
```

Important: If running the rqt_bag, you must press the right mouse button over all topics in the bag file and select play. Otherwise, the topics are not published.

## <a name="stereobagfile"></a>Stereo and SLAM from a Bagfile

First, you must be playing back recorded data. See section [Playing Back Data](#playingbackdata)

To run the stereo

```bash
roslaunch rustbot_bringup all.launch do_stereo:=true do_slam:=true online_stereo:=false do_gps:=false
```

with accumulation 

```bash
roslaunch rustbot_bringup all.launch do_stereo:=true do_slam:=true online_stereo:=false do_gps:=false do_accumulation:=true
```


Now you should receive both disparity images /stereo/disparity as well as point clouds /stereo/points2


## <a name="publishingdatazmq"></a>Publishing Data Using ZMQ

Note that in order for this node to work, there must be some node publishing images and point clouds.
This can be done online (not yet funcional) or offline.

To _publish images offline_ follow sections [Playing Back Data](#playingbackdata) to publish the recorded images, and also section [Stereo and SLAM from a Bagfile](#stereobagfile) to publish the disparity maps and the point clouds.


To launch the ZMQ publisher, do:

```bash
rosrun rustbot_translation sev_publisher.py
```

Now launch the ZMQ test subscriber (in Ubuntu python) to see if any images are received:

```bash
rosrun rustbot_translation sev_listener.py
```

If the listener receives images, so should the C# application called [RustBotCSharp](https://github.com/carlosmccosta/RustBotCSharp) as long as its propperly configured.

## <a name="findingcameraip"></a>Finding IP Address of Cameras

If the network has no dhcp service, the cameras fall back to a default ip address, which is:

169.254.133.197 Left Camera
169.254.4.55 Right Camera
169.254.4.50 NUC computer, select "SEV local network" in the network manager

To find out the ip address and the id of your cameras. From the **$(VimbaPath)/examples/ListCamera** folder, run

```bash
cd $(Vimba_2_0)
/Tools/Viewer/Bin/x86_64bit/VimbaViewer
```

You shoud see the VimbaViewer gui with all connected cameras with their id (guid). 
If no cameras appear then you may be in a foreign subnet, i.e. have an ipaddress in a different segment. In this case run 

```bash
cd $(Vimba_2_0)
sudo -E /Tools/Viewer/Bin/x86_64bit/VimbaViewer
```
and you should see the cameras

![vimba_viewer](https://github.com/miguelriemoliveira/RustBot/blob/master/docs/vimba_viewer.png)

Select a camera and then, on the right side pane, select All propperties and search for "IP Address" to find out the the ip address.

![get_ipaddress](https://github.com/miguelriemoliveira/RustBot/blob/master/docs/get_ipaddress.png)

You may edit the file **launch/mono_camera.launch** and 
 

By setting the correct guid and ip of your camera.

You can get the guid and ip of your camera by using the ListCamera or VimbaViewer binaries in

Vimba_2_0/VimbaCPP/Examples/Bin/x86_64bit

## <a name="zeromqtutorial"></a> ZeroMQ + Google Protocol Buffers Tutorial

To launch the publisher

```bash
rosrun rustbot_translation example_publisher.py
```

and to receive the subscriber

```bash
rosrun rustbot_translation example_listener.py
```

## <a name="compilemessages"></a>Compile Google Protocol Buffers Messages

To compile all the required messages use the script:

```bash
roscd rustbot_translation && ./compile_proto.sh
```

If you want the compile individual mesages for python use something like:

```bash
roscd rustbot_translation
```

```bash
protoc -I=./msgs -I=/home/mike/workingcopy/protoc-3.1.0-linux-x86_64/include/google/protobuf/ --python_out=src/ msgs/SEVData.proto
```

This assumes you have the protoc version 3 binaries in /home/mike/workingcopy/protoc-3.1.0-linux-x86_64/

You may download [here](https://github.com/google/protobuf/releases/tag/v3.1.0). 

Add a link to this version of protoc.

```bash
mkdir ~/bin && ln -s ~/workingcopy/protoc-3.1.0-linux-x86_64/bin/protoc protoc
```

Make sure the ~/bin folder is the first in your path, otherwise you may be using another version of protoc.
You may confirm this with

```bash
protoc --version
```

See [this](https://developers.google.com/protocol-buffers/docs/pythontutorial) for additional info.

## <a name="penwifi"></a>Update pen-wifi drivers after kernel uptade
```bash
 cd ~/workingcopy/mt7610u_wifi_sta_v3002_dpo_20130916
 
 make
 
 sudo make install
 ```
 more: http://criis-projects.inesctec.pt/issues/2496


## <a name="copybagfilesubuntu"></a>Copy bag files to local computer (Ubuntu)

To copy the bag files from the NUC computer to a local file, first connect to the wireless network as described in section [Quick start](#quickstart)

Then, select Places -- Connect to Server.
Then, add 

```bash
sftp://192.168.0.150/home/sev/Desktop
```

Drag and drop the bag file to the place you want on your local computer.

## <a name="fixrosbag"></a>Fix a rosbag file (bags collected before 14-12-2016) 

Bag files collected before 14-12-2016 had a conflit with the same tf frames being published by slam and mavros.
You can "fix" them by removing the tfs published by mavros:

```bash
rosbag filter original.bag fixed.bag '(topic=="/stereo/left/image_raw/compressed" or topic =="/stereo/left/image_color/compressed" or topic=="/stereo/left/camera_info" or topic=="/stereo/right/image_raw/compressed" or topic=="/stereo/right/image_color/compressed" or topic=="/stereo/right/camera_info" or topic=="/mavros/global_position/raw/fix" or topic=="/mavros/imu/data" or topic=="/mavros/imu/data_raw" or topic=="/mavros/global_position/raw/gps_vel" or topic=="/mavros/global_position/raw/global" or topic=="/stereo_odometry" or topic=="/stereo/points2") or (topic=="/tf" and m.transforms[0].header.frame_id!="map")'
```

## <a name="rawfromcomplete"></a>Getting a bagfile with only raw data from a complete bagfile

Assume you have a bag file (_complete.bag_) with all the system messages. You will have also messages on topics _/stereo/points2_ and _/stereo_camera/odometry_. These messages are produced by the processing of the stereo and SLAM nodes respectivelly.

It is possible to extract from the _complete.bag_ a _rawdata.bag_. To do this, execute:

```bash
rosbag filter sev_2016-12-14-12-19-55_fixed.bag sev_2016-12-14-12-19-55_fixed_raw2.bag  'topic=="/stereo/left/image_raw/compressed" or topic =="/stereo/left/image_color/compressed" or topic=="/stereo/left/camera_info" or topic=="/stereo/right/image_raw/compressed" or topic=="/stereo/right/image_color/compressed" or topic=="/stereo/right/camera_info" or topic=="/mavros/global_position/raw/fix" or topic=="/mavros/imu/data" or topic=="/mavros/imu/data_raw" or topic=="/mavros/global_position/raw/gps_vel" or topic=="/mavros/global_position/raw/global" or (topic=="/tf" and m.transforms[0].header.frame_id!="odom" and m.transforms[0].header.frame_id!="odom_vehicle" and m.transforms[0].child_frame_id!="odom" and m.transforms[0].child_frame_id!="odom_vehicle")'
```

Note: if you have a bagfile collected before 14-12-2016 you may want to fix it first (see [Fix a rosbag file (bags collected before 14-12-2016) ](#fixrosbag)).

##Correcting the frame id of IMU data

```
roscd rustbot_translation/
 python rosbag_correct_imu_frame.py -i /home/sev/Desktop/sev_2016-12-14-12-19-55_fixed_raw2.bag -o /home/sev/Desktop/sev_2016-12-14-12-19-55_fixed_raw2_imu.bag
 ```

## <a name="evaluateodometry"></a>Evaluating Odometry Errors

You must first launch the normal playback and stereo+slam processing, i.e. see [Playing Back Data](#playingbackdata)
 and [Stereo and SLAM from a Bagfile](#stereobagfile)
 
 Then, launch:
 
 ```bash
 roslaunch rustbot_bringup evaluate_odom_error.launch
 ```
 
 
