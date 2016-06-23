# ProjectSEV
Software tools for the Sistemas Embarcados de Vistoria (SEV) project

## Table of Contents

* [The Robot](#therobot)
* [Installation](#installation)
* [Usage](#usage)
* [Calibration](#calibration)
* [Calibration Setup](#calibrationsetup)

## <a name="therobot"></a>The Robot


## <a name="installation"></a>Installation

First, the Vimba sdk must be installed and compiled. We will refer to the folder where the sdk is installed as **$(VimbaPath)**

You may download this sdk from [https://www.alliedvision.com/en/products/software.html](https://www.alliedvision.com/en/products/software.html). Then follow the installation instructions in **$(VimbaPath)/Readme.txt** to install the sdk.

After that, the ros vimba wrapper at [http://wiki.ros.org/avt_vimba_camera](http://wiki.ros.org/avt_vimba_camera) should work. Follow these steps.

Find out the ip address and the id of your cameras. From the **$(VimbaPath)/examples/ListCamera** folder, run

```bash
./ListCameras
```

You shoud see something like

```bash
***********************************
No camera detected ...
***********************************
50-0536881129 - Mako G-192C - Unique ID =  6009961 IP@ =    192.168.1.85 [available]
50-0536881130 - Mako G-192C - Unique ID =  6009962 IP@ =    192.168.1.89 [available]
***********************************
```

In this case we have two cameras connected.

You may edit the file launch/mono_camera.launch 
 

By setting the correct guid and ip of your camera.

You can get the guid and ip of your camera by using the ListCamera or VimbaViewer binaries in

Vimba_2_0/VimbaCPP/Examples/Bin/x86_64bit
