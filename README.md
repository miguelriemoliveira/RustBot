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

First, the Vimba sdk must be installed and compiled.

You may download this sdk from [https://www.alliedvision.com/en/products/software.html](https://www.alliedvision.com/en/products/software.html). Then follow the installation instructions in Vimba_2_0/Documentation/ReleaseNotes.txt to install the sdk.

After that, the ros vimba wrapper at [http://wiki.ros.org/avt_vimba_camera](http://wiki.ros.org/avt_vimba_camera) should work. You may edit the file 

```file
 launch/mono_camera.launch
 ```

By setting the correct guid and ip of your camera.

You can get the guid and ip of your camera by using the ListCamera or VimbaViewer binaries in

Vimba_2_0/VimbaCPP/Examples/Bin/x86_64bit
