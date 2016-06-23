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

ou must first install the Vimba sdk

https://www.alliedvision.com/en/products/software.html

then in

Vimba_2_0/Documentation/ReleaseNotes.txt

Follow the installation instructions

After that, the ros vimba wrapper should work

http://wiki.ros.org/avt_vimba_camera

It does not have instructions on how to use (like the prosilica had), but you just have to edit the launch/mono_camera.launch

By setting the correct guid and ip of your camera.

You can get the guid and ip of your camera by using the ListCamera or VimbaViewer binaries in

Vimba_2_0/VimbaCPP/Examples/Bin/x86_64bit
