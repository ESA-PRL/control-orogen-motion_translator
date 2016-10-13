# Motion Translator

## Overview

Package transforming the joystick raw commands to motion commands for wheels, steering and PTU.

**Authors: Karl Kangur  
Contact: Martin Azkarate  
Affiliation: Automatipon and Robotics Laboratories, ESTEC, ESA**


## Installation

### Dependencies

This package depends on the following packages:

* [drivers/controldev](https://github.com/rock-drivers/drivers-controldev)
* [drivers/orogen/controldev](https://github.com/rock-drivers/drivers-orogen-controldev)

To add the dependancies add these lines to `autoproj/manifest` file under `layout:`
    - drivers/controldev
    - drivers/orogen/controldev

### Building

In order to install, clone the latest version from this repository into your workspace under `control/orogen/motion_translator`, add the following line to `autoproj/manifest` under `layout:`
    - control/orogen/motion_translator

Execute the following to build the package
    $ autoproj build


## Basic Usage

### motion_translator

#### Inputs

* **`raw_command`** (/controldev/RawCommand)

Raw messages coming from a joystick or gamepad.

#### Outputs

* **`motion_command`** (/base/MotionCommand2D)

Motion commands directed to wheels and steering, for example for the [locomotion_control](https://github.com/hdpr-rover/control-orogen-locomotion_control) package.

