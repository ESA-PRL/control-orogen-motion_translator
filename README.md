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

### Building

In order to install, clone the latest version from this repository into your workspace under `control/orogen/motion_translator`, add the following line to `autoproj/manifest` under `layout:`

    - control/orogen/motion_translator

Execute the following to build the package:

    $ autoproj build


## Basic Usage

### motion_translator

#### Inputs

* **`raw_command`** (/controldev/RawCommand)

Raw messages coming from a joystick or gamepad.

#### Outputs

* **`motion_command`** (/base/MotionCommand2D)

Motion commands directed to wheels and steering, for example for the [locomotion_control](https://github.com/hdpr-rover/control-orogen-locomotion_control) package.

* **`ptu_pan_angle`** (/double)

Pan motion command directed to a pan-tilt unit, for example for the [ptu_directedperception](https://github.com/rock-drivers/drivers-orogen-ptu_directedperception) package.

* **`ptu_tilt_angle`** (/double)

Tilt motion command directed to a pan-tilt unit, for example for the [ptu_directedperception](https://github.com/rock-drivers/drivers-orogen-ptu_directedperception) package.

#### Parameters

* **`minSpeedPointTurn`** (/double)

When performing a tight turn the translational speed might reach 0, this is not acceptable as the rover will switch to point-turn mode. This speed parameter provides a small offset speed to prevent automatic mode switching. Arbitrary units.

* **`speedRatioStep`** (/double)

Step by which to increase the maximum movement speed of the platform. Arbitrary units.

* **`ptuMaxSpeed`** (/double)

PTU movement speed higher limit. Arbitrary units.

* **`ptuMaxPanAngle`** (/double)

PTU pan axis higher limit. Arbitrary units.

* **`ptuMinPanAngle`** (/double)

PTU pan axis lower limit. Arbitrary units.

* **`ptuMaxTiltAngle`** (/double)

PTU tilt axis higher limit. Arbitrary units.

* **`ptuMinTiltAngle`** (/double)

PTU tilt axis lower limit. Arbitrary units.

