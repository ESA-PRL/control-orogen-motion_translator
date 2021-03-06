#include "Task.hpp"
#include <math.h>
#include <base/commands/Joints.hpp>

using namespace motion_translator;
using namespace locomotion_switcher;

Task::Task(std::string const& name):
    TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine):
    TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if(!TaskBase::configureHook())
    {
        return false;
    }

    ptu_maxPanAngle = _ptuMaxPanAngle.get();
    ptu_minPanAngle = _ptuMinPanAngle.get();
    ptu_maxTiltAngle = _ptuMaxTiltAngle.get();
    ptu_minTiltAngle = _ptuMinTiltAngle.get();
    ptu_maxSpeed = _ptuMaxSpeed.get();
    minSpeedPointTurn = _minSpeedPointTurn.get();
    // Set linear (speedRatio) and angular (angularSpeedRatio) max-Velocities according to config
    speedRatioStep = _speedRatioStep.get();

    
    speedRatio = _initialSpeedRatio.get();
    angularSpeedRatio = _initialSpeedRatio.get();

    // Initialize the motion_command message parameters
    motion_command.translation = 0.0;
    motion_command.rotation = 0.0;

    axis_translation = 0.0;
    axis_rotation = 0.0;
    axis_pan = 0.0;
    axis_tilt = 0.0;
    sent_zero = false;

    ptu_pan_angle = 0.0;
    ptu_tilt_angle = 0.0;
    ptu_command.resize(2);
    ptu_command.names[0] = "MAST_PAN";
    ptu_command.names[1] = "MAST_TILT";
    
    pointTurn = false;

    locomotion_mode = LocomotionMode::DRIVING;

    return true;
}

bool Task::startHook()
{
    if(!TaskBase::startHook())
    {
        return false;
    }
    ptu_command["MAST_PAN"].position=base::NaN<float>();
    ptu_command["MAST_TILT"].position=base::NaN<float>();
    ptu_command["MAST_PAN"].speed=0.0;
    ptu_command["MAST_TILT"].speed=0.0;

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    
    joystick_command_prev = joystick_command;
    // Process data from the joystick, sent regularly even when there are no changes
    if(_raw_command.read(joystick_command) == RTT::NewData)
    {            

        // Prevent exception if there is no single entry in previous joystick_command
        if (joystick_command_prev.axes.empty())
        {
            joystick_command_prev = joystick_command;
        }

        // TODO stop in case the communication with the joystick is lost (timeout), this process is called periodically, one can use a counter as the timeout
        // TODO ignore first command from the joystick as it might not be sending any data (not sure if actually an issue), for some reason the wheels turn at the beginning
        // When the joystick is not yet functional (a button has net been pressed) the receieved values are not set to 0 by default. Instead they are set to 1, -1, -1, -1 for the joystick axes.

        // The PTU is controlled in incremental mode, this must be called every time, regardless if changed or not

        axis_pan = joystick_command.axes["ABS_Z"];
        axis_tilt = -joystick_command.axes["ABS_RZ"];

        ptu_command["MAST_PAN"].speed= axis_pan*ptu_maxSpeed;
        ptu_command["MAST_TILT"].speed= axis_tilt*ptu_maxSpeed;
        // _ptu_command.write(ptu_command); // avoid sending zero commands continuously

        if ((axis_pan == 0.0) && (axis_tilt == 0.0))
        {
            if (!sent_zero)
            {
                _ptu_command.write(ptu_command);
                sent_zero = true;
            }
        }
        else
        {
            _ptu_command.write(ptu_command);
            sent_zero = false;
        }

        if(axis_pan != 0)
        {
            // Make sure the PTU maximum and minimum pan values are not exceeded
            double new_ptu_pan_angle = ptu_pan_angle + ptu_maxSpeed * axis_pan;
            if(new_ptu_pan_angle < ptu_maxPanAngle && new_ptu_pan_angle > ptu_minPanAngle)
            {
                ptu_pan_angle = new_ptu_pan_angle;
            }
            else if(new_ptu_pan_angle > ptu_maxPanAngle)
            {
                ptu_pan_angle = ptu_maxPanAngle;
            }
            else if(new_ptu_pan_angle < ptu_minPanAngle)
            {
                ptu_pan_angle = ptu_minPanAngle;
            }
            
            // Actually send the command the port is connected to somethings
            if(_ptu_pan_angle.connected())
            {
                _ptu_pan_angle.write(ptu_pan_angle);
            }
        }
        
        if(axis_tilt != 0)
        {
            // Make sure the PTU maximum and minimum tilt values are not exceeded
            double new_ptu_tilt_angle = ptu_tilt_angle + ptu_maxSpeed * axis_tilt;
            if(new_ptu_tilt_angle < ptu_maxTiltAngle && new_ptu_tilt_angle > ptu_minTiltAngle)
            {
                ptu_tilt_angle = new_ptu_tilt_angle;
            }
            else if(new_ptu_tilt_angle > ptu_maxTiltAngle)
            {
                ptu_tilt_angle = ptu_maxTiltAngle;
            }
            else if(new_ptu_tilt_angle < ptu_minTiltAngle)
            {
                ptu_tilt_angle = ptu_minTiltAngle;
            }
            
            // Actually send the command if the port is connected to something
            if(_ptu_tilt_angle.connected())
            {
                _ptu_tilt_angle.write(ptu_tilt_angle);
            }
        }

        // print which button has been pressed
        //for (int i = 0; i < sizeof(joystick_command.buttons)/sizeof(joystick_command.buttons[0]); i++)
        //{
        //    if (joystick_command.buttons[i])
        //        std::cout << "button " << i << std::endl;
        //}

        // Toggle locomotion mode
        // Button Y (rising edge) detection:
        if(joystick_command.buttons[3] && !joystick_command_prev.buttons[3])
        {
            if (locomotion_mode == LocomotionMode::DRIVING)
                locomotion_mode = LocomotionMode::WHEEL_WALKING;
            else if (locomotion_mode == LocomotionMode::WHEEL_WALKING)
                locomotion_mode = LocomotionMode::DRIVING;
            _locomotion_mode.write(locomotion_mode);
        }

        // Process data only when it has actually changed (latching is not required)
        if(
            (joystick_command_prev.axes["ABS_Y"] != joystick_command.axes["ABS_Y"]) ||
            (joystick_command_prev.axes["ABS_X"] != joystick_command.axes["ABS_X"]) ||
            (joystick_command_prev.buttons["BTN_Y"] != joystick_command.buttons["BTN_Y"]) ||
            (joystick_command_prev.buttons["BTN_TL"] != joystick_command.buttons["BTN_TL"]) ||
            (joystick_command_prev.buttons["BTN_A"] != joystick_command.buttons["BTN_A"]) ||
            (joystick_command_prev.buttons["BTN_Z"] != joystick_command.buttons["BTN_Z"]) ||
            (joystick_command_prev.buttons["BTN_TR"] != joystick_command.buttons["BTN_TR"])
        )
        {

            axis_translation = joystick_command.axes["ABS_Y"];
            axis_rotation = -joystick_command.axes["ABS_X"];
            
            // Increase or decrease the speedRatio
            if(joystick_command.buttons["BTN_Y"] && speedRatio < 1.0)
            {
                speedRatio += speedRatioStep;
            }
            else if(joystick_command.buttons["BTN_TL"] && speedRatio > speedRatioStep)
            {
                // Never goes lower than one step increment
                speedRatio -= speedRatioStep;
            }

            // Increase or decrease the angularSpeedRatio
            if(joystick_command.buttons["BTN_Z"] && angularSpeedRatio < 1.0)
            {
                angularSpeedRatio += speedRatioStep;
            }
            else if(joystick_command.buttons["BTN_TR"] && angularSpeedRatio > speedRatioStep)
            {
                // Never goes lower than one step increment
                angularSpeedRatio -= speedRatioStep;
            }

            // Toggle point turn mode with X
            // Unfortunately the gamepad is recognized as a Logitech Rumblepad2, therefore button X is mapped to button A
            if(joystick_command.buttons["BTN_A"])
            {
                // Toggle the mode
                pointTurn = !pointTurn;

                if(pointTurn)
                {
                    // Force the locomotion mode switching by sending a tiny
                    // rotational command with 0 translational speed
                    // The speed does not actually get set because locomotion
                    // control sets the speeds to 0 when switching modes
                    motion_command.translation = 0.0;
                    motion_command.rotation = minSpeedPointTurn;

                    // Send the command immediately
                    _motion_command.write(motion_command);

                    // Return as to not send any other speed commands
                    return;
                }
                else
                {
                    // Force the locomotion mode switching by sending a tiny
                    // translational command with 0 rotational speed
                    motion_command.translation = minSpeedPointTurn;
                    motion_command.rotation = 0.0;
                    _motion_command.write(motion_command);
                    return;
                }
            }

            if(pointTurn)
            {
                // In point turn mode the translational speed is always 0,
                // otherwise it will trigger mode switching
                motion_command.translation = 0.0;
                motion_command.rotation = axis_rotation * angularSpeedRatio;
            }
            else
            {
                // In Ackermann mode the translational speed is never 0
                if(axis_translation != 0.0)
                {
                    motion_command.translation = axis_translation * speedRatio;
                    motion_command.rotation = axis_rotation * angularSpeedRatio;
                    // Rotational speed must be reversed when translational speed is negative, or all the wheels will steer the other way
                    if(motion_command.translation < 0)
                    {
                        motion_command.rotation *= -1;
                    }
                }
                else if(axis_rotation != 0.0)
                {
                    // Prevent translation speed reaching 0 or it will trigger mode switching
                    motion_command.translation = minSpeedPointTurn;
                    motion_command.rotation = axis_rotation * angularSpeedRatio;
                }
                else
                {
                    // Stop when both axis are 0
                    motion_command.translation = 0.0;
                    motion_command.rotation = 0.0;
                }
            }
            _locomotion_mode.write(locomotion_mode);
            _motion_command.write(motion_command);
        }
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();

    // Inform user about error
    std::cout << "motion_translator::errorHook: Error encountered, stopping." << std::endl;

    // When an error occurs in this package stop the rover
    motion_command.translation = 0.0;
    motion_command.rotation = 0.0;
    _motion_command.write(motion_command);
}

void Task::stopHook()
{
    TaskBase::stopHook();

    // When the stop hook is called stop the rover
    motion_command.translation = 0.0;
    motion_command.rotation = 0.0;
    _motion_command.write(motion_command);
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
