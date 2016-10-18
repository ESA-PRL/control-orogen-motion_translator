#include "Task.hpp"
#include <math.h>

using namespace motion_translator;

Task::Task(std::string const& name):
    TaskBase(name),
    ptu_maxPanAngle(_ptuMaxPanAngle.get()),
    ptu_minPanAngle(_ptuMinPanAngle.get()),
    ptu_maxTiltAngle(_ptuMaxTiltAngle.get()),
    ptu_minTiltAngle(_ptuMinTiltAngle.get()),
    ptu_maxSpeed(_ptuMaxSpeed.get()),
    minSpeedPointTurn(_minSpeedPointTurn.get()),
    speedRatioStep(_speedRatioStep.get())
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine):
    TaskBase(name, engine),
    ptu_maxPanAngle(_ptuMaxPanAngle.get()),
    ptu_minPanAngle(_ptuMinPanAngle.get()),
    ptu_maxTiltAngle(_ptuMaxTiltAngle.get()),
    ptu_minTiltAngle(_ptuMinTiltAngle.get()),
    ptu_maxSpeed(_ptuMaxSpeed.get()),
    minSpeedPointTurn(_minSpeedPointTurn.get()),
    speedRatioStep(_speedRatioStep.get())
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
    
    // Initialize the motion_command message parameters
    motion_command.translation = 0.0;
    motion_command.rotation = 0.0;
    
    axis_translation = 0.0;
    axis_rotation = 0.0;
    axis_pan = 0.0;
    axis_tilt = 0.0;
    
    ptu_pan_angle = 0.0;
    ptu_tilt_angle = 0.0;
    
    pointTurn = false;
    
    // Minimum speed is the smallest step increment
    speedRatio = speedRatioStep;
    
    return true;
}

bool Task::startHook()
{
    if(!TaskBase::startHook())
    {
        return false;
    }
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    
    // Process data from the joystick, sent regularly even when there are no changes
    if(_raw_command.read(joystick_command) == RTT::NewData)
    {            
        // TODO stop in case the communication with the joystick is lost (timeout), this process is called periodically, one can use a counter as the timeout
        // TODO ignore first command from the joystick as it might not be sending any data (not sure if actually an issue), for some reason the wheels turn at the beginning
        // When the joystick is not yet functional (a button has net been pressed) the receieved values are not set to 0 by default. Instead they are set to 1, -1, -1, -1 for the joystick axes.
        
        // The PTU is controlled in incremental mode, this must be called every time, regardless if changed or not
        axis_pan = joystick_command.axisValue[0][2];
        axis_tilt = -joystick_command.axisValue[1][0];
        
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
        
        // Process data only when it has actually changed (latching is not required)
        if(joystick_command.axisValue != axis || joystick_command.buttonValue != buttons)
        {
            buttons = joystick_command.buttonValue;
            axis_translation = joystick_command.axisValue[0][0];
            axis_rotation = -joystick_command.axisValue[0][1];
            
            // Increase or decrease the speedRatio
            if(buttons[LB] && speedRatio < 1.0)
            {
                speedRatio += speedRatioStep;
            }
            else if(buttons[LT] && speedRatio > speedRatioStep)
            {
                // Never goes lower than one step increment
                speedRatio -= speedRatioStep;
            }
            
            // Toggle point turn mode with X
            if(buttons[X])
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
                motion_command.rotation = axis_rotation * speedRatio;
            }
            else
            {
                // In Ackermann mode the translational speed is never 0
                if(axis_translation != 0.0)
                {
                    motion_command.translation = axis_translation * speedRatio;
                    motion_command.rotation = axis_rotation * speedRatio;
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
                    motion_command.rotation = axis_rotation * speedRatio;
                }
                else
                {
                    // Stop when both axis are 0
                    motion_command.translation = 0.0;
                    motion_command.rotation = 0.0;
                }
            }
            
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
    
    // Inform user about error
    std::cout << "motion_translator::stopHook: Stopping the platform." << std::endl;
    
    // When the stop hook is called stop the rover
    motion_command.translation = 0.0;
    motion_command.rotation = 0.0;
    _motion_command.write(motion_command);
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

