#include "Task.hpp"

using namespace motion_translator;

Task::Task(std::string const& name): TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine): TaskBase(name, engine)
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
    
    pointTurn = false;
    speedRatio = 0.0;
    // These parameters are defined in motion_translator.orogen file
    // Minimum translation speed in m/s before switching to point turn mode
    minSpeedPointTurn = _minSpeedPointTurn.get();
    // Step by which to increase the speedRatio
    speedRatioStep = _speedRatioStep.get();
    
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
        // Process data only when it has actually changed (latching is not required)
        if(joystick_command.axisValue != axis || joystick_command.buttonValue != buttons)
        {
            // This makes the package not work, do not remap this way!        
            // Remap the axis, the joystick has a strange axis mapping...
            /*axis[LEFT][HORIZONTAL] = joystick_command.axisValue[0][0];
            axis[LEFT][VERTICAL] = joystick_command.axisValue[0][1];
            axis[RIGHT][HORIZONTAL] = joystick_command.axisValue[0][2];
            axis[RIGHT][VERTICAL] = joystick_command.axisValue[1][0];*/
            
            // Save values
            axis = joystick_command.axisValue;
            buttons = joystick_command.buttonValue;
            
            // Remap values
            axis_translation = axis[0][0];
            axis_rotation = -axis[0][1];
            
            // Increase or decrease the speedRatio
            if(buttons[Y] && speedRatio < 1.0)
            {
                speedRatio += speedRatioStep;
            }
            else if(buttons[A] && speedRatio > 0.0)
            {
                speedRatio -= speedRatioStep;
            }
            
            // Toggle point turn mode
            if(buttons[X])
            {
                // Toggle the mode
                pointTurn = !pointTurn;
                
                // TODO switching does not work properly
                if(pointTurn)
                {
                    // Force the switching by sending a tiny rotational command
                    motion_command.translation = 0.0;
                    motion_command.rotation = minSpeedPointTurn;
                    _motion_command.write(motion_command);
                    motion_command.translation = 0.0;
                    motion_command.rotation = 0.0;
                    _motion_command.write(motion_command);
                }
                else
                {
                    // Force the switching by sending a tiny translational command
                    motion_command.translation = minSpeedPointTurn;
                    motion_command.rotation = 0.0;
                    _motion_command.write(motion_command);
                    motion_command.translation = 0.0;
                    motion_command.rotation = 0.0;
                    _motion_command.write(motion_command);
                }
            }
            
            if(pointTurn)
            {
                // In point turn mode the translational speed is always 0
                motion_command.translation = 0.0;
                motion_command.rotation = axis_rotation * speedRatio;
            }
            else
            {
                // In Ackermann mode the translational speed is never 0
                // TODO fix this, the direction behaves strangely when going in the 3rd and 4th quadrants from 1st and 2nd
                if(axis_translation != 0.0)
                {
                    motion_command.translation = axis_translation * speedRatio;
                    motion_command.rotation = axis_rotation * speedRatio;
                }
                else if(axis_rotation != 0.0)
                {
                    // Add a small theshold when rotation is requested, but translation is 0
                    motion_command.translation = minSpeedPointTurn;
                    motion_command.rotation = axis_rotation * speedRatio;
                }
                else
                {
                    // Stop when axis are both 0
                    motion_command.translation = 0.0;
                    motion_command.rotation = 0.0;
                }
            }
            
            // Point turn left
            /*if(buttons[X])
            {
                // Rover switches to point turn when translational speed is 0
                motion_command.translation = 0.0;
                // Speed in point turn mode is only dependent on the speed ratio
                motion_command.rotation = speedRatio;
            }
            else if(buttons[B])
            {
                motion_command.translation = 0.0;
                motion_command.rotation = -speedRatio;
            }
            else if(axis_rotation != 0.0)
            {
                // As long as X or B are not pushed the rover does Ackermann steering
                motion_command.translation = axis_translation * speedRatio;
                
                if(motion_command.translation == 0.0)
                {
                    // The rover switches to Ackermann when translation speed is 0,
                    // prevent that from happening
                    motion_command.translation = minSpeedPointTurn;
                }
                motion_command.rotation = axis_rotation * speedRatio;
            }*/
            
            _motion_command.write(motion_command);
            
            // TODO PTU control with the right joystick
            
            // TODO use the front, back and start buttons to do something
            
            // TODO use the start button to emit a beep for example (connection test)
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
    // TODO reset parameters?
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

