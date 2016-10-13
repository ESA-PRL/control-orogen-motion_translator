/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

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

    // Process data from the joystick
    if(_raw_command.read(joystick_command) == RTT::NewData)
    {
        if(joystick_command.axisValue != axis || joystick_command.buttonValue != buttons)
        {
            axis = joystick_command.axisValue;
            buttons = joystick_command.buttonValue;
            
            //std::cout << "Left: " << axis[LEFT][VERTICAL] << "," << axis[LEFT][HORIZONTAL] << ", right: " << axis[RIGHT][VERTICAL] << "," << axis[RIGHT][HORIZONTAL] << std::endl;
            
            motion_command.translation = axis[LEFT][VERTICAL];
            motion_command.rotation = axis[LEFT][HORIZONTAL];
            
            _motion_command.write(motion_command);
        }
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

