/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef MOTION_TRANSLATOR_TASK_TASK_HPP
#define MOTION_TRANSLATOR_TASK_TASK_HPP

#include "motion_translator/TaskBase.hpp"
#include "controldev/JoystickTaskBase.hpp"

namespace motion_translator
{
    enum JoystickName
    {
        LEFT,
        RIGHT
    };
    
    enum AxisDirection
    {
        VERTICAL,
        HORIZONTAL
    };
    
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        // Joystick current command input
        controldev::RawCommand joystick_command;
        // Motion command sent out
        base::MotionCommand2D motion_command;
        std::vector<std::vector<double> > axis;
        std::vector<uint8_t> buttons;
    public:
        Task(std::string const& name = "motion_translator::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
	    ~Task();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif

