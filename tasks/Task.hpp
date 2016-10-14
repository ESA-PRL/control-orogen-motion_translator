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
    
    // Button names are in order in which they appear in the vector
    enum ButtonName
    {
        X,
        A,
        B,
        Y,
        LB,
        RB,
        LT,
        RT,
        BACK,
        START,
        LJOY,
        RJOY
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
        
        double axis_translation;
        double axis_rotation;
        bool pointTurn;
        double speedRatio;
        double speedRatioStep;
        double minSpeedPointTurn;
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

