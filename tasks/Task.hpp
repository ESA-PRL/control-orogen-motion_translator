#ifndef MOTION_TRANSLATOR_TASK_TASK_HPP
#define MOTION_TRANSLATOR_TASK_TASK_HPP

#include "motion_translator/TaskBase.hpp"
#include "controldev/JoystickTaskBase.hpp"
#include <base/commands/Joints.hpp>
#include "base/Angle.hpp"
#include <base-logging/Logging.hpp>

namespace motion_translator
{
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

        // Local copy of incoming axis and button data from the joystick
        controldev::RawCommand joystick_command_prev;
        //std::vector<double> axis;
        //std::vector<uint8_t> buttons;

        // Motion command sent to motors, contains translation and rotation speeds
        base::commands::Motion2D motion_command;

        // PTU variables
        double ptu_pan_angle;
        double ptu_tilt_angle;
        double ptu_maxSpeed;
        base::commands::Joints ptu_command;

        // PTU movement limits
        double ptu_maxPanAngle;
        double ptu_minPanAngle;
        double ptu_maxTiltAngle;
        double ptu_minTiltAngle;

        // Joystick axis values
        double axis_translation;
        double axis_rotation;
        double axis_heading;
        double axis_pan;
        double axis_tilt;
        bool sent_zero;

        locomotion_switcher::LocomotionMode locomotion_mode;

        // Locomotion related parameters
        bool pointTurn;
        bool genericCrab;
        bool crab;

        double speedRatioStep;
        double speedRatio;
        double angularSpeedRatio;
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
