
// This is a remake of the TrapezoidalPlanner class for the SimpleFOC library. This iteration calculates the motion in a S-curve using sinusoidal calculations (1.0 - cos(t)). 
// The S-curve is a smoother motion profile than the trapez, since the acceleration and deceleration are not linear, but curved.
// Furthermore, the goal is to implement the planner with a circular buffer, so that the planner can be fed commands from a serial port,
// and the planner will execute the commands in the order they were received. Therefore the planner will be able to execute varius G and M commands.
// In order to use it with the SimpleFOC library, the planner will need to be created and linked in the main.cpp file, and the runPlannerOnTick() function will need to be called in the main loop.
// The commander is obviosly also needed. 
//
// We need to create a circularBuffer class in the same lib folder as the planner is #include "../CircularBuffer/CircularBuffer.h" 
// NOTE: The circular buffer class was a colaboration with GPTchat.
//

// TrapezoidalPlanner planner(5);
// 
//
// Further more we need to add the following to the main.cpp.
/*

void doPlanner(char *cmd){
  planner.doGcommandBuffer(cmd);
}

void MPlanner(char *cmd){
  planner.doMCommand(cmd);
}

//And in the void setup() {
 
 ....

      planner.linkMotor(&motor);
  commander.add('G', doPlanner, "Motion Planner");
  commander.add('M', MPlanner, "Motion Planner");

 ....
  
  }

  Here is a example of a loop that will run the planner on every tick.

    // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  //motor.monitor();
  
  commander.run();
  planner.runPlannerOnTick();


*/

#ifndef __TRAPEZOIDAL_PLANNER__H
#define __TRAPEZOIDAL_PLANNER__H
#include <SimpleFOC.h>
#include "../CircularBuffer/CircularBuffer.h"

class TrapezoidalPlanner
{
public:
    TrapezoidalPlanner(int);
    void doTrapezoidalPlannerCommand(char *command, char *command2);
    void doGcommandBuffer(char *command);
    void doMCommand(char *command);
    void linkMotor(StepperMotor*);
    void runPlannerOnTick();
    bool isPlannerMoving();
private:
 
    
    StepperMotor* motor;
    unsigned long plannerTimeStap;
    int plannerPeriod = 0.5; // 1000 / this number = Hz, i.e. 1000 / 100 = 10Hz, 1000 / 10 = 100Hz, 1000 / 5 = 200Hz, 1000 / 1 = 1000hZ
    float Vmax_ = 30.0f;    // # Velocity max (rads/s)
    float Amax_ = 20.0f;    // # Acceleration max (rads/s/s)
    float Dmax_ = 20.0f;    // # Decelerations max (rads/s/s)

    //M400 commands tells the planner to wait until all moves are completed before proceeding.
    bool m400_flag = false;

    float Y_;
    float Yd_;
    float Ydd_;
    float Tf_, Xi_, Xf_, Vi_, Ar_, Dr_, Vr_, Ta_, Td_, Tv_, yAccel_;
    unsigned long plannerStartingMovementTimeStamp;
    bool isTrajectoryExecuting;
    float sign(float val);
    float sign_hard(float val);
    bool calculateTrapezoidalPathParameters(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax);

    void startExecutionOfPlannerTo(float newPos);
    void computeStepValuesForCurrentTime(float currentTrajectoryTime);
    char* tailItem;
    char* nextItem;
    char* command_char;
    //CircuarBuffer holder for commands, set desired size.
    CircularBuffer buffer = CircularBuffer(100);

     // Additional setpoints for trajectory
    float dXmin;

    // COSINE PLANNER VARIABLES
    float t;
    float desiredAmplitude;
};

#endif