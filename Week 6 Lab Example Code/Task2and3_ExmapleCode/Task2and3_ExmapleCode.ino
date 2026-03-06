#include "InterruptAndTimerSetup.h"
float MotorSpeedCalculationLowPassFactor = 0.5;
//////////////////////////////////////Parameters for DC Motor 1/////////////////////////////////////
#define Motor1_ENA_PIN 45
#define Motor1_IN1_PIN 30
#define Motor1_IN2_PIN 31
DC_Motor Motor1;
float M1_Kp = 0.2; float M1_Ki = 0.3; float M1_Kd = 0.00; float M1_Kff = 0.04;
// P gain, I gain, D gain, Feedfoward controller (p) gain
#define Encoder1_PhaseA_Pin 2 // must be 2, 3, 18, or 19
#define Encoder1_PhaseB_Pin 3 // must be 2, 3, 18, or 19
float Motor1_Cmd, Motor1Pos, LastMotor1Pos, Motor1Vel, DesiredMotor1Vel;        /////////////////////
//////////////////////////////////////Parameters for DC Motor 2 /////////////////////////////////////
#define Motor2_ENA_PIN 44
#define Motor2_IN1_PIN 33
#define Motor2_IN2_PIN 32
DC_Motor Motor2;
float M2_Kp = 0.2; float M2_Ki = 0; float M2_Kd = 0.00; float M2_Kff = 0.00;
// P gain, I gain, D gain, Feedfoward controller (p) gain
#define Encoder2_PhaseA_Pin 18 // must be 2, 3, 18, or 19
#define Encoder2_PhaseB_Pin 19 // must be 2, 3, 18, or 19
float Motor2_Cmd, Motor2Pos, LastMotor2Pos, Motor2Vel, DesiredMotor2Vel;/////////////////////////////
//////
float ISR1_dt = 0.01;  float ISR2_dt = 0.05; float ISR3_dt = 0.1;// sec
float RoverGlobalDirection = 0;
float RoverGlobalCoordsX = 0; float RoverGlobalCoordsY = 0;
float DesiredForwardSpeed = 0; // inch/sec
float DesiredRotationSpeed = 0; // rad/sec
float CircularMotionTimeToFinish, CalculatedCircularOmega;
float WheelDiameter = 3; // inch
float WheelDistance = 13; // inch
float DrawingCircleRadius = 15; // inch
float DrawingCircleSpeed = 4; // inch/sec
float InitialWait = 3;// sec  : How long to wait before the rover starts moving


void setup() {
  Serial.begin(115200);
  DC_Motor_Init(&Motor1, Motor1_IN1_PIN, Motor1_IN2_PIN, Motor1_ENA_PIN);
  Motor1_Cmd = 0;
  Encoder1ExtISRSetup(Encoder1_PhaseA_Pin, Encoder1_PhaseB_Pin, &Motor1.CurrentEncoderCount);// PA, PB, couter to be updated
  DC_Motor_Init(&Motor2, Motor2_IN1_PIN, Motor2_IN2_PIN, Motor2_ENA_PIN);
  Motor2_Cmd = 0;
  Encoder2ExtISRSetup(Encoder2_PhaseA_Pin, Encoder2_PhaseB_Pin, &Motor2.CurrentEncoderCount);// PA, PB, couter to be updated
  delay(100);
  HighPriorityInterruptInit(ISR1, 1/ISR1_dt); // Callback function, Triggering frequency between 31Hz - 2MHz
  MediumPriorityInterruptInit(ISR2, 1/ISR2_dt); // Callback function, Triggering frequency between 4Hz - 250kHz
  LowerPriorityInterruptInit(ISR3, 1/ISR3_dt); // Callback function, Triggering frequency between 0.24 Hz - 15625 Hz
  CalculatedCircularOmega = CircularMotionSteeringSpeed(DrawingCircleRadius, DrawingCircleSpeed);
}

void loop() { 
  
  Serial.print(DesiredMotor1Vel);
  Serial.print("\t");
  Serial.print(DesiredMotor2Vel);
  Serial.print("\t");
  
  Serial.print(RoverGlobalCoordsX);
  Serial.print("\t");
  Serial.print(RoverGlobalCoordsY);
  Serial.print("\n");
  delay(10);
  
  }

void ISR1(void){ // Higher Priority ISR
    Motor1Pos = DC_Motor_GetPos(&Motor1); // Get motor 1 angle in rad
    Motor2Pos = DC_Motor_GetPos(&Motor2); // Get motor 1 angle in rad
    CalculateRoverDynamics(Motor1Pos, Motor2Pos, WheelDistance, WheelDiameter);
    
    float DesiredRovTransVel = 0; // Desired Rover Translational Velocity
    float DesiredRovAngVel = 0; // Desired Rover angular (steering) Velocity (CCW as positive)
    if(millis() >= InitialWait*1000 && millis() < (InitialWait+CircularMotionTimeToFinish)*1000)
    { // Only feed in the command within the time frame
      DesiredRovTransVel = DrawingCircleSpeed; 
      DesiredRovAngVel = CalculatedCircularOmega;
    }
    
    //////////////////////////////////////////////////////////
    float WheelSpeed[2] = {0};
    RoverToMotorCmd(WheelSpeed, DesiredRovTransVel, DesiredRovAngVel, WheelDistance, WheelDiameter);
    DesiredMotor1Vel = WheelSpeed[0]; // Get the right wheel speed (motor 1)
    DesiredMotor2Vel = WheelSpeed[1]; // Get the left wheel speed (motor 2)
    Motor1SpeedControl(DesiredMotor1Vel); // Send speed to PID controller
    Motor2SpeedControl(DesiredMotor2Vel); // Send speed to PID controller
  }

void ISR2(void){  // Medium Priority ISR

  }
void ISR3(void){  // Lower Priority ISR

  }

void RoverToMotorCmd(float *ReturnedMotorSpeed, float TranslationalSpeed, float SteeringSpeed, float WheelDistance, float WheelDiameter)
{
  float MotorR_Speed, MotorL_Speed;
  // Use TranslationalSpeed, SteeringSpeed, WheelDistance, WheelDiameter to calculate each motor's required speed
  //================= Start your code here============================//
  
  // Calculate the radius of the wheel (r = D/2)
  float WheelRadius = WheelDiameter / 2.0;

  // --- Step 1 & 2: Component due to Translational Speed ---
  // If the robot only moved forward, both wheels would spin at this angular speed.
  // v = r * omega  ->  omega = v / r
  float WheelAngVelocity_Translation = TranslationalSpeed / WheelRadius;
  
  // --- Step 3 & 4: Component due to Steering Speed ---
  // To rotate, wheels must move in opposite directions.
  // The tangential velocity required at the wheel to support the rotation is:
  // v_tan = Omega_robot * (Distance_between_wheels / 2)
  float TangentialSpeed_Rotation = SteeringSpeed * (WheelDistance / 2.0);
  
  // Convert this tangential speed to wheel angular speed
  float WheelAngVelocity_Rotation = TangentialSpeed_Rotation / WheelRadius;

  // --- Step 5: Combine Components ---
  // Right Motor = Translation + Rotation (Right wheel spins faster to turn Left)
  // Left Motor  = Translation - Rotation
  MotorR_Speed = WheelAngVelocity_Translation + WheelAngVelocity_Rotation;
  MotorL_Speed = WheelAngVelocity_Translation - WheelAngVelocity_Rotation;
  
  //================= End your code here============================//
   ReturnedMotorSpeed[0] = MotorR_Speed; // send back calculated right motor speed
   ReturnedMotorSpeed[1] = MotorL_Speed; // send back calculated left motor speed
}

void CalculateRoverDynamics(float Motor1Pos, float Motor2Pos, float WheelDistance, float WheelDiameter)
{
  // Note that global variables are used here only for demostration
  // Later this will be changed to use local variable and pointer for better structure
  //Use these variables: 
  // Motor1Pos;LastMotor1Pos;Motor2Pos;LastMotor2Pos;WheelDistance;WheelDiameter;
  // RoverGlobalCoordsX;RoverGlobalCoordsY;RoverGlobalDirection;

  // Hints:
  // Step 1: Use Motor1Pos and LastMotor1Pos, Motor2Pos and LastMotor2Pos to find d_theta1, d_theta2
  float d_theta1 = Motor1Pos - LastMotor1Pos;
  float d_theta2 = Motor2Pos - LastMotor2Pos;
  
  // Step 2: Find d_l1 and d_l2 from d_theta1, d_theta2 with WheelDiameter
  // Arc Length formula: s = r * theta. Radius = Diameter / 2.
  float d_l1 = d_theta1 * (WheelDiameter / 2.0); 
  float d_l2 = d_theta2 * (WheelDiameter / 2.0);
  
  // Step 3: Find d_l from the average of d_l1 and d_l2
  // This represents the distance the center of the robot traveled.
  float d_l = (d_l1 + d_l2) / 2.0;
  
  // Step 4: Find d_alpha from d_l1 and d_l2 and WheelDistance
  // From your reference image: Dw * da = dl1 - dl2
  // Therefore: da = (dl1 - dl2) / Dw
  float d_alpha = (d_l1 - d_l2) / WheelDistance;
  
  // Step 5: Add d_alpha to the current RoverGlobalDirection to update the new direction
  RoverGlobalDirection = RoverGlobalDirection + d_alpha;
  
  // Step 6: Calculate d_x, d_y from d_l, cos(RoverGlobalDirection), sin(RoverGlobalDirection) 
  // We project the distance traveled (d_l) onto the X and Y axes using the new heading.
  float d_x = d_l * cos(RoverGlobalDirection);
  float d_y = d_l * sin(RoverGlobalDirection);
  
  // Step 7: update coords x and y by adding d_x, d_y to the current RoverGlobalCoordsX and Y
  RoverGlobalCoordsX = RoverGlobalCoordsX + d_x;
  RoverGlobalCoordsY = RoverGlobalCoordsY + d_y;
  
  // Step 8: Remember to update LastMotor1Pos and LastMotor2Pos with Motor1Pos and Motor2Pos
  LastMotor1Pos = Motor1Pos;
  LastMotor2Pos = Motor2Pos;
  
}

void Motor1SpeedControl(float DesiredSpeed)
{
  Motor1Vel = DC_Motor_GetVel(&Motor1, ISR1_dt, MotorSpeedCalculationLowPassFactor);
  Motor1_Cmd = 0;
  Motor1_Cmd += FF_Controller(DesiredSpeed, M1_Kff); // Replace this with Motor1_Cmd += PID_Controller(DesiredMotor1Vel, CurrentMotor1Vel);
  Motor1_Cmd += DC_PID_Controller(&Motor1, DesiredSpeed, Motor1Vel, ISR1_dt, M1_Kp, M1_Ki, M1_Kd);
  DC_Motor_Go(&Motor1, Motor1_Cmd);
}
void Motor2SpeedControl(float DesiredSpeed)
{
  Motor2Vel = DC_Motor_GetVel(&Motor2, ISR1_dt, MotorSpeedCalculationLowPassFactor);
  Motor2_Cmd = 0;
  Motor2_Cmd += FF_Controller(DesiredSpeed, M2_Kff); // Replace this with Motor1_Cmd += PID_Controller(DesiredMotor1Vel, CurrentMotor1Vel);
  Motor2_Cmd += DC_PID_Controller(&Motor2, DesiredSpeed, Motor2Vel, ISR1_dt, M2_Kp, M2_Ki, M2_Kd);
  DC_Motor_Go(&Motor2, Motor2_Cmd);
}


float CircularMotionSteeringSpeed(float CircleRadius, float DrawingSpeed)
{
  float PathLength = 2*CircleRadius*PI;
  CircularMotionTimeToFinish = PathLength/DrawingSpeed; // sec
  float SteeringSpeed = 2*PI/CircularMotionTimeToFinish;
  return SteeringSpeed;
}
