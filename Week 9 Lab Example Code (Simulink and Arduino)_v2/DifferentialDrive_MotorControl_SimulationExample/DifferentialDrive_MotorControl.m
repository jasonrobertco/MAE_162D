%{ 
Example of Differential Drive Trajectory and Motor Servo Control
Ts   % sampling invertval of driving update
Ts_vel  % sampling interval of motor servo control

Created 2026,  T-C. Tsao, UCLA Mechatronics and Controls Lab
%}


clear all
close all

%differential drive 
 rr=0.0254*3/2;  %wheel radius
 dd=14*0.0254;   %Wheel distance

MM=  [1/2 1/2; -1/dd 1/dd].*rr;

MMinv= [1 -dd/2; 1  dd/2]./rr;

Ts_vel=1/200;   %discrete sampling time for the differenatial drive and Motor PI control

%open DifferetialDrive.slx
%sim DifferetialDrive.slx
%pause;

%lumped motormodel: motor speed/duty cycle transfer function = ku/(tau *s+1)
ku1=10;
tau1=0.14;  
ku2=10;
tau2=0.14; 

% Motor PI control gains:
% Plant: 1+ku/(tau *s +1)
% Charactristic equation: 1+(kp+ki/s)*ku/(tau *s +1)=0
% Charactristic equation: s^2+ (1+ku*kp)/tau *s + ku*ki= s0

wn1= 2*pi*5;   %rad/sec desired closed loop natural frequency
zeta1=0.7;     % desired closed loop damping ratio

ki1_vel= wn1^2*tau1/ku1;  % PI control integral gain
kp1_vel= (2*zeta1*wn1*tau1-1)/ku1;   %PI control proportional gain

wn2= 2*pi*5;   %rad/sec desired closed loop natural frequency
zeta2=0.7;     % desired closed loop damping ratio

ki2_vel= wn2^2*tau2/ku2;  % PI control integral gain
kp2_vel= (2*zeta2*wn2*tau2-1)/ku2;   %PI control proportional gain

ki1_vel= 0.35;  
kp1_vel= 0.12;
ki2_vel= 0.35;  
kp2_vel= 0.12;

%open DifferetialDrive_MotorControl.slx
%sim DifferetialDrive_MotorControl.slx
%pause;

%Trajectory Proportional Feedback Control:

Ts=0.1;         % Sampling interval of the driving update

Kp_L =1;       %Longitudinal control:Proportional feedback gain of path length 
Kp_theta = 1;  %Lattidue control:Proportional feedback gain of path angle

MoveTime =5;   %Move Time duration for each segment

%open DifferetialDrive_Traj.slx
%sim DifferetialDrive_Traj.slx
%pause;
v_sat = 8; % inch/sec // TranslationalVelCmdSaturation
o_sat = 2; % rad/sec  // RotationalVelCmdSaturation

open ("DifferetialDrive_MotorControl_Traj.slx")
sim ("DifferetialDrive_MotorControl_Traj.slx")

% use this if you have Matlab 2024b
%open ("DifferetialDrive_MotorControl_Traj_2024b.slx")
%sim ("DifferetialDrive_MotorControl_Traj_2024b.slx")