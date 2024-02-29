%steering_torque_fun (function)
function [ds, ext] = steering_torque_fun(t,s)

global R1 J1 C1 G1 R2 J2 C2 rs mb R3 PI

% input information
Ft=170; % lateral force acting on steering rack (lbf)
T1=0.1; %response time of driver's input;
if t>T1
    Td=150;  % input torque from driver (lb-in)
else
    Td=0; 
end
    
% state equation
H3=s(1); % angular momentum of shaft on driver's side
Q5=s(2); % deflection angle of shaft on driver's side
H9=s(3); % angular momentum of shaft on output side
Q11=s(4); % deflection angle of shaft on output side
P15=s(5); % momentum of steering rack

% state equation -A metric
dH3=(-R1/J1)*H3-(1/C1)*Q5+Td; % torque on shaft of driver's side
dQ5=(1/J1)*H3-(1/J2/G1)*H9; % deflet angular velocity of shaft on driver's side
dH9=(G1/C1)*Q5-(R2/J2)*H9-(1/C2)*Q11; %Torque on output shaft
dQ11=(1/J2)*H9-(1/(rs*mb))*P15; % deflet angular velocity of output shaft
dP15=(1/(C2*rs))*Q11-(R3/mb)*P15-Ft; % force on steering rack


disp(t);
ds=[dH3;dQ5;dH9;dQ11;dP15];

steering_torque=dH3; % steering torque
steering_angular_velocity=H3/J1; % steering angular velicity
steering_angular_acceleration=dH3/J1; % steering angular acceleration
Steering_rack_force=dP15; % steering rack force
shaft_torque_driverside=Q5/C1; % shaft torque on driver's side
shaft_torque_output=Q11/C2; %shaf torque on output side
rack_vel=P15/mb; % steering rack velocity
pinion_ang_vel=rack_vel/rs; % pinion angular velocity

ext=[steering_torque;steering_angular_velocity;steering_angular_acceleration;Steering_rack_force;shaft_torque_driverside;shaft_torque_output;rack_vel;pinion_ang_vel];


