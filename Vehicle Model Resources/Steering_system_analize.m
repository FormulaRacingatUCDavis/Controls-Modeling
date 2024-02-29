clear all
close all
clc

global R1 J1 C1 G1 R2 J2 C2 rs mb R3 PI
PI=3.14159;
%% 

% steering rack information
rack_travel=4.5; % steering rack total travel (inch)
lock_lock=7/8; % steering lock to lock (rev)
rs=rack_travel/(lock_lock*2*PI);  % radius of steering pinion (inch)
%% 

% steering shaft information
L1=9; % length of shaft on driver's side (inch)
L2=16.5; %length of shaft on output side (inch)
D1=5/8; %diameter of shaft on driver's side (inch)
D2=5/8; %diameter of shaft on ouput side (inch)
G_steel=11.2*10^6; %Modulus of rigidity (psi)
mb=1.08; % mass of steering rack (lbf)
ds1=0.29; % density of shaft on driver's side; (lb/in^3)
ds2=0.29; % density of shaft on output side; (lb/in^3)
m1=ds1*PI/4*(D1^2)*L1; % mass of shaft on driver's side
m2=ds2*PI/4*(D2^2)*L2; % mass of shaft on output side

% steering system information
R1=1; % resistance of bearing#1
J1=0.5*m1*(D1^2)/4; % angular inertia of shaft on driver's side (lb-in^2)
C1=L1/J1/G_steel ; %c omplient of shaft on driver's side
G1=1; % Gear ratio
R2=1; % resistance of bearing#2 
J2=0.5*m2*(D2^2)/4; % angular inertia of shaft on output side (lb-in^2)
C2=L2/J2/G_steel; % complient of shaft on output side
R3=1; % resistance cause by friction between steering rack and pinion
%% 

initial=[0;0;0;0;0];
tspan=linspace(0,5,30);
[t,s]=ode45(@steering_torque_fun,tspan,initial);

H3=s(:,1); % angular momentum of shaft on driver's side
Q5=s(:,2); % deflection angle of shaft on driver's side
H9=s(:,3); % angular momentum of shaft on output side
Q11=s(:,4); % deflection angle of shaft on output side
P15=s(:,5); % momentum of steering rack

ext=zeros(length(t),8);
ds=zeros(length(t),5);

for i=1:length(t)
 [ds(i,:),ext(i,:)] = steering_torque_fun(t(i),s(i,:));
end

steering_torque=ext(:,1); % steering torque
steering_angular_velocity=ext(:,2); % steering angular velicity
steering_angular_acceleration=ext(:,3); % steering angular acceleration
Steering_rack_force=ext(:,4); % steering rack force
shaft_torque_driverside=ext(:,5); % shaft torque on driver's side
shaft_torque_output=ext(:,6); %shaf torque on output side
rack_vel=ext(:,7); % steering rack velocity
pinion_ang_vel=ext(:,8); % pinion angular velocity

close all
%% 

%figure('Name','Steering torque vs time','NumberTitle','off','Color','white')
%plot(t,steering_torque,'b','LineWidth',2);grid on
%title('Steering torque vs time')
%legend('Steering torque')
%ylabel('Torque (in-lb)')
%xlabel('Time (s)')

figure('Name','Steering rack force vs time','NumberTitle','off','Color','white')
plot(t,Steering_rack_force,'b','LineWidth',2);grid on
title('Steering rack force vs. time')
legend('Steering rack force')
ylabel('Force(lbf)')
xlabel('Time (s)')

figure('Name','steering accerleration vs. time','NumberTitle','off','Color','white')
plot(t,steering_angular_acceleration,'b','LineWidth',2);grid on
title('Steering angular acceleration vs. time')
legend('Steering angular acceleration')
ylabel('accerleration (rad/s^2)')
xlabel('Time (s)')

figure('Name','Torque on shaft','NumberTitle','off','Color','white')
plot(t,shaft_torque_driverside,'b',t,shaft_torque_output,'r','LineWidth',2);grid on
title('Torque on shaft')
legend('shaft on driver side','shaft on output side')
ylabel('Torque in-lb')
xlabel('Time (s)')

y1=t*0+0;
figure('Name','rack_vel','NumberTitle','off','Color','white')
plot(t,rack_vel,'LineWidth',2);grid on
hold;
plot(t,y1,'r');
title('rack velocity')
ylabel('velocity (inch/sec)')
xlabel('Time (s)')


figure('Name','steering angular velocity','NumberTitle','off','Color','white')
plot(t,steering_angular_velocity,'b','LineWidth',2);grid on
title('Steering angular velocity')
ylabel('angular velocity rad/s)')
xlabel('Time (s)')

%% 

%y1=t*0+2.5;
%y2=t*0+-2.5;
%figure('Name','Steering rack displacement','NumberTitle','off','Color','white')
%plot(t,P15,'b','LineWidth',2);
%hold; 
%plot(t,y1,'r');
%plot(t,y2,'r');
%title('Steering rack displacement')
%legend('Front suspension Displacement')
%ylabel('Displacement (inch)')
%xlabel('Time (s)')
