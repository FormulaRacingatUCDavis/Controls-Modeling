clc;clear;close all
%% Steer-Steer
Inside_Tire_Max = 27.5;
Inside_Tire_Min = -24.2;
Outside_Tire_Max = 24.2;
Outside_Tire_Min = -27.5;
Rack_displacement = 64.21;
static_toe = 0.5;
N = 200;
x_Rack_Displacement = linspace(-Rack_displacement/2,Rack_displacement/2,N);
x_Left_Tire = linspace(Inside_Tire_Min,Inside_Tire_Max,N);
x_Right_Tire = linspace(Outside_Tire_Min,Outside_Tire_Max,N);
Deviation = x_Left_Tire-x_Right_Tire;

%% Attempt 1
figure(1)  

yyaxis left
plot(x_Rack_Displacement,x_Left_Tire, 'b'); hold on;
plot(x_Rack_Displacement,x_Right_Tire, 'b--');
ylabel( 'Steer Angle' )

yyaxis right
plot(x_Rack_Displacement,Deviation,'r' ); hold on; 
ylabel( {'Linear Deviation ($\alpha_{I}$ - $\alpha_{O}$) [$deg$]'}, 'Interpreter', 'latex');
ylim( [-5 5] )
    
xlabel( 'Rack Displacement' )    
legend( {['Inside Tire Steer Angle'], ['Outside Tire Steer Angle']...
    ['Steer Angle Deviation']}, 'Interpreter', 'latex') 
title( 'Steer Steer Plot' );

%% Attempt 2
figure(2)  

plot(x_Rack_Displacement,x_Left_Tire, 'b'); hold on;
plot(x_Rack_Displacement,x_Right_Tire, 'b--');
ylabel( 'Steer Angle' )
    
xlabel( 'Rack Displacement' )    
legend( {['Inside Tire Steer Angle'], ['Outside Tire Steer Angle']}, 'Interpreter', 'latex') 
title( 'Steer Steer Plot' );

%% Steer - Steer Final
l=1550; %l= wheelbase of car
t=1220-32; %t= trackwidth-kingpin offset
d=69.78; %d= steering moment arm length
beta=14*pi/180; %beta= steering moment arm angle in degrees
did=0:0.1:27.5;
syms do real;
for i=1:length(did)
 di(i)=did(i)*(pi/180);
 eq(i)=(t-2*d*sin(beta))^2==(t-d*sin(beta+di(i))-...
     d*sin(beta-do))^2+(d*cos(beta-do)-d*cos(beta+di(i)))^2;
 dod(i)=vpasolve(eq(i),do);
end
dod=vpa(180*dod/pi);
x_Rack_Displacement_Positive = linspace(0,Rack_displacement/2,length(dod));
x_Rack_Displacement_Negative = linspace(-Rack_displacement/2,0,length(dod));
x_Rack_Displacement = [x_Rack_Displacement_Negative,x_Rack_Displacement_Positive];

flip_did = -flip(did);
flip_dod = -flip(dod);
Steer_Angle_Outside = [flip_dod,did];
Steer_Angle_Inside = [flip_did,dod];

Deviation = Steer_Angle_Inside-Steer_Angle_Outside;

Nominal_Deviation = zeros(length(x_Rack_Displacement));

figure(3)

yyaxis left
plot(x_Rack_Displacement,Steer_Angle_Inside, 'b'); hold on;
plot(x_Rack_Displacement,Steer_Angle_Outside, 'b--');
ylabel( 'Steer Angle' )

yyaxis right
plot(x_Rack_Displacement,Deviation,'r' ); hold on; 
plot(x_Rack_Displacement,Nominal_Deviation, 'r--');
ylabel( {'Linear Deviation ($\alpha_{I}$ - $\alpha_{O}$) [$deg$]'}, 'Interpreter', 'latex');
ylim( [-5 5] )
    
xlabel( 'Rack Displacement' )    
legend( {['Inside Tire Steer Angle'], ['Outside Tire Steer Angle']...
    ['Steer Angle Deviation'],['Nominal Deviation']}, 'Interpreter', 'latex') 
title( 'Steer Steer Plot' );