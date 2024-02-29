clc; clear all; close all;

%% Outer vs Inner Tire Steering Angle

l=1600; % l=wheelbase of car
w=1200-32; % w=trackwidth of car
did=0:0.1:40; % did= inner wheel turning angle in degrees
for i=1:length(did)
 di(i)=did(i)*(pi/180);
 cotdo(i)=(w/l)+(cot(di(i)));
 do(i)=acot(cotdo(i));
 dod(i)=do(i)*(180/pi) % dod= outer wheel turning angle in degrees
end
figure(1)
plot(did,dod)
title('Outer wheel w.r.t. inner wheel on steering for Ackerman Steering')
xlabel('Angle on inner wheel (deg.)')
ylabel('Angle on outer wheel (deg.)')
grid on

%% Relation between inner wheel angles and turning radius in an Ackermann Geometry
clc; clear all;

a=752; % a= CG distance from rear axle
l=1600; % l=wheelbase of car
w=1200-32; % w=trackwidth of car
r=2500:10:3500; % r= turning radius values
for i=1:length(r)
 Cotdi(i)=[(((r(i).^2)-(a^2))/(l^2)).^(0.5)]-(w/(2*l));
 di(i)=acot(Cotdi(i));
 did(i)=di(i)*(180/pi) % did= inner wheel turning angle in degrees
end
figure(2)
plot(r,did)
title('Inner wheel steering angle w.r.t. turning radius')
xlabel('Turning Radius (mm)')
ylabel('Angle on inner wheel (deg.)')
grid on

%% Relation between outer wheel angles and turning radius in an Ackermann Geometry
clc;clear all;

a=752; % a= CG distance from rear axle
l=1600; % l=wheelbase of car
w=1200-32; % w=trackwidth of car
r=2500:10:3500; % r= turning radius values
for i=1:length(r)
 Cotdo(i)=[(((r(i).^2)-(a^2))/(l^2)).^(0.5)]+(w/(2*l));
 do(i)=acot(Cotdo(i));
 dod(i)=do(i)*(180/pi) % dod= outer wheel turning angle in degrees
end
figure(3)
plot(r,dod)
title('Outer wheel steering angle w.r.t. turning radius')
xlabel('Turning Radius (mm)')
ylabel('Angle on outer wheel (deg.)')
grid on

%% Clerance Required for a turn
clc;clear all;

g=855.1; % g= distance between front most part to front axle
l=1600; % l= wheelbase of car
w=1200-32; % w= trackwidth of car
did=0:0.01:40; % did= inner wheel turning angle in degrees
for i=1:length(did)
 di(i)=did(i)*(pi/180);
 x(i)= ((l/(tan(di(i))))+2*w) ;
 y(i) = l+g ;
dr(i) = ((x(i)).^2 + (y(i)).^2).^(0.5)-l/(tan(di(i))) % space required by vehicle make the turn
end
figure(4)
plot(did,dr)
title('Space required w.r.t. inner wheel steering angle')
xlabel('Inner wheel angle (deg.)')
ylabel('Turning space required (mm)')
grid on

%% Trapezoidal System
clc;clear all;

l=1600; %l= wheelbase of car
t=1200-32; %t= trackwidth-kingpin offset
d=116; %d= steering moment arm length
beta=30.25*pi/180; %beta= steering moment arm angle in degrees
did=0:0.1:40;
syms do real;
for i=1:length(did)
 di(i)=did(i)*(pi/180);
 eq(i)=(t-2*d*sin(beta))^2==(t-d*sin(beta+di(i))-...
     d*sin(beta-do))^2+(d*cos(beta-do)-d*cos(beta+di(i)))^2;
 dod(i)=vpasolve(eq(i),do);
end
dod=vpa(180*dod/pi)
figure(5)
plot(did,dod)
title('Outer wheel w.r.t. inner wheel on steering')
xlabel('Angle on inner wheel (deg.)')
ylabel('Angle on outer wheel (deg.)')
grid on

%% RMS Error Minimizer
clc;clear all;
xi=linspace(1,26,50); %xi= range of inner wheel angles
l=1.6;a=0.53*l;b=0.47*l;t=1.2-.032; %l= wheelbase of car in meters
d=0.116; %d= steering moment arm length
beta=linspace(26,36,10); %beta= steering moment arm angle
xo=zeros(length(beta),length(xi));
syms yo
for i=1:length(beta)
for j=1:length(xi)
xo(i,j)=acot((t/l)+cot(xi(j)*pi/180));
eq(i,j)=(t-2*d*sin(beta(i)*pi/180))^2==(t-d*sin((beta(i)+(xi(j)))*pi/180)-...
    d*sin(beta(i)*pi/180-yo))^2+(d*cos(beta(i)*pi/180-yo)-...
d*cos((beta(i)+xi(j))*pi/180))^2;
end
end
for i=1:length(beta)
for j=1:length(xi)
 y(i,j)=vpasolve(eq(i,j),yo);
end
end
for i=1:length(beta)
for j=1:length(xi)
 z(i,j)=(y(i,j)-xo(i,j)).^2;
end
end
w=(sum(z,2))/length(xi);
figure(6)
plot(beta,w');
title('Ackerman Model')
xlabel('Steering moment arm angle (mm)')
ylabel('RMS Error (mm)')
grid on
set(gca, 'YTick', 0:0.25*10^(-4):2.5*10^(-4))
