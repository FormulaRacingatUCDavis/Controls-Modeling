clc;
clear all
l=1600; %l= wheelbase of car
t=1200-32; %t= trackwidth-kingpin offset
d=116; %d= steering moment arm length
beta=30.25*pi/180; %beta= steering moment arm angle in
degrees
did=0:0.1:40;
syms do real;
for i=1:length(did)
 di(i)=did(i)*(pi/180);
 eq(i)=(t-2*d*sin(beta))^2==(t-d*sin(beta+di(i))-
d*sin(beta-do))^2+(d*cos(beta-do)-d*cos(beta+di(i)))^2;
 dod(i)=vpasolve(eq(i),do);
end
dod=vpa(180*dod/pi)
plot(did,dod)
title('Outer wheel w.r.t. inner wheel on steering')
xlabel('Angle on inner wheel (deg.)')
ylabel('Angle on outer wheel (deg.)')
grid on