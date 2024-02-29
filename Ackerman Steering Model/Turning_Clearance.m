clc;
clear all
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
plot(did,dr)
title('Space required w.r.t. inner wheel steering angle')
xlabel('Inner wheel angle (deg.)')
ylabel('Turning space required (mm)')
grid on
