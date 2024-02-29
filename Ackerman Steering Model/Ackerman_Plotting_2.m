clc;
clear all
a=752; % a= CG distance from rear axle
l=1600; % l=wheelbase of car
w=1200-32; % w=trackwidth of car
r=2500:10:3500; % r= turning radius values
for i=1:length(r)
 Cotdi(i)=[(((r(i).^2)-(a^2))/(l^2)).^(0.5)]-(w/(2*l));
 di(i)=acot(Cotdi(i));
 did(i)=di(i)*(180/pi) % did= inner wheel turning angle in degrees
end
plot(r,did)
title('Inner wheel steering angle w.r.t. turning radius')
xlabel('Turning Radius (mm)')
ylabel('Angle on inner wheel (deg.)')
grid on