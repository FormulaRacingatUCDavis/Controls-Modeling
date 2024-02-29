clc;
clear all
a=752; % a= CG distance from rear axle
l=1600; % l=wheelbase of car
w=1200-32; % w=trackwidth of car
r=2500:10:3500; % r= turning radius values
for i=1:length(r)
 Cotdo(i)=[(((r(i).^2)-(a^2))/(l^2)).^(0.5)]+(w/(2*l));
 do(i)=acot(Cotdo(i));
 dod(i)=do(i)*(180/pi) % dod= outer wheel turning angle in degrees
end
plot(r,dod)
title('Outer wheel steering angle w.r.t. turning radius')
xlabel('Turning Radius (mm)')
ylabel('Angle on outer wheel (deg.)')
grid on
