clc
clear all
close all
xi=linspace(1,26,50); %xi= range of inner wheel angles
l=1.6;a=0.53*l;b=0.47*l;t=1.2-.032; %l= wheelbase of car
in meters
d=0.116; %d= steering moment arm length
beta=linspace(26,36,10); %beta= steering moment arm
angle
xo=zeros(length(beta),length(xi));
syms yo
for i=1:length(beta)
for j=1:length(xi)
xo(i,j)=acot((t/l)+cot(xi(j)*pi/180));
eq(i,j)=(t-2*d*sin(beta(i)*pi/180))^2==(td*sin((beta(i)+(xi(j)))*pi/180)-d*sin(beta(i)*pi/180-
yo))^2+(d*cos(beta(i)*pi/180-yo)-
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
plot(beta,w');
title('Ackerman Model')
xlabel('Steering moment arm angle (mm)')
ylabel('RMS Error (mm)')
grid on
set(gca, 'YTick', 0:0.25*10^(-4):2.5*10^(-4))