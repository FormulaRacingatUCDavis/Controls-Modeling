% Parameters
V = 5; % Voltage across pot, V
theta = 0; % Offset starting angle, degrees
pedalTravel = 30; % degrees
Rtot = 5000; % Ohms

% Angular range of rotation; add 25 degrees to enter electronic range
% (theta is an extra buffer); 30 degrees of pedal travel
alpha = (25+theta):(25+pedalTravel+theta);
travel = linspace(0,100,length(alpha));

% 9851
R2a = (Rtot/120)*alpha;
R1a = Rtot - R2a;
Vouta = V./(R1a.*((1./R1a)+(1./R2a)));

figure('Name','9851','NumberTitle','off','Color','white')
plot(alpha,Vouta), grid on
title('Voltage vs Angular Position')
ylabel('Voltage (V)')
xlabel('Angular Position (degrees)')

figure('Name','9851 Percent Travel','NumberTitle','off','Color','white')
plot(travel,Vouta), grid on
title('Voltage vs Percent Travel')
ylabel('Voltage (V)')
xlabel('Percent Travel')

% 9856
R2b = (Rtot/180)*alpha;
R1b = Rtot - R2b;
Voutb = V./(R1b.*((1./R1b)+(1./R2b)));

figure('Name','9856','NumberTitle','off','Color','white')
plot(alpha,Voutb), grid on
title('Voltage vs Angular Position')
ylabel('Voltage (V)')
xlabel('Angular Position (degrees)')

figure('Name','9856 Percent Travel','NumberTitle','off','Color','white')
plot(travel,Voutb), grid on
title('Voltage vs Percent Travel')
ylabel('Voltage (V)')
xlabel('Percent Travel')