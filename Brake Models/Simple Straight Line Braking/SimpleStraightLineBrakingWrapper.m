clc; clear; close all;

%% Simple Braking & Traction Model

%% Braking Input 
Input.Fp = 100:100:1000; % Pedal Force [N]

%% Parameters
Parameter.Mass.m  = 270;          % Mass [kg]
Parameter.Mass.h  = 206.18./1000; % C.G. Height [mm -> m]
Parameter.Mass.pf = .48;           % Percent Front Weight Distribution [ ]

Parameter.Susp.L = 1.525; % Wheelbase [m]

Parameter.Wheel.J = 0.148; % Wheel Spin Inertia [kg-m^2]

Parameter.Brake.Db = [0.59; 0.8125] .* 0.0254; % Cylinder Bore Diameter [in -> m]
Parameter.Brake.mu = 0.55;                   % Pad Friction [ ]
Parameter.Brake.Ap = [2.9; 1.45] * 0.0254^2;        % Brake Pad Area [in^2 -> m^2]
Parameter.Brake.Rr = 3.3 * 0.0254;           % Rotor Radius [in -> m] 

Parameter.Pedal.eta = 5.7 ; % Pedal Ratio [ ]
Parameter.Pedal.pbb = .5;  % Balance Bar Setting [ ]

load('Hoosier_R25B_16x75-10x7.mat'); 
Parameter.Pacejka = Tire.Pacejka;
Tire.Pacejka.L.mu.x = 2/3;
Parameter.Pacejka.L.mu.x = 2/3;

%% Tire Calcs
Fidelity = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );

Fz0 = Parameter.Mass.m  * 9.81/4; % Nominal Normal Load [N]

Kxk = ( ContactPatchLoads( Tire, 0, 0.01, (Parameter.Mass.m .* 9.81 ./ 4) , 80, 0, 10, 1, Fidelity ) - ...
        ContactPatchLoads( Tire, 0, 0.00, (Parameter.Mass.m .* 9.81 ./ 4) , 80, 0, 10, 1, Fidelity ) ) ./ 0.01;
    % Slip Stiffness [N/[]]
    
Parameter.Re = Tire.Radius.Effective( Fz0, 80, 12 ) ./ 1000; % Nominal Effective Radius [m]

%% Initialization
x0(1) = 30; % Initial Speed [m/s]
x0(2:3) = x0(1) ./ Parameter.Re; % Initial Wheel Speed [rad/s]

%% Run Simulink Model
for i= 1 : numel( Input.Fp )
    Parameter.Fp = Input.Fp(i);
    
    Out(i) = sim( 'SimpleStraightLineBraking.slx' );
end

%% Plotting
figure
subplot(5,1,1)
for i = 1:numel(Input.Fp)
    scatter( Out(i).yout{7}.Values.Data(:,1), Out(i).yout{8}.Values.Data(:,1), 'b.' ); hold on;
    plot( -1:0.005:0, ContactPatchLoads( Tire, 0, -1:0.005:0, ...
        Out(i).yout{9}.Values.Data(end,1), 80, 0, 10, 1, Fidelity ), 'b' );

    scatter( Out(i).yout{7}.Values.Data(:,2), Out(i).yout{8}.Values.Data(:,2), 'r.' );
    plot( -1:0.005:0, ContactPatchLoads( Tire, 0, -1:0.005:0, ...
        Out(i).yout{9}.Values.Data(end,2), 80, 0, 10, 1, Fidelity ), 'r' );
end

xlabel( 'Slip Ratio [ ]' )
ylabel( 'Longitudinal Force [N]' )

subplot(5,1,2)
for i = 1:numel(Input.Fp)
    plot( Out(i).yout{2}.Values.Time, Out(i).yout{2}.Values.Data(:,1), 'b' ); hold on;
    plot( Out(i).yout{2}.Values.Time, Out(i).yout{2}.Values.Data(:,2), 'r' );
    plot( Out(i).tout([1 end]), 20*10^6*ones(2,1), 'k--' )
end

xlabel( 'Time [s]' )
ylabel( 'Line Pressure [Pa]' )

subplot(5,1,3)
for i = 1:numel(Input.Fp)
    plot( Out(i).yout{5}.Values.Time, Out(i).yout{5}.Values.Data(:,1) * 60/(2*pi), 'b' ); hold on;
    plot( Out(i).yout{5}.Values.Time, Out(i).yout{5}.Values.Data(:,2) * 60/(2*pi), 'r' );
end

xlabel( 'Time [s]' )
ylabel( 'Wheel Speed [rpm]' ); ylim([0, 1.05*x0(2)*60/(2*pi)]);

subplot(5,1,4)
for i = 1:numel(Input.Fp)
    plot( Out(i).yout{3}.Values.Time, Out(i).yout{3}.Values.Data, 'k' ); hold on;
end

xlabel( 'Time [s]' )
ylabel( 'Speed [m/s]' ); ylim([0, 1.05*x0(1)]);

subplot(5,1,5)
for i = 1:numel(Input.Fp)
    plot( Out(i).yout{4}.Values.Time, Out(i).yout{4}.Values.Data./9.81, 'k' ); hold on;
end

xlabel( 'Time [s]' )
ylabel( 'Acceleration [g]' )

figure
for i = 1:numel(Input.Fp); Acceleration(i) = Out(i).yout{4}.Values.Data(end); end
plot( Input.Fp, Acceleration / 9.81 )

xlabel( 'Pedal Force [N]' )
ylabel( 'Acceleration [g]' )