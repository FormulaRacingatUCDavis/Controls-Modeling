clc; clear; close all;

%% Simple Braking & Traction Model

%% Braking Input 
Input.Fp = 1000; % Pedal Force [N]

%% Parameters
Parameter.Mass.m  = 274;          % Mass [kg]
Parameter.Mass.h  = 10 .* 0.0254; % C.G. Height [in -> m]
Parameter.Mass.pf = .5;           % Percent Front Weight Distribution [ ]

Parameter.Susp.L = 1.525; % Wheelbase [m]

Parameter.Wheel.J = 0.148; % Wheel Spin Inertia [kg-m^2]

Parameter.Brake.Db = [0.59; 0.625] .* 0.0254; % Cylinder Bore Diameter [in -> m]
Parameter.Brake.mu = 0.55;                   % Pad Friction [ ]
Parameter.Brake.Ap = [2.9; 1.45] * 0.0254^2;        % Brake Pad Area [in^2 -> m^2]
Parameter.Brake.Rr = 3.3 * 0.0254;           % Rotor Radius [in -> m] 

Parameter.Pedal.eta = 4; % Pedal Ratio [ ]
Parameter.Pedal.pbb = .47;  % Balance Bar Setting [ ]

load('Hoosier_R25B_16x75-10x7.mat'); Parameter.Pacejka = Tire.Pacejka;
Tire.Pacejka.L.mu.x = 2/3;
Parameter.Pacejka.L.mu.x = 2/3;

%% Tire Calcs
Fidelity = struct( 'Pure', 'Pacejka', 'Combined', 'Pure' );

Fz0 = Parameter.Mass.m * 9.81/4; % Nominal Normal Load [N]

Kxk = ( Tire.ContactPatchLoads( 0, 0.01, Fz0, 80, 0, 10, 1, Fidelity ) - ...
        Tire.ContactPatchLoads( 0, 0.00, Fz0, 80, 0, 10, 1, Fidelity ) ) ./ 0.01;
    % Slip Stiffness [N/[]]
    
Parameter.Re = Tire.Radius.Effective( Fz0, 80 ) ./ 1000; % Nominal Effective Radius [m]

%% Initialization
x0(1) = 65; % Initial Speed [m/s]
x0(2:3) = x0(1) ./ Parameter.Re; % Initial Wheel Speed [rad/s]

%% Run Simulink Model
Out = sim( 'SimpleStraightLineBraking.slx' );

%% Plotting
figure
subplot(4,1,1)
scatter( Out.yout{7}.Values.Data(:,1), Out.yout{8}.Values.Data(:,1), 'b.' ); hold on;
plot( -1:0.005:0, Tire.ContactPatchLoads( 0, -1:0.005:0, ...
    Out.yout{9}.Values.Data(end,1), 80, 0, 10, 1, Fidelity ), 'b' );

scatter( Out.yout{7}.Values.Data(:,2), Out.yout{8}.Values.Data(:,2), 'r.' );
plot( -1:0.005:0, Tire.ContactPatchLoads( 0, -1:0.005:0, ...
    Out.yout{9}.Values.Data(end,2), 80, 0, 10, 1, Fidelity ), 'r' );

xlabel( 'Slip Ratio [ ]' )
ylabel( 'Longitudinal Force [N]' )

subplot(4,1,2)
plot( Out.yout{2}.Values.Time, Out.yout{2}.Values.Data(:,1), 'b' ); hold on;
plot( Out.yout{2}.Values.Time, Out.yout{2}.Values.Data(:,2), 'r' );
plot( Out.tout([1 end]), 20*10^6*ones(2,1), 'k--' )

xlabel( 'Time [s]' )
ylabel( 'Line Pressure [Pa]' )

subplot(4,1,3)
plot( Out.yout{5}.Values.Time, Out.yout{5}.Values.Data(:,1) * 60/(2*pi), 'b' ); hold on;
plot( Out.yout{5}.Values.Time, Out.yout{5}.Values.Data(:,2) * 60/(2*pi), 'r' );

xlabel( 'Time [s]' )
ylabel( 'Wheel Speed [rpm]' ); ylim([0, 1.05*x0(2)*60/(2*pi)]);

subplot(4,1,4)
plot( Out.yout{5}.Values.Time, Out.yout{5}.Values.Data, 'k' ); 

xlabel( 'Time [s]' )
ylabel( 'Speed [m/s]' ); ylim([0, 1.05*x0(1)]);

fprintf( ['Pedal Force Gradient: ', num2str( abs( Input.Fp ./ ...
    (Out.yout{4}.Values.Data(end) ./ 9.81) ) ), ' [N/g]\n'] )