clc; clear; close all;

set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

%% System Parameters
Pedal.Length = 3.77092954;
Pedal.Offset = 0.5;

Sensor.X = 68/25.4;
Sensor.Z = 15.125/25.4;
Sensor.Arm = 0.75;
Sensor.Offset=90;

Linkage.Length = norm([Pedal.Offset,Pedal.Length]-...
                        [Sensor.X,Sensor.Z+Sensor.Arm]);

%% Finding Pickup Point Through Sweep
Pedal.Angle = 90 : 0.05 : 112.5;

Pickup.X = sqrt(Pedal.Length.^2 + Pedal.Offset.^2) .* ...
    cosd( 180 - Pedal.Angle - atan2d( Pedal.Offset, Pedal.Length ) );

Pickup.Z = sqrt(Pedal.Length.^2 + Pedal.Offset.^2) .* ...
    sind( 180 - Pedal.Angle - atan2d( Pedal.Offset, Pedal.Length ) );

%% Determining Linkage & Key Position
Pickup.Distance = vecnorm( [Pickup.X; Pickup.Z] - [Sensor.X; Sensor.Z] );
Pickup.Angle = atan2d( Pickup.Z - Sensor.Z, Pickup.X - Sensor.X );

Linkage.Alpha = acos( (Linkage.Length^2 + Pickup.Distance.^2 - Sensor.Arm^2) ./ ...
    (2.*Linkage.Length.*Pickup.Distance) );
Linkage.Beta = acosd( (Sensor.Arm^2 + Pickup.Distance.^2 - Linkage.Length^2) ./ ...
    (2.*Sensor.Arm.*Pickup.Distance) );

Linkage.Gamma = 180 - (Linkage.Alpha + Linkage.Beta);

Sensor.Angle = Pickup.Angle - Linkage.Beta;

Sensor.Xp = Sensor.X + Sensor.Arm .* cosd(Sensor.Angle);
Sensor.Zp = Sensor.Z + Sensor.Arm .* sind(Sensor.Angle);

%% Plotting
figure
title( 'APPS Geometry Animation' );
surf( repmat( Pickup.X, 2, 1), ...
    repmat( Pickup.Z, 2, 1), ...
    Pedal.Angle .* ones( 2, 1 ), ...
    'EdgeColor', 'interp' );
hold on

surf( repmat( Sensor.Xp, 2, 1), ...
    repmat( Sensor.Zp, 2, 1), ...
    Pedal.Angle .* ones( 2, 1 ), ...
    'EdgeColor', 'interp' );

plot( Sensor.X, Sensor.Z, 'ko' );
plot( 0, 0, 'kx' );

axis equal
view( 0, 90 );

xlabel( 'Position ($X$) [$in$]' );
ylabel( 'Position ($Z$) [$in$]' );

ColorBar = colorbar;
ColorBar.Label.Interpreter = 'latex';
ColorBar.TickLabelInterpreter = 'latex';
ColorBar.Label.String = 'Pedal Angle, $\theta_{p}$ [$deg$]';

title( 'APPS Kinematics' )

figure
yyaxis left
plot( Pedal.Angle, 180-Sensor.Angle, 'b' );
hold on

Fit = fitlm( Pedal.Angle, Sensor.Angle );
plot( Pedal.Angle, feval(Fit, Pedal.Angle), 'b:' );

%plot( Pedal.Angle, 175*ones(size(Pedal.Angle)), 'b-.' );
%plot( Pedal.Angle, 95*ones(size(Pedal.Angle)), 'b-.' );

ylabel( 'Sensor Angle ($\theta_{S}$) [$deg$]' );
ylim( [90 200] );

yyaxis right
plot( Pedal.Angle, Sensor.Angle - feval(Fit, Pedal.Angle) );
hold on

plot( Pedal.Angle, zeros(size(Pedal.Angle)) );
ylabel( 'Linear Deviation ($\theta_{S} - L(\theta_{S})$) [$deg$]' );

xlabel( 'Pedal Angle ($\theta_{P}$) [$deg$]' );
title( 'Sensor to Pedal Angle' );

xlim( [Pedal.Angle(1), Pedal.Angle(end)] );

figure
plot( Pedal.Angle, (180-Sensor.Angle)/90 );
ylabel( 'Normalized Response [ ]' );

xlabel( 'Pedal Angle ($\theta_{P}$) [$deg$]' );
title( 'Uniform Response' );

xlim( [Pedal.Angle(1), Pedal.Angle(end)] );
