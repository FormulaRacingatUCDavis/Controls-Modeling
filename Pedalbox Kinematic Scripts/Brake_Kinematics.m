clc; clear; close all;

set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

%% System Parameters
Cylinder.X = 2.25 * 25.4; % X-Coord Master Cylinder Bearing Pivot Pickup [mm]
Cylinder.Z = 0 * 25.4; % Z-Coord Master Cylinder Bearing Pivot Pickup [mm] 

Cylinder.Angle = 69.9880839; % Initial Master Cylinder Angle with Horizontal [deg]

Cylinder.Length = 155.65 + 11; %Initial Length of Master Cylinder [mm]
Cylinder.Length(2) = Cylinder.Length - 30; %Final Length of Master Cylinder (Initial - Stroke) [mm] 

Pedal.Length = 9 * 25.4; % Pedal Length Between Pivot & Pedal Face [mm] 
Pedal.Angle = 90; % Initial pedal angle from -x axis [degrees]

%% Solving for MC Pickup Lengths and Pedal Offset Length [L_pp, L_po]
BalanceBar.Position = [Cylinder.X + Cylinder.Length(1)*cosd(180 - Cylinder.Angle); ...
    Cylinder.Z + Cylinder.Length(1)*sind(180 - Cylinder.Angle)]; %Vector Between Pedal Pivot and Balance Bar [mm]

Pedal.Direction = [cosd(180 - Pedal.Angle); sind( 180 - Pedal.Angle)]; %Pedal Direction (normal vector)

Pedal.Lever = dot(BalanceBar.Position, Pedal.Direction); % Length Along Pedal Centerline to Balance Bar [mm]

Pedal.Offset = cross([BalanceBar.Position; 0], [Pedal.Direction; 0]); % Length Off of Pedal Centerline to Balance Bar [mm]
Pedal.Offset = Pedal.Offset(3);

%% Solving for Final Pedal Displacement
Cylinder.Angle(2) = acosd( ((Cylinder.X^2 + Cylinder.Z^2) + Cylinder.Length(2)^2 - (Pedal.Lever^2 + Pedal.Offset^2)) ./ ...
    (2*sqrt(Cylinder.X^2 + Cylinder.Z^2)*Cylinder.Length(2)) ) - tand( Cylinder.Z ./ Cylinder.X ); 
    % Solving for Final Master Cylinder Theta [deg]
    
Pedal.Angle(2) = 180 - ( atan2d( (Cylinder.Z + Cylinder.Length(2)*sind(180 - Cylinder.Angle(2))), ...
    (Cylinder.X + Cylinder.Length(2)*cosd(180 - Cylinder.Angle(2))) ) + ...
    atand( Pedal.Offset ./ Pedal.Lever ) ); % Final Pedal Displacement [deg]

%% Sweeping Through Pedal Travel
Pedal.Angle = Pedal.Angle(1) : 0.1 : Pedal.Angle(2); % All Movement Based Values Now Linspaced Instead of Limits

BalanceBar.Position = [(Pedal.Lever.*cosd(180 - Pedal.Angle) + Pedal.Offset.*cosd(Pedal.Angle - 90)); ...
    Pedal.Lever*sind(180 - Pedal.Angle) - Pedal.Offset*sind(Pedal.Angle - 90)]; 

Cylinder.Length = sqrt( (BalanceBar.Position(1,:) - Cylinder.X).^2 + ...
    (BalanceBar.Position(2,:) - Cylinder.Z).^2 ); % Cylinder Length [mm]

Cylinder.Angle =  atan2d( (BalanceBar.Position(2,:) - Cylinder.Z), ...
    (Cylinder.X - BalanceBar.Position(1,:)) ); % Cylinder Angle [deg]

Post.MotionRatio = gradient( Cylinder.Length, Pedal.Angle ); % Motion Ratio (dL_c / dT_p) [mm/deg]

%% Finding Lever Arm Ratio
Cylinder.Direction  = [Cylinder.X - BalanceBar.Position(1,:); ...
    Cylinder.Z - BalanceBar.Position(2,:); zeros( size(Pedal.Angle) )];

Cylinder.Direction = Cylinder.Direction ./ vecnorm( Cylinder.Direction);

Cylinder.Lever = cross( -[BalanceBar.Position; zeros( size(Pedal.Angle) )], ...
    Cylinder.Direction );

Cylinder.Lever = Cylinder.Lever(3,:); % Normal Distance Between Pedal Pivot & Master Cylinder Plane [mm]

Post.PedalRatio = (Pedal.Length.*sind(Pedal.Angle)) ./ Cylinder.Lever; % Lever Ratio []

%% Determining Reaction Forces
Pedal.Input = 2000; %Pedal Input Force [N]

Cylinder.Force = Pedal.Input .* Post.PedalRatio;

Cylinder.Reactions = [ Cylinder.Force .* cosd( Cylinder.Angle ); ...
                       Cylinder.Force .* sind( Cylinder.Angle ) ] ;

Pedal.Reactions = [ Cylinder.Reactions(1,:) - Pedal.Input; ...
                    -Cylinder.Reactions(2,:) ];
      
[MaxNum, MaxIdx] = max( abs( [ Cylinder.Reactions(:,Pedal.Angle < 100); ...
                          Pedal.Reactions(:,Pedal.Angle < 100) ] ), [], 2 );

Post.Reactions = [ Cylinder.Reactions(:, Pedal.Angle < 100); ...
                   Pedal.Reactions(:, Pedal.Angle < 100) ];

for i = 1 : 4
    Post.Reactions(:,i) = Post.Reactions( :, MaxIdx(i) );
end

Post.Reactions(:,5:end) = [];

%% Plotting
figure
sgtitle( 'Brake Geometry' )

subplot( 2, 1, 1 )
plot(Pedal.Angle, Cylinder.Length)

xlabel( 'Pedal Angle, $\theta_{p}$ [$deg$]' )
ylabel( 'Cylinder Length, $L_{c}$ [$mm$]' )

subplot( 2, 1, 2 )
plot(Pedal.Angle, Cylinder.Angle)

xlabel( 'Pedal Angle, $\theta_{p}$ [$deg$]' )
ylabel( 'Cylinder Angle, $\theta_{c}$ [$deg$]' )

figure
title( 'Brake Geometry Animation' )
surf( repmat( BalanceBar.Position(1,:), 2, 1), ...
    repmat( BalanceBar.Position(2,:), 2, 1), ...
    Pedal.Angle .* ones( 2, 1 ), ...
    'EdgeColor', 'interp' )
hold on

surf( repmat( Pedal.Length.*cosd(180 - Pedal.Angle), 2, 1), ...
    repmat( Pedal.Length.*sind(180 - Pedal.Angle), 2, 1), ...
    Pedal.Angle .* ones( 2, 1 ), ...
    'EdgeColor', 'interp' )

plot( Cylinder.X, Cylinder.Z, 'ko' )
plot( 0, 0, 'kx' )

axis equal
view( 0, 90 )

xlabel( 'Position (X) [$mm$]' )
ylabel( 'Position (Z) [$mm$]' )

ColorBar = colorbar;
ColorBar.Label.Interpreter = 'latex';
ColorBar.TickLabelInterpreter = 'latex';
ColorBar.Label.String = 'Pedal Angle, $\theta_{p}$ [$deg$]';

figure
sgtitle( 'Kinematic Advantages' )
subplot( 2, 1, 1 )
plot(Pedal.Angle, Post.MotionRatio)

xlabel( 'Pedal Angle, $\theta_{p}$ [$deg$]' )
ylabel( 'Motion Ratio, $\frac{dL_{c}}{d\theta_{p}}$ [$\frac{mm}{deg}$]' )

subplot( 2, 1, 2 )
plot(Pedal.Angle, Post.PedalRatio)

hold on
plot( Pedal.Angle([1 end]), 4.1.*ones(2,1), 'k-.' )
plot( Pedal.Angle([1 end]), 4.3.*ones(2,1), 'k-.' )

ylim( [0 4.5] )
xlabel( 'Pedal Angle, $\theta_{p}$ [$deg$]' )
ylabel( 'Pedal Ratio, [ ]' )

figure

plot( Pedal.Angle, -Cylinder.Reactions(1,:), 'b-' )
hold on
plot( Pedal.Angle, Cylinder.Reactions(2,:), 'b--' )
plot( Pedal.Angle, Pedal.Reactions(1,:), 'r-' )
plot( Pedal.Angle, Pedal.Reactions(2,:), 'r--' )

title( 'Quasistatic Platform Reactions' )

legend( {'Master Cylinder, X', 'Master Cylinder, Z', ...
    'Pedal Pivot, X', 'Pedal.Pivot, Z'} )
ylabel( 'Reaction Force, [N]')
xlabel( 'Pedal Angle, [deg]' )

Axes = gca;
Axes.YRuler.Exponent = 0;

clear ColorBar i MaxNum MaxIdx