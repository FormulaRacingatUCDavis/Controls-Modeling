function [outputArg1,outputArg2] = BrakeModel(PedalForce)
%% TireContactPatchLoads - Evaluates Model for F_x, F_y, M_z, M_x, and M_y
% Computes tire loads for a given operating condition and model fidelty. 
%
% Inputs:
%   Tire     - FRUCD Tire Data    [struct]
%   Alpha    - Slip Angle         [deg]
%   Kappa    - Slip Ratio         [ ]
%   Fz       - Normal Load        [N]
%   Pi       - Inflation Pressure [kPa]
%   Inc      - Inclination Angle  [deg]
%   Vc       - Tire Velocity      [m/s]
%   Idx      - Tire Index         [ ]
%   Model    - Model Selection    [struct]
%       .Pure     : Pure Slip Model     {'Linear', 'Pacejka'}
%       .Combined : Combined Slip Model {'Pure'  , 'MNC'    }
% 
% Outputs:
%  Fx - Longitudinal Force [N]
%  Fy - Lateral Force      [N]
%  Mz - Aligning Moment    [Nm]
%  Mx - Overturning Moment [Nm]
%  My - Rolling Resistance [Nm]
%
% Author(s): 
% Blake Christierson (bechristierson@ucdavis.edu) [Sep 2018 - Jun 2021] 
% 
% Last Updated: 4-Mar-2021

outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

