function [footPose, tau2] = anklePulleyDynamics(beta, beta_dot, beta_ddot, R1, R2, tau1, geometryParams)
% Description:
%   Computes the foot pose and output torque for an artificial ankle joint
%   with belt transmission based on the given parameters.
%
% Inputs:
%   beta:         Driving angle motion of the input pulley (radians)
%   beta_dot:     Angular velocity of the input pulley (rad/s)
%   beta_ddot:    Angular acceleration of the input pulley (rad/s^2)
%   R1:           Radius of the input pulley (m)
%   R2:           Radius of the output pulley (m)
%   tau1:         Input torque on the input pulley (Nm)
%   geometryParams: Structure containing design parameters:
%       geometryParams.phi: Pitch angle of line linking rotation centers (radians)
%       geometryParams.delta: Central angle of tangent line (radians)
%       geometryParams.d: Relative distance between parallel axes of the pulley set (m)
%
% Outputs:
%   footPose:     Foot pose as [alpha, alpha_dot, alpha_ddot] (radians, rad/s, rad/s^2)
%   tau2:         Output torque on the output pulley (Nm)

%% Extract geometry parameters
phi = geometryParams.phi;
delta = geometryParams.delta;
d = geometryParams.d;

%% Angular Motion of Output Pulley
alpha = (R1 / R2) * beta;          % Output pulley angle (radians)
alpha_dot = (R1 / R2) * beta_dot;  % Angular velocity (rad/s)
alpha_ddot = (R1 / R2) * beta_ddot; % Angular acceleration (rad/s^2)

%% Output Torque on Output Pulley
tau2 = tau1 * (R1 / R2);           % Torque transfer based on pulley radii

%% Foot Pose Calculation (Optional Extension)
% Foot pose might depend on angular motion (alpha), pitch (phi), and geometry.
xF = d * cos(phi) + R2 * cos(alpha + delta); % X-coordinate of foot
yF = d * sin(phi) + R2 * sin(alpha + delta); % Y-coordinate of foot
angF = alpha;                               % Foot angle with horizontal

footPose = [xF, yF, angF];
end
