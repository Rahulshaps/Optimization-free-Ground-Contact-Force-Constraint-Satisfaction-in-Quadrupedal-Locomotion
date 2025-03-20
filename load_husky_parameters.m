%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Load the HROM parameters into the data struct 'p'.
% Parameters include the controller and ground model parameters
% 
% Authors:  Eric Nauli Sihite
%           Andrew Lessieur
% 
% Date: 5/29/2020
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Symbolic Model Parameters

p.g = 9.8; % Gravity constant

if p.use_husky == 1
    % Body mass* (used in simulation, estimated from real robot) = 2.2337 kg
    p.mb = 4.4; %2.2337; % kg
    
    % Body inertia* = diag([0.0030709, 0.0046624, 0.0035462])
    % IB = [X Y Z]
    p.Ib_1 = 0.0030709;
    p.Ib_2 = 0.0046624;
    p.Ib_3 = 0.0035462;
        
    % COM coordinates* (x,y,z) relative to body center
    if (p.use_centered_mass)
      p.d_body_com = [0;0;0]; % Centered CoM vs the hips position
    else
      p.d_body_com = [0.021107; 0; -0.041523];
    end
    
    % Frontal hip joint location (x,y,z) relative to body center in the following order: FR, FL, BR, BL = 
    % [0.19745, -0.045, 0,      0.19745, 0.045, 0,      -0.19745, -0.045, 0,      -0.19745, 0.045, 0] m
    p.lh_FR = [  0.19745, -0.045, 0]' - p.d_body_com;
    p.lh_FL = [  0.19745,  0.045, 0]' - p.d_body_com;
    p.lh_HR = [ -0.19745, -0.045, 0]' - p.d_body_com;
    p.lh_HL = [ -0.19745,  0.045, 0]' - p.d_body_com;
else
    p.d_body_com = [0;0;0];
    p.mb = 4.4;
    w=0.2;
    h=0.2;
    l=0.4;
    % Inertia model estimate (box)
    p.Ib_1 = p.mb/12*(w^2 + h^2);
    p.Ib_2 = p.mb/12*(l^2 + h^2);
    p.Ib_3 = p.mb/12*(l^2 + w^2);

    % Length vectors from body CoM to hips in body frame
    % x = forward, y = left, z = up

    p.dx = 0;

    p.lh_FR = [  0.5*l - p.dx;  -0.5*w;    0];
    p.lh_FL = [  0.5*l - p.dx;   0.5*w;    0];
    p.lh_HR = [ -0.5*l - p.dx;  -0.5*w;    0];
    p.lh_HL = [ -0.5*l - p.dx;   0.5*w;    0];
end

% Ground position in inertial frame
p.ground_z = 0;

%% Other Model Parameters

% initial states

p.leg_init_height = 0.5; % initial leg height




% Compliant ground model ==================================================


p.kp_g = 8000;                  % 2000; %  Spring coefficient
p.kd_g = sqrt(p.kp_g)*2*1.5;    % 200 Damping coefficient
% p.kp_g = 3000;                  % 2000; %  Spring coefficient
% p.kd_g = sqrt(p.kp_g)*2*1.5;    % 200 Damping coefficient

% Ground friction model ===================================================


% Basic friction model parameters
p.kfs = 0.6;   % Static friction coefficient
p.kfc = p.kfs*0.9;    % Coulomb friction coefficient
p.kfb = 0.85;   % Viscous friction coefficient

% LuGre model parameters
p.s0 = 100;                 % LuGre bristle stiffness
p.s1 = sqrt(p.s0)*2*0.5;    % LuGre bristle damping
p.gv_min = 1e-3;    % static/coulomb friction threshold for no contact assumption
p.zd_max = 100;    % z decay rate during no contact
p.vs = 1e-3;        % Stribeck velocity (m/s)


% Observer model parameters ===============================================


p.K_mbdo = diag([1, 1, 1, 1, 1, 1])*200;    % Observer gain


% Joint controller ========================================================


% PD controller gains
p.Kd = 2500;            % Proportional gain
p.Kp = (p.Kd/2)^2;      % Derivative gain


% Compliant leg parameters ================================================


% Thin tube 2nd moment of area
p.R1 = 0.005; % Leg beam inner radius (m), assuming a thin cylindrical rod
p.R2 = 0.004; % Leg beam outer radius (m), assuming a thin cylindrical rod
p.I = (p.R2^4 - p.R1^4)*pi/2;   

% p.E = 95e9;   % Average carbon fiber modulus of elasticity (Pa)
p.E = 1e9;   % Modulus of elasticity (Pa)

% Set to zero to transform this model into HROM rigid
p.kc_p = 1000*1*0;  % Spring constant of the deflection model
p.kc_d = sqrt(p.kc_p)*2*1.4;   % Damping constant of the deflection model


% Touchdown estimation ====================================================


% Probabilistic model, phase based, contact. [FR, FL, HR, HL]
p.mean_ph_c0 = [0.5;0;0;0.5];   % phase at the beginning of contact
p.mean_ph_c1 = [1;0.5;0.5;1];   % phase at the end of contact
p.std_ph_c0 = 0.04;             % Standard deviation
p.std_ph_c1 = 0.04;

% Probabilistic model, phase based, no contact (swing)
p.mean_ph_nc0 = [0;0.5;0.5;0];    % phase at the beginning of swing
p.mean_ph_nc1 = [0.5;1;1;0.5];    % phase at the end of swing
p.std_ph_nc0 = 0.04;              % Standard deviation
p.std_ph_nc1 = 0.04;

% Probabilistic model, gcf based (z direction only)
p.mean_gcf = ones(4,1)*10;    % About 10 N for all legs when idling
p.std_gcf = 1;

% Probabilistic model, foot position based (z direction only)
p.mean_pz = [5;5;5;5];
p.std_pz = 1;













