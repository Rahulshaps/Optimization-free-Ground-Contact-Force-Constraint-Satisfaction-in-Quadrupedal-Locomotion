% Husky Reduced Order Model, Rigid legs

close all, clear all, clc

%% Symbolic parameters

% Constants ===============================================================

syms mb g Ib_1 Ib_2 Ib_3 t ground_z real

% Length from body CoM to the hip (body frame)
l_hip_FR = sym('lh_FR_', [3,1], 'real');
l_hip_FL = sym('lh_FL_', [3,1], 'real');
l_hip_HR = sym('lh_HR_', [3,1], 'real');
l_hip_HL = sym('lh_HL_', [3,1], 'real');

I_body = diag([Ib_1, Ib_2, Ib_3]); % Body inertia matrix

% Time varying variables ==================================================

% Body variables
x_body = sym('xb_', [3,1], 'real');   % Body linear position
v_body = sym('vb_', [3,1], 'real');   % Body linear velocity

R_body = sym('Rb_', [3,3], 'real');   % Body rotation matrix
w_body = sym('wb_', [3,1], 'real');   % Body angular velocity (body frame)

vD_body = sym('vbD_', [3,1], 'real');   % body linear acceleration
wD_body = sym('wbD_', [3,1], 'real');   % body angular acceleration

% Leg variables, id = [FR, FL, HR, HL]
theta_f = sym('qf_', [4,1], 'real');  % Frontal angles
theta_s = sym('qs_', [4,1], 'real');  % Sagittal angles
l_pris  = sym('ql_', [4,1], 'real');  % Prismatic joint lengths

thetaD_f = sym('qfD_', [4,1], 'real');  % Frontal angles velocity
thetaD_s = sym('qsD_', [4,1], 'real');  % Sagittal angles velocity
lD_pris  = sym('qlD_', [4,1], 'real');  % Prismatic joint lengths velocity

% Compliant leg variables
l_comp_x = sym('qcx_', [4,1], 'real');  % Leg compliance x direction
l_comp_y = sym('qcy_', [4,1], 'real');  % Leg compliance y direction
l_comp_xD = sym('qcxD_', [4,1], 'real');  % Leg compliance x direction rate of change
l_comp_yD = sym('qcyD_', [4,1], 'real');  % Leg compliance y direction rate of change

% Input vectors ===========================================================

accel_joints = sym('ua_', [12,1], 'real'); % joints accelerations
u_external = sym('ue_', [6,1], 'real'); % external forces acting on body (force, torque)
u_ground_ext = sym('ug_', [12,1], 'real'); % ground forces (non-constraint, any directions, each 3D linear forces)
u_ground_con = sym('ugc_', [4,1], 'real'); % constraint ground forces (inertial z)

% foot accel
dp_i = sym('dpi_', [3,1], 'real');
dp_b = sym('dpb_', [3,1], 'real');
dpD_i = sym('dpiD_', [3,1], 'real');
dpD_b = sym('dpbD_', [3,1], 'real');
dpDD_i = sym('dpiDD_', [3,1], 'real');
dpDD_b = sym('dpbDD_', [3,1], 'real');

%% Kinematic Formulation

% Rotation matrix rate of change, defined in SO(3)
RD_body = R_body*skew(w_body);

% Prepare substitution elements for symbollic time differentiation
qq = [x_body; v_body; w_body; theta_f; theta_s; l_pris; R_body(:);...
        l_comp_x; l_comp_y;dp_i;dpD_i;dp_b;dpD_b];
qqd = [v_body; vD_body; wD_body; thetaD_f; thetaD_s; lD_pris;  ...
        RD_body(:); l_comp_xD; l_comp_yD;dpD_i;dpDD_i;dpD_b;dpDD_b];
Nq = length(qq);
    
syms x real
ft = symfun('x(t)',[x,t]);
xx0 = sym('x_', [Nq,1], 'real');
xx = xx0;
xxd = xx;
for i = 1:Nq
  xx(i) = ft(xx0(i),t);
  xxd(i) = diff(xx(i),t);
end

% Leg rotation matrices (body -> leg frame)
R_FR = rot_x(theta_f(1))*rot_y(theta_s(1));
R_FL = rot_x(theta_f(2))*rot_y(theta_s(2));
R_HR = rot_x(theta_f(3))*rot_y(theta_s(3));
R_HL = rot_x(theta_f(4))*rot_y(theta_s(4));

% Feet linear positions (inertial)
pos_feet_FR = x_body + R_body*(l_hip_FR + R_FR*[l_comp_x(1);l_comp_y(1);-l_pris(1)]);
pos_feet_FL = x_body + R_body*(l_hip_FL + R_FL*[l_comp_x(2);l_comp_y(2);-l_pris(2)]);
pos_feet_HR = x_body + R_body*(l_hip_HR + R_HR*[l_comp_x(3);l_comp_y(3);-l_pris(3)]);
pos_feet_HL = x_body + R_body*(l_hip_HL + R_HL*[l_comp_x(4);l_comp_y(4);-l_pris(4)]);


% Feet linear velocities (inertial)
vel_feet_FR = time_derivative(pos_feet_FR,qq,qqd,xx,xxd,t);
vel_feet_FL = time_derivative(pos_feet_FL,qq,qqd,xx,xxd,t);
vel_feet_HR = time_derivative(pos_feet_HR,qq,qqd,xx,xxd,t);
vel_feet_HL = time_derivative(pos_feet_HL,qq,qqd,xx,xxd,t);

% Leg angular velocity (body frame)
angvel_FR = [thetaD_f(1);0;0] + rot_x(theta_f(1))*[0;thetaD_s(1);0];
angvel_FL = [thetaD_f(2);0;0] + rot_x(theta_f(2))*[0;thetaD_s(2);0];
angvel_HR = [thetaD_f(3);0;0] + rot_x(theta_f(3))*[0;thetaD_s(3);0];
angvel_HL = [thetaD_f(4);0;0] + rot_x(theta_f(4))*[0;thetaD_s(4);0];

% Leg angular velocity (Inertial frame)
angvel_FR_I = R_body*(angvel_FR + w_body);
angvel_FL_I = R_body*(angvel_FL + w_body);
angvel_HR_I = R_body*(angvel_HR + w_body);
angvel_HL_I = R_body*(angvel_HL + w_body);

% Collect the foot inertial positions
pos_feet = [pos_feet_FR; pos_feet_FL; pos_feet_HR; pos_feet_HL];
vel_feet = [vel_feet_FR; vel_feet_FL; vel_feet_HR; vel_feet_HL];

% Jacobian of the inertial foot positions vs joint states
q_FR = [l_comp_x(1); l_comp_y(1); l_pris(1)];
q_FL = [l_comp_x(2); l_comp_y(2); l_pris(2)];
q_HR = [l_comp_x(3); l_comp_y(3); l_pris(3)];
q_HL = [l_comp_x(4); l_comp_y(4); l_pris(4)];

J_FR = jacobian(pos_feet_FR,q_FR);
J_FL = jacobian(pos_feet_FL,q_FL);
J_HR = jacobian(pos_feet_HR,q_HR);
J_HL = jacobian(pos_feet_HL,q_HL);


%% Equation of Motion

% Dynamic EoM formulations

% Lagrangian
L = mb/2*(v_body.'*v_body) + 1/2*w_body.'*I_body*w_body - mb*g*x_body(3);

% EoM using Hamiltonian principles for the angular velocity
dL_dw = jacobian(L, w_body).';
dL_dR = zeros(3,1);
for j=1:3
  dL_dR = dL_dR + skew(R_body(j,:)) * jacobian(L,R_body(j,:)).';
end
eom1 = time_derivative(dL_dw,qq,qqd,xx,xxd,t) + skew(w_body)*dL_dw + dL_dR;

% EoM using Euler-Lagrangian the linear velocity
dL_dx = jacobian(L,x_body).';
dL_dv = jacobian(L,v_body).';
eom2 = time_derivative(dL_dv,qq,qqd,xx,xxd,t) - dL_dx;

% Combined dynamic eom
eom_dyn = [eom2; eom1];           % Combined EoM (linear vel, ang vel)

% Dynamic Equation of motion 
xdd = [vD_body; wD_body];         % System acceleration
M = jacobian(eom_dyn, xdd);
h = subs(eom_dyn,xdd,xdd*0);

%% Forces equations

% Ground contact force, applied at the foot (inertial frame)
% Generalized force = Bg*ug, where ug = [fg_FR; fg_FL; fg_HR; fg_HL]
Bg = jacobian(vel_feet,[v_body; w_body]).';


%% Inverse Kinematics

% Feet linear positions (body frame)
pos_feet_body_FR = l_hip_FR + R_FR*[l_comp_x(1);l_comp_y(1);-l_pris(1)];
pos_feet_body_FL = l_hip_FL + R_FL*[l_comp_x(2);l_comp_y(2);-l_pris(2)];
pos_feet_body_HR = l_hip_HR + R_HR*[l_comp_x(3);l_comp_y(3);-l_pris(3)];
pos_feet_body_HL = l_hip_HL + R_HL*[l_comp_x(4);l_comp_y(4);-l_pris(4)];

% Feet linear velocities (body frame)
vel_feet_body_FR = time_derivative(pos_feet_body_FR,qq,qqd,xx,xxd,t);
vel_feet_body_FL = time_derivative(pos_feet_body_FL,qq,qqd,xx,xxd,t);
vel_feet_body_HR = time_derivative(pos_feet_body_HR,qq,qqd,xx,xxd,t);
vel_feet_body_HL = time_derivative(pos_feet_body_HL,qq,qqd,xx,xxd,t);

% vel_feet_body_FR = subs( subs( diff( subs(pos_feet_body_FR,qq,xx) ,t), xxd,qqd), xx,qq );
% vel_feet_body_FL = subs( subs( diff( subs(pos_feet_body_FL,qq,xx) ,t), xxd,qqd), xx,qq );
% vel_feet_body_HR = subs( subs( diff( subs(pos_feet_body_HR,qq,xx) ,t), xxd,qqd), xx,qq );
% vel_feet_body_HL = subs( subs( diff( subs(pos_feet_body_HL,qq,xx) ,t), xxd,qqd), xx,qq );

% From position in body frame
J_ik_FR = jacobian(vel_feet_body_FR, [thetaD_f(1);thetaD_s(1);lD_pris(1)]);
J_ik_FL = jacobian(vel_feet_body_FL, [thetaD_f(2);thetaD_s(2);lD_pris(2)]);
J_ik_HR = jacobian(vel_feet_body_HR, [thetaD_f(3);thetaD_s(3);lD_pris(3)]);
J_ik_HL = jacobian(vel_feet_body_HL, [thetaD_f(4);thetaD_s(4);lD_pris(4)]);


%%

dp_inertial = R_body*dp_b;

dpD_inertial = time_derivative(dp_inertial,qq,qqd,xx,xxd,t);
dpDD_inertial = time_derivative(dpD_inertial,qq,qqd,xx,xxd,t);

temp = subs(dpDD_inertial,wD_body,wD_body*0); % ang accel = 0

% dp_dd_inertial = J_leg*dp_dd_body + h_leg
J_leg = jacobian(temp, dpDD_b);
h_leg = subs(temp ,dpDD_b,dpDD_b*0);

simplify(J_leg*dpDD_b + h_leg - temp)

% dp_dd_body = J_leg^(-1)*(dp_dd_inertial - h_leg)
leg_length_b = R_body.'*(dpDD_i - h_leg);

%% Generate Function Files

% System states
states = [  theta_f; theta_s; l_pris; x_body; R_body(:); ...
            thetaD_f; thetaD_s; lD_pris; v_body; w_body; ...
            l_comp_x; l_comp_y; l_comp_xD; l_comp_yD];

pos_feet_body = [ pos_feet_body_FR; pos_feet_body_FL;...
                  pos_feet_body_HR; pos_feet_body_HL];

vel_feet_body = [ vel_feet_body_FR; vel_feet_body_FL;...
                  vel_feet_body_HR; vel_feet_body_HL];
         
% System parameters
params = [ mb; g; Ib_1; Ib_2; Ib_3; ...
          l_hip_FR; l_hip_FL; l_hip_HR; l_hip_HL; ground_z];
      
% NOTE: using filesep makes folder names usable on all OS

% foot position

% Jacobian of feet inertial positions end wrt leg joint space
matlabFunction(J_FR, J_FL, J_HR, J_HL, 'File', ...
  ['generated_functions', filesep, 'func_feet_end_Jacobians'], ...
    'Vars', {states, params});

% Dynamic equation of motion for the massed system
matlabFunction(M, h, Bg, 'File', ...
  ['generated_functions', filesep, 'func_MhBe'], 'Vars', {states, params});

% Feet positions and velocities
matlabFunction(pos_feet, vel_feet, 'File', ...
  ['generated_functions', filesep, 'func_feet'], 'Vars', {states, params});

matlabFunction(pos_feet_body, vel_feet_body, 'File', ...
  ['generated_functions', filesep, 'func_feet_body'], 'Vars', {states, params});

matlabFunction(leg_length_b, 'File', ...
  ['generated_functions', filesep, 'func_feetlength'], ...
  'Vars', {states, dp_b, dpD_b, dpDD_i});

disp('done!')

%% Local functions

% % Euler Rotation matrices
% function Rx = rot_x(theta)
% Rx = [1, 0, 0; ...
%   0, cos(theta), -sin(theta); ...
%   0, sin(theta), cos(theta)];
% end
% 
% function Ry = rot_y(theta)
% Ry = [ cos(theta), 0, sin(theta); ...
%   0, 1, 0;...
%   -sin(theta), 0, cos(theta)];
% end
% 
% function Rz = rot_z(theta)
% Rz = [cos(theta), -sin(theta), 0; ...
%   sin(theta), cos(theta), 0;...
%   0, 0, 1];
% end
% 
% % Skew-symmetric transformation
% function S = skew(v)
% S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
% end

% Time derivative workaround for the symbolic toolbox
function df = time_derivative(f,qq,qqd,xx,xxd,t)
  df = subs( subs( diff( subs(f,qq,xx) ,t), xxd,qqd), xx,qq )  ;
end


