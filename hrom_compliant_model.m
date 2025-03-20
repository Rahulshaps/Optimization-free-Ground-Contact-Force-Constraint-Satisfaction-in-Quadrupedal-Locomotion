%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% HROM dynamic model, which calculates the value of dx/dt.
%
% Note: the outputs other than xd are used for plotting.
%
% Inputs: 
%         x = simulated states
%         ue = external forces acting on the body = [linear force; torque]
%         uj = joint accelerations = [theta_f_DD; theta_s_DD; l_pris_DD]
%         t = current time
%         p = parameter struct
%
% Outputs:
%         xd = dx/dt for the numerical integrator
%         ug_model = ground contact force vectors for observer
%         pos_feet = feet inertial position
%         vel_feet = feet inertial velocity
%         del_ref = compliance model deflections
% 
% Authors:  Eric Nauli Sihite
%           Andrew Lessieur
% 
% Date: 5/29/2020
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function [xd, ug_model, pos_feet, vel_feet,pos_feet_b, vel_feet_b, del_ref, ug_wrench] = ...
                                      hrom_compliant_model(x,ue,uj,t,p)

% x = states = [theta_f; theta_s; l_pris; x_body; R_body(:); ...
%               thetaD_f; thetaD_s; lD_pris; v_body; w_body]
% ue = external forces acting on the body, ue = [f_in; tau_body]
% uj = joint space acceleration = [theta_f_DD; theta_s_DD; l_pris_DD]
% t = current time
% p = parameter struct

% Get the states, if you need them
% qf = x(1:4);        % Hip frontal angles [FR, FL, HR, HL] 
% qs = x(5:8);        % Hip sagittal angles
lp = x(9:12);       % Leg prismatic joint length
% xb = x(13:15);      % Body inertial position
Rb = reshape(x(16:24), [3,3]);   % Body rotation matrix
% qsD = x(25:28);     % Hip frontal ang velocity
% qsD = x(29:32);     % Hip sagittal ang velocity
% lpD = x(33:36);     % Leg prismatic joint length rate of change
% vb = x(37:39);      % Body inertial velocity
wb = x(40:42);        % Body angular velocity (about body frame)

% df = x(43:50);      % Feet displacements due to compliance
% dfD = x(51:58);     % Feet displacements rate of change due to compliance
z = x(59:66);   % Lugre model z (FR_xy, FL_xy, etc)

% Used in calculating the functions generated symbolically
params = [p.mb; p.g; p.Ib_1; p.Ib_2; p.Ib_3; ...
          p.lh_FR; p.lh_FL; p.lh_HR; p.lh_HL; p.ground_z];

% Non-constraints forces ==================================================

% Calculate the dynamic model variables, M*acc + h = Bg*ug
[M, h, Bg] = func_MhBe(x, params);
RbD = Rb*skew(wb);

% Determine non-constraint ground contact forces (on each legs)
[pos_feet, vel_feet] = func_feet(x, params);
[pos_feet_b, vel_feet_b] = func_feet_body(x, params);
% Ground reaction forces
if (p.use_lugre == 1)
  % LuGre friction model
  [fg_FR, zd_FR] = ground_force_model_lugre(pos_feet(1:3), vel_feet(1:3), z(1:2), p);
  [fg_FL, zd_FL] = ground_force_model_lugre(pos_feet(4:6), vel_feet(4:6), z(3:4), p);
  [fg_HR, zd_HR] = ground_force_model_lugre(pos_feet(7:9), vel_feet(7:9), z(5:6), p);
  [fg_HL, zd_HL] = ground_force_model_lugre(pos_feet(10:12), vel_feet(10:12), z(7:8), p);
  zd = [zd_FR; zd_FL; zd_HR; zd_HL];
else
  % Coulomb and viscous friction model
  fg_FR = ground_force_model(pos_feet(1:3), vel_feet(1:3), p);
  fg_FL = ground_force_model(pos_feet(4:6), vel_feet(4:6), p);
  fg_HR = ground_force_model(pos_feet(7:9), vel_feet(7:9), p);
  fg_HL = ground_force_model(pos_feet(10:12), vel_feet(10:12), p);
  zd = zeros(8,1);
end

%%%%%% USE constraint equation to find out the GRF instead of the spring
%%%%%% model. How to selectively add the constraint to the only the legs on
%%%%%% the ground. Maybe the Jc is a function of time based on the gait or
%%%%%% TD data. Need everything in 
%%%%%% u_g = (Jc*M^(-1)*Jc')^(-1)(Jc*M^(-1)(S'(tau)-b -g)+dJ*dq). tau is
%%%%%% all the inputs [u_b, u_l]. I need joint positions and velocities at
%%%%%% every instant to calculate Jc and dJC and also need uj

ug = [fg_FR;fg_FL;fg_HR;fg_HL];
ug_model = ug;

% Gather all non-constraint forces and joint acceleration
h0 = -h + ue + Bg*ug; % length 6

ug_wrench = Bg*ug;

% Compliance model ========================================================

% Jacobian of leg inertial position to the leg length space (df_x, df_y, lp)
[Jl_FR, Jl_FL, Jl_HR, Jl_HL] = func_feet_end_Jacobians(x,params);

% Ground forces mapped to the leg displacements
fg_FR_mapped = Jl_FR*fg_FR;
fg_FL_mapped = Jl_FL*fg_FL;
fg_HR_mapped = Jl_HR*fg_HR;
fg_HL_mapped = Jl_HL*fg_HL;

% Target deflection depending on the lateral forces (xy only)
del_ref_FR = fg_FR_mapped(1:2) * lp(1)^3 / (3*p.E*p.I);
del_ref_FL = fg_FL_mapped(1:2) * lp(2)^3 / (3*p.E*p.I);
del_ref_HR = fg_HR_mapped(1:2) * lp(3)^3 / (3*p.E*p.I);
del_ref_HL = fg_HL_mapped(1:2) * lp(4)^3 / (3*p.E*p.I);
del_ref = -[del_ref_FR(1); del_ref_FL(1); del_ref_HR(1); del_ref_HL(1); ...
            del_ref_FR(2); del_ref_FL(2); del_ref_HR(2); del_ref_HL(2)];

% Acceleration for the displacement (spring damping model)
dfDD = p.kc_p*(del_ref - x(43:50)) - p.kc_d*x(51:58);

% Constraint vel y, angvel xz
% Js = [0, 1, 0, 0, 0, 0; ...
%       0, 0, 0, 1, 0, 0;
%       0, 0, 0, 0, 0, 1];
% 
% Mc = [M, -Js'; Js, zeros(3,3)];
% hc = [h0; zeros(3,1)];
% temp = Mc\(hc);

temp = M\h0;
qdd = temp(1:6);


% Calculate dx/dt =========================================================

xd = x*0;

xd(1:15) = x(25:39);  % Velocities
xd(16:24) = RbD(:);   % Rotation matrix rate of change in SO(3)
xd(25:36) = uj;       % Joint acceleration
% xd(37:42) = M\(h0);   % Body accelerations (dynamic model)
xd(37:42) = qdd;   % Body accelerations (dynamic model)

% Compliant model
xd(43:50) = x(51:58); % Foot displacement rate of change
xd(51:58) = dfDD;     % Foot displacement 2nd derivative

% Friction model (LuGre)
xd(59:66) = zd;

end

%% Local Functions

function f = ground_force_model(x,v,p)
  % Ground force model using basic friction model

  % x = foot inertial position vector
  % v = foot inertial velocity vector
  
  if (x(3) <= p.ground_z)
    fz = ground_model(x(3), v(3), p);
    fx = friction_model(v(1), fz, p);
    fy = friction_model(v(2), fz, p);
    f = [fx;fy;fz];
  else
    f = [0;0;0];
  end
  
end

function f = friction_model(v,N,p)

  % v = velocity along the surface, N = normal force 
  % Coulomb and viscous friction model
  fc = p.kfc * norm(N) * sign(v);
  fs = p.kfs * norm(N) * sign(v);
  f = -(fc + (fs-fc)*exp(-norm(v/p.vs)^2) + p.kfb * v);
  
end

function [f,zd] = ground_force_model_lugre(x,v,z,p)

  % LuGre friction model. Solve for friction force and dz/dt
  % x = foot inertial position vector
  % v = foot inertial velocity vector
  % z = vector of average bristle displacement in Dahl and LuGre model
  %       in x and y directions
  
  if (x(3) <= p.ground_z)
    fz = ground_model(x(3), v(3), p);
    flag = true;
  else
    fz = 0;
    flag = false;
  end
    
  % Calculate zdx
  [fx,zdx] = friction_model_lugre(v(1), z(1), fz, p);
  [fy,zdy] = friction_model_lugre(v(2), z(2), fz, p);
  
  % fx and fy are zero if not contacting the ground
  if (flag == false)
    fx = 0;
    fy = 0;
  end
  
  f = [fx;fy;fz];
  zd = [zdx;zdy];
end

function [f,zd] = friction_model_lugre(v,z,N,p)

  % v = velocity along the surface, N = normal force 
  % z = average bristle displacement and rate (Dahl and LuGre model)
  
  fc = p.kfc * norm(N); % Coulomb friction
  fs = p.kfs * norm(N); % Static friction  
  
  gv = fc + (fs-fc)*exp(-norm(v/p.vs)^2);
  
  % If gv smaller than certain threshold, assume leg no longer contacting
  % the ground. Rapidly decay the dz/dt in that case.
  if (gv < p.gv_min)
    % Lose contact or barely any normal force
    zd = - p.zd_max * z;         
    f = 0;
  else
    % LuGre model
    zd = v - p.s0 * norm(v) / gv * z;         % dz/dt
    f = -(p.s0 * z + p.s1 * zd + p.kfb * v);  % friction force
  end
  
end

function f = ground_model(x,v,p)
  % Assume x < ground_z 
  if (p.use_damped_rebound == 1)
    
    % damped rebound model
    f = -p.kp_g * (x - p.ground_z) - p.kd_g * v;
  else 
    % undamped rebound model
    if (v < 0)
      f = -p.kp_g * (x - p.ground_z) - p.kd_g * v;
    else
      f = -p.kp_g * (x - p.ground_z);
    end
  end
end

function Rx = rot_x(theta)
Rx = [1, 0, 0; ...
  0, cos(theta), -sin(theta); ...
  0, sin(theta), cos(theta)];
end

function Ry = rot_y(theta)
Ry = [ cos(theta), 0, sin(theta); ...
  0, 1, 0;...
  -sin(theta), 0, cos(theta)];
end

function Rz = rot_z(theta)
Rz = [cos(theta), -sin(theta), 0; ...
  sin(theta), cos(theta), 0;...
  0, 0, 1];
end

function S = skew(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end