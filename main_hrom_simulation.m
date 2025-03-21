  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Husky Reduced Order Model (HROM) simulation.
% Includes the rigid vs compliant model variations, optimization, control,
% MBDO, ERG, and others yet specified features.
%
% Authors:  Eric Nauli Sihite
%           Andrew Lessieur
%           Pravin Dangol
%
% Date: 8/17/2020
%
% Version: BETA
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulation Setup

close all, clear all, clc

% Add folder, filesep makes this usable on all OS

% Functions from symbollic toolbox
addpath(['generated_functions',filesep])

% Commonly used math functions
addpath(['utility_functions',filesep])

% Simulation data
addpath(['simulation_data',filesep])

% Elist=[];
% for Kp_foot=0:5:80
%     for Kd_foot=0:5:40

% Parameters ==============================================================

% Model parameters (stored in p)
p.use_lugre = 0;            % 1 = enable lugre friction model
p.use_damped_rebound = 1;   % 1 = enable damped rebound, very plastic impact
p.use_husky = 0;            % 1 = load Husky model parameters, 0 = preset box
p.use_centered_mass = 0;    % 0 = use Husky front heavy mass distribution
load_husky_parameters       % Load the other Husky and model parameters
% Gait parameters
p.N_gait = 15;          % Number of steps simulated
% Simulation parameters
p.dt = 0.0001;        % Simulation discrete time step (second)
p.gait_t0 = 0.25;      % Gait starting time (second)
p.gait_T = 0.25;      % 2 Hz %0.25; % 4 Hz
p.t_end = p.gait_t0 + p.N_gait*p.gait_T; % simulation end time
p.vx_t=0.5;
%swing traj params
p.step_length=p.vx_t*p.gait_T/2;
p.swing_height=0.1;
% foot pos ref gains
p.Kp_foot=diag([0,15,20]);
p.Kd_foot=diag([5,5,0]);
p.Kp_st=0.15;
%animation
p.anim_speed=400;

params = [p.mb; p.g; p.Ib_1; p.Ib_2; p.Ib_3; ...
          p.lh_FR; p.lh_FL; p.lh_HR; p.lh_HL; p.ground_z];
%VMC params
Kp_x    = 0;%Unused
Kp_y    = 0;%Unused
Kp_z    = 200;
Kp_roll  = 500;
Kp_pitch = 135;
Kp_yaw  = 0;%Unused
Kd_x    = 80;
Kd_y    = 0;%Unused
Kd_z    = 40;
Kd_roll  = 40;
Kd_pitch = 40;
Kd_yaw  = 50;

p.K_list=[  Kp_x;
            Kp_y;
            Kp_z;
            Kp_roll;
            Kp_pitch;
            Kp_yaw;
            Kd_x;
            Kd_y;
            Kd_z;
            Kd_roll;
            Kd_pitch;
            Kd_yaw];
%% Simulation Setup

% Initialize variables
t_sim = 0:p.dt:p.t_end;   % simulation time vector
N = length(t_sim);        % simulation step length

% Initial states
Nx = 66;                  % State size
x = zeros(Nx,1);          % Simulation state

% initial body rotation matrix
R0 = rot_x(0);
x(16:24) = R0(:);

% initial body center of mass position
x(13:15) = [0; 0; p.leg_init_height - p.lh_FR(3)]; % assume neutral orientation

% Initial foot inertial position (body frame)
pos_foot_FR = p.lh_FR - [0; 0; p.leg_init_height]+[0.05;-0.1;0].*p.use_husky+[0;-0.05;0].*~p.use_husky;
pos_foot_FL = p.lh_FL - [0; 0; p.leg_init_height]+[0.05;0.1;0].*p.use_husky+[0;0.05;0].*~p.use_husky;
pos_foot_HR = p.lh_HR - [0; 0; p.leg_init_height]+[-0.05;-0.1;0].*p.use_husky+[0;-0.05;0].*~p.use_husky;
pos_foot_HL = p.lh_HL - [0; 0; p.leg_init_height]+[-0.05;0.1;0].*p.use_husky+[0;0.05;0].*~p.use_husky;

pos_foot_FR_w = p.lh_FR +[0.05;-0.1;0].*p.use_husky+[0;-0.05;0].*~p.use_husky;
pos_foot_FL_w = p.lh_FL +[0.05;0.1;0].*p.use_husky+[0;0.05;0].*~p.use_husky;
pos_foot_HR_w = p.lh_HR +[-0.05;-0.1;0].*p.use_husky+[0;-0.05;0].*~p.use_husky;
pos_foot_HL_w = p.lh_HL +[-0.05;0.1;0].*p.use_husky+[0;0.05;0].*~p.use_husky;


% use inverse kinematics to get the initial joint states
[q_FR, qd_FR] = ik(pos_foot_FR, [0;0;0], p.lh_FR);
[q_FL, qd_FL] = ik(pos_foot_FL, [0;0;0], p.lh_FL);
[q_HR, qd_HR] = ik(pos_foot_HR, [0;0;0], p.lh_HR);
[q_HL, qd_HL] = ik(pos_foot_HL, [0;0;0], p.lh_HL);

% initial joint states
temp1 = [q_FR, q_FL, q_HR, q_HL]';
temp2 = [qd_FR, qd_FL, qd_HR, qd_HL]';
x(1:12) = temp1(:);       % joint angles
x(25:36) = temp2(:);  % joint ang vel
q_des = x(1:12);
dq_des = x(25:36);

% other variable
ue = zeros(6,1);        % external forces
ug = zeros(12,1);       % ground forces
uj = zeros(12,1);       % joint state accelerations
walking_state = 0;      % walking state (0 = left, 1 = right)
s  = 0;                 % gait timing, s = [0,1]
s_old = 0;              % gait timing opne step before, s = [0,1]
xf_ref = [pos_foot_FR; pos_foot_FL; pos_foot_HR; pos_foot_HL];   % foot position reference
p.zero_config=[pos_foot_FR; pos_foot_FL; pos_foot_HR; pos_foot_HL];
p.zero_config_w=[pos_foot_FR_w; pos_foot_FL_w; pos_foot_HR_w; pos_foot_HL_w];
dxf_ref = zeros(12,1);  % foot velocity reference
ddxf_ref = zeros(12,1);
cost1 = 0;

% body ref
xc_ref = [0;0;x(15)];
xw_ref = [0;0;x(15)];
xcd_ref = [p.vx_t;0;0];
xwd_ref = [0;0;0];

%ERG 
alpha_ERG=25;
use_ERG=1;

% p.kfs=0.2;
%%Swing trajectory
swingfootpos0 = repmat([0;0;0],4,1); 
swingfootpos1 = repmat([p.step_length;0;0],4,1); 
[qswing, dqswing] = getSwingFootTraj(p.zero_config_w,[0; 0; p.leg_init_height;0;0;0],p);

%Linearising perturbations 

 h = 1e-5; 

% Run the Simulation
TD1=1;
TD2=1;
TD3=1;
TD4=1;
TD=ones(4,1);
nstep=0;
flag_start = 1;
t0=p.gait_t0;

[Mkminus1,~,~]= func_MhBe(x,params);

rkminus1 = 0; 

sum = 0;

Ko = 100*eye(6);

for i = 1:N
  
  t = t_sim(i); % Simulation time stamp

  % ---------------------------------------------------
  % Observer
  % ---------------------------------------------------
  
  % Extract the ground contact forces, foot pos and vel

    [xd, ug_temp, pos_feet, vel_feet, pos_feet_b, vel_feet_b, del_ref, ug_wrench] = ...
    hrom_compliant_model(x, ue, uj, t, p);

    % x_inertial = Rb*x_body
    Rb = reshape(x(16:24), [3,3]); % Body rotation matrix

    % Extract roll pitch and yaw
    Rb_T = Rb.';
    pitch = asin(-Rb_T(1,3));   
    yaw = atan2(Rb_T(1,2), Rb_T(1,1));    
    roll = atan2(Rb_T(2,3), Rb_T(3,3));    
    xc_w = x(13:15);    
    xcd_w = x(37:39);    
    c_state_w=[xc_w;xcd_w];    
    c_state_b=blkdiag(Rb_T,Rb_T)*c_state_w;

    % 

    TD1=pos_feet(3)<=p.ground_z;
    TD2=pos_feet(6)<=p.ground_z;
    TD3=pos_feet(9)<=p.ground_z;
    TD4=pos_feet(12)<=p.ground_z;
    TD=[TD1;TD2;TD3;TD4];

    %---------------------------------------
    % Conjugate Momentum
    % --------------------------------------

    v_body = x(37:39);

    w_body = x(40:42);

    qdot = [v_body; w_body];

    [Mk, hk, ~] = func_MhBe(x, params);

    Mdot = (Mk - Mkminus1)./p.dt; 

    pk = Mk*qdot; 

    beta = -Mdot*qdot + hk; 

    sum = sum + (rkminus1 - beta)*(p.dt);

    rk = Ko*(pk - sum); 

    Mkminus1 = Mk;

    rkminus1 = rk;     

  % ---------------------------------------------------
  % Update state and state reference variables
  % ---------------------------------------------------
    % start walking after p.gait_t0
    if (t > p.gait_t0)
        
        % state machine
        s = (t - t0)/p.gait_T;
        sn=round(s/p.dt*p.gait_T)+1;
        if walking_state==0 %Swing 1&4 Stance 2&3
            leg_states=[2 1 1 2];%stance 1 swing 2 stop 0 seq:FR FL BR BL
            if s>=1
                walking_state=1;
                nstep=nstep+1;
                t0=t;
                leg_states=zeros(1,4);
                [qswing, dqswing] = getSwingFootTraj(pos_feet,c_state_w,p);
            end
        elseif walking_state==1 %Swing 2&3 Stance 1&4
                leg_states=[1 2 2 1];
            if s>=1
                walking_state=0;
                nstep=nstep+1;
                t0=t;
                leg_states=zeros(1,4);
                [qswing, dqswing] = getSwingFootTraj(pos_feet,c_state_w,p);
            end
        end
        if nstep>p.N_gait %stop Simulation
            break
        end
        
        ref_w=[xc_ref;xcd_ref];
        %ERG
if use_ERG==1
        if flag_start==1
            ref_applied = ref_w;
        end
        if walking_state==0 %stance 2&3
            [f_e,J_r,d_r,s_r,v] =func_ERG_params(x, params, ref_w,[pos_feet(4:6);pos_feet(7:9)],p.Kp_foot,p.Kd_foot,p.kfs); 
            [d_ref_applied,hr,hw]=ERG_test(J_r,d_r,s_r,ref_w,ref_applied,alpha_ERG);  
        elseif walking_state==1 %stance 1&4
            [f_e,J_r,d_r,s_r,v] =func_ERG_params(x, params, ref_w,[pos_feet(1:3);pos_feet(10:12)],p.Kp_foot,p.Kd_foot,p.kfs); 
            [d_ref_applied,hr,hw]=ERG_test(J_r,d_r,s_r,ref_w,ref_applied,alpha_ERG);  
        end
end
        % leg_Controller
        for ii=1:4 %seq FR FL BR BL
            if leg_states(ii)==1 %stance
                if use_ERG==1
                    v_stance=[p.Kp_foot,p.Kd_foot]*(ref_applied -  c_state_w);
                    ddxf_ref(3*ii-2:3*ii) = -Rb_T*v_stance;
                else
                    ddxf_ref(3*ii-2:3*ii)=stance(c_state_w,ref_w,Rb_T,p);
                end
            elseif leg_states(ii)==2 %swing
                xf_ref(3*ii-2:3*ii)=Rb_T*(qswing(3*ii-2:3*ii,sn)-c_state_w(1:3));
                dxf_ref(3*ii-2:3*ii)=Rb_T*(dqswing(3*ii-2:3*ii,sn)-c_state_w(4:6));
                ddxf_ref(3*ii-2:3*ii)=zeros(3,1);
            elseif leg_states(ii)==0 %stop
                xf_ref(3*ii-2:3*ii)=pos_feet_b(3*ii-2:3*ii);
                dxf_ref(3*ii-2:3*ii)=zeros(3,1);
                ddxf_ref(3*ii-2:3*ii)=zeros(3,1);
            end
        end
%
  % calculate the inverse kinematics
  % NOTE: xf_ref and xdf_ref are about body frame
  [q_FR, qd_FR] = ik(xf_ref(1:3),   dxf_ref(1:3),   p.lh_FR);
  [q_FL, qd_FL] = ik(xf_ref(4:6),   dxf_ref(4:6),   p.lh_FL);
  [q_HR, qd_HR] = ik(xf_ref(7:9),   dxf_ref(7:9),   p.lh_HR);
  [q_HL, qd_HL] = ik(xf_ref(10:12), dxf_ref(10:12), p.lh_HL);
  
  % form joint state references
  temp1 = [q_FR, q_FL, q_HR, q_HL]';
  temp2 = [qd_FR, qd_FL, qd_HR, qd_HL]';
  q_des = temp1(:);   % target joint angles
  dq_des = temp2(:);  % target joint ang vel
  
  
  % ---------------------------------------------------
  % Force and Joint Actuation
  % ---------------------------------------------------
  
  % External forces acting on the body
  % ue = [force (inertial frame) ; torque (body frame)]
  % ue = zeros(6,1);
  
  % Joint acceleration
  % uj = [q_hip_frontal_DD; q_hip_sagittal_DD; l_pris_DD]
  % order for each one of the states: [FR, FL, HR, HL]

  delta_angle = delta_theta(q_des, x(1:12));
  uj = ( p.Kp*delta_angle + p.Kd*(dq_des-x(25:36)) );
  % ---------------------------------------------------
  % Record Data
  % ---------------------------------------------------
  
  % Update the data struct
  data.t(:,i) =t;
  data.x(:,i) = x;
  data.ue(:,i) = ue;
  data.ug(:,i) = ug_temp;
  data.ug_wrench(:,i) = ug_wrench;
  data.uj(:,i) = uj;
  data.rpy(:,i) = [roll;pitch;yaw];
  data.xf(:,i) = pos_feet;
  data.vf(:,i) = vel_feet;
  data.del_ref(:,i) = del_ref;
  
  % xf in body frame
  data.xfB(1:3,i) = Rb'*(pos_feet(1:3) -  x(13:15));
  data.xfB(4:6,i) = Rb'*(pos_feet(4:6) -  x(13:15));
  data.xfB(7:9,i) = Rb'*(pos_feet(7:9) -  x(13:15));
  data.xfB(10:12,i) = Rb'*(pos_feet(10:12) -  x(13:15));
  
  % % xf desired in body frame
  %         data.xfB_t(:,i) = pos_des_B;
  
  data.q_des(:,i) = q_des;
  data.xc_ref(:,i) = xc_ref;
  data.xcd_ref(:,i) = xcd_ref;

  data.rk(:,i) = rk;

  if use_ERG==1
      data.xw_ref(:,i) = ref_applied(1:3);
      data.xwd_ref(:,i) = ref_applied(4:6);
      data.hr(:,i)=hr;
      data.hw(:,i)=hw;
      data.fe(:,i) = f_e;
  end
  
  data.TD(:,i)=TD;
  data.s(:,i)=s;
  data.leg_states(:,i)=leg_states;
  % 
  % if mod(i,20) == 0  
  % [A,Be,Bj] = linearize(x, ue, uj, t, p , h);
  % 
  % ue_last = zeros(size(ue,1),4);
  % % Q = diag([10 100000 100]);
  % Q = 500000*eye(3,3);
  % R=  1*eye(6,6);
  % % ue_opt = zeros(size(ue,1),4);
  % % cost1 = optimizer_lin(x, ue_opt, uj, A, Be, Bj,Q, R, p, Rb);
  % data.cost1(:,i) = cost1;
  % options = optimoptions('fmincon','Display', 'iter');
  % 
  % ue_opt = fmincon(@(ue_opt) optimizer_lin(x, ue_opt, uj, A, Be, Bj,Q, R, p, Rb), ue_last, [],[],[],[],zeros(6,4),10*ones(6,4),[],options);
  % ue = ue_opt(:,1);
  % end

  % march states using RK4
  xk = march_rk4(x, ue, uj, t, p);  % x_{k+1}
  x = xk;     % Update the states for the next time step
  
  % march foot reference trajectories
  xf_ref  = xf_ref  + p.dt * dxf_ref;
  dxf_ref = dxf_ref + p.dt * ddxf_ref;
  
  %update references
  if use_ERG==1
        ref_applied = ref_applied+p.dt*d_ref_applied;
  end
  xc_ref = xc_ref + p.dt* xcd_ref;  
%   xw_ref = xw_ref + p.dt* xwd_ref;  
    end
end

%% Plotting and animation

plot_animation_script

plot_data_script
%% Local functions
% function cost = optimizer_lin(decision_var)
% t = 0;
% h=1e-5;
% [A,Be,Bj] = linearize(x, ue, uj, t, p, h);
% Q = 1*eye(3,3);
% R = 1*eye(6,6);
% % A, B linear model
% dt = 0.0001;
% % Explicit Euler: x_(k+1) = x_k + dt*f(x,u)
% Ad = dt * A;
% Bed = dt * Be;
% Bjd = dt * Bj;
% xk = x0;
% cost = 0;   
% for i = 1:4
%   % ue = ue_opt(i);
%   xk = xk + Ad*xk + Bed * ue + Bjd*uj;
%   Rb = reshape(xk(16:24), [3,3]); % Body rotation matrix
%   % Extract roll pitch and yaw
%   Rb_T = Rb.';
%   pitch = asin(-Rb_T(1,3));
%   yaw = atan2(Rb_T(1,2), Rb_T(1,1));
%   roll = atan2(Rb_T(2,3), Rb_T(3,3));
%   xe = [pitch roll yaw];
%   cost = cost + xe*Q*xe' + ue'*R*ue;
% end
% 
% end
function [A,Be,Bj ] = linearize(x, ue, uj, t, p , h)

% preallocating size for the A, Be and Bj matrices
A = zeros([size(x,1) size(x,1)]);
Be = zeros([size(x,1) size(ue,1)]); 
Bj = zeros([size(x,1) size(uj,1)]);

for i= 1:size(x) % looping through each x component
  x_pert_in = x; % setting perturbation input
  
  x_pert_in(i) = x_pert_in(i)+h; %small perturbation in the forward direction 
  
  [xd_pert1, ~, ~, ~, ~, ~, ~] = hrom_compliant_model(x_pert_in, ue, uj, t, p); % output perturbation

  x_pert_in(i) = x_pert_in(i)-2*h; % small perturbation in the backward direction , ( times 2 to compensate for addition in the earlier step)
  
  [xd_pert2, ~, ~, ~, ~, ~, ~] = hrom_compliant_model(x_pert_in, ue, uj, t, p); % output perturbation

  A(:,i) = (xd_pert1-xd_pert2)./(2*h); %appending to A matrix
end

for i=1:size(ue) % looping through each ue compenent
  ue_pert_in = ue; 
  
  ue_pert_in(i) = ue_pert_in(i)+h;
  [xd_pert1, ~, ~, ~, ~, ~, ~] = hrom_compliant_model(x, ue_pert_in, uj, t, p);
  
  ue_pert_in(i) = ue_pert_in(i)-2*h;
  [xd_pert2, ~, ~, ~, ~, ~, ~] = hrom_compliant_model(x, ue_pert_in, uj, t, p);

  Be(:,i) = ((xd_pert1-xd_pert2)./(2*h));
end

for i = 1:size(uj) % looping through each uj compo
  uj_pert_in = uj;

  uj_pert_in(i) = uj_pert_in(i)+h; 
  [xd_pert1, ~, ~, ~, ~, ~, ~] = hrom_compliant_model(x, ue, uj_pert_in, t, p);

  uj_pert_in(i) = uj_pert_in(i)-2*h;
  [xd_pert2, ~, ~, ~, ~, ~, ~] = hrom_compliant_model(x, ue, uj_pert_in, t, p);

  Bj(:,i) = ((xd_pert1-xd_pert2)./(2*h));
end
end

function [qswing, dqswing] = getSwingFootTraj(pos_feet,c_state,p) %swing trajectory in inertia frame
    vx=c_state(4);
    vy=c_state(5);
    qswing=[];
    dqswing=[];
    footpos0=pos_feet;
    footpos1=pos_feet+repmat([p.gait_T*vx*1.5;p.gait_T*vy*1.5+vy*0.05;0],4,1);%y compensation using Raibert's law
    for ii=1:4
        nn=ii*3-2; 
        swingheight=p.swing_height;
        timestp0=0;
        timestpf=1;
        Ts=p.dt/p.gait_T;

        % Trajectory for X and Y
        waypointsXY = [footpos0(nn:nn+1) footpos1(nn:nn+1)];
        timestampsXY = [timestp0  timestpf];
        timevecswingXY = timestampsXY(1):Ts:timestampsXY(end);
        [XYq, XYqd, ~, ~] = quinticpolytraj(waypointsXY, timestampsXY, timevecswingXY);

        % Trajectory for Z
        waypointsZ = [footpos0(nn+2) 0.5*footpos0(nn+2)+0.5*footpos1(nn+2)+swingheight footpos1(nn+2)]; 
        timestpMid = (timestp0+timestpf)/2; % Top of swing at midpoint
        timestampsZ = [timestp0 timestpMid timestpf];
        timevecswingZ = timestampsZ(1):Ts:timestampsZ(end);
        [Zq, Zqd, ~, ~] = quinticpolytraj(waypointsZ, timestampsZ, timevecswingZ);

        % combine xy and z trajectory
        qswing = [qswing;XYq; Zq];
        dqswing = [dqswing;XYqd; Zqd];
    end
end

function xdd_b=stance(c_state_w,ref_w,Rb_T,p)
        xdd_w = p.Kp_foot*(ref_w(1:3) - c_state_w(1:3)) + p.Kd_foot*(ref_w(4:6) -  c_state_w(4:6));
        xdd_b = -Rb_T*xdd_w;    
end

% RK4 integration scheme
function xk = march_rk4(x, ue, uj, t, p)
f1 = hrom_compliant_model(x, ue, uj, t, p);
f2 = hrom_compliant_model(x + f1*p.dt/2, ue, uj, t + p.dt/2, p);
f3 = hrom_compliant_model(x + f2*p.dt/2, ue, uj, t + p.dt/2, p);
f4 = hrom_compliant_model(x + f3*p.dt, ue, uj, t + p.dt, p);
xk = x + (f1/6 + f2/3 + f3/3 + f4/6)*p.dt;
end

% Inverse Kinematics, only considers one leg at a time.
% inputs: xf_b = desired foot position,
%         dxf_b = desired foot velocity
%         Lh_b = length from center of mass to the hip joint
% outputs: [q,dq] = joint states and velocities

function [q,dq] = ik(xf_b, dxf_b, Lh_b)
% Prismatic Joint Length
lp = norm(xf_b - Lh_b);
y = (xf_b - Lh_b)/lp;
theta_f = atan2(y(2),-y(3));
theta_s = atan2(-y(1),norm([y(2),y(3)]));
q = [theta_f;theta_s;lp];

% jacobian(xf_dot,dq)
delK = [0                                 -lp*cos(theta_s)                -sin(theta_s);
  lp*cos(theta_f)*cos(theta_s)    -lp*sin(theta_f)*sin(theta_s)    sin(theta_f)*cos(theta_s);
  lp*sin(theta_f)*cos(theta_s)     lp*cos(theta_f)*sin(theta_s)   -cos(theta_f)*cos(theta_s)];
dq = delK*dxf_b;
end

function y = delta_theta(x1,x2)
y = x1-x2;

while (y > 2*pi)
  y = y - 2*pi;
end

while (y < -2*pi)
  y = y + 2*pi;
end
end
                  