% Animate

if (ishandle(10))
  close(10);
end

%% Calculate animation vertex
N = length(data.t);

anim.p_com  = zeros(3,N);   % body center of mass position
anim.p_box  = zeros(24,N);  % body box positions [x(8); y(8); z(8)]
anim.p_hip  = zeros(12,N);  % hip positions [FR, FL, HR, HL]
anim.p_feet = zeros(12,N);  % feet positions with compliance [FR, FL, HR, HL]
anim.p_fnoc = zeros(12,N);  % foot positions no compliance [FR, FL, HR, HL]

% Parameters for symbolic model functions
params = [p.mb; p.g; p.Ib_1; p.Ib_2; p.Ib_3; ...
          p.lh_FR; p.lh_FL; p.lh_HR; p.lh_HL; p.ground_z];
        
for i=1:N
  % Gather relevant states
  x = data.x(:,i);
  xb = x(13:15);
  Rb = reshape(x(16:24), [3,3]);
  
  % Body center of mass
  anim.p_com(:,i) = xb;
    
  % Hip positions
  p_hip_FR = xb + Rb*p.lh_FR;
  p_hip_FL = xb + Rb*p.lh_FL;
  p_hip_HR = xb + Rb*p.lh_HR;
  p_hip_HL = xb + Rb*p.lh_HL;
  anim.p_hip(:,i) = [p_hip_FR; p_hip_FL; p_hip_HR; p_hip_HL];
  
  % Box positions, 1-2-5-6 is the front side
  %   7  --------- 6
  %     /       /
  %  8 --------- 5
  %   3  --------- 2
  %     /       /
  %  4 --------- 1
  %
  p1 = xb + Rb*(p.lh_FR + [0;0;0.1]);
  p2 = xb + Rb*(p.lh_FL + [0;0;0.1]);
  p3 = xb + Rb*(p.lh_HL + [0;0;0.1]);
  p4 = xb + Rb*(p.lh_HR + [0;0;0.1]);
  p5 = xb + Rb*(p.lh_FR - [0;0;0.1]);
  p6 = xb + Rb*(p.lh_FL - [0;0;0.1]);
  p7 = xb + Rb*(p.lh_HL - [0;0;0.1]);
  p8 = xb + Rb*(p.lh_HR - [0;0;0.1]);
  temp = [p1,p2,p3,p4,p5,p6,p7,p8]';
  anim.p_box(:,i) = reshape(temp, [24,1]);
  
  % Leg positions (with compliance)
  x_com = x;
  anim.p_feet(:,i) = func_feet(x, params);
  
  % Leg positions (assuming no compliance)
  x_com(43:58) = zeros(16,1);
  anim.p_fnoc(:,i) = func_feet(x_com, params);
end

%% Animate

% close all

fig10 = figure(10);
set(fig10,'position',[100,100,1600,1200]);

axis_range = [-0.5,5.0, -2,2, -0.5,1];

% Draw the surface
draw_surface(axis_range)

% Begin animation
for i = p.gait_t0/p.dt:p.anim_speed:N
  
  clf
  
  % Draw body CoM (blue)
  draw_sphere(anim.p_com(:,i), 0.025, [0,0,1], 1);
  hold on
  
  % Draw box
  draw_box(anim.p_box(:,i))
  
  % Draw the legs
  draw_legs(anim.p_hip(:,i), anim.p_feet(:,i), anim.p_fnoc(:,i))
  
  % Draw the ground contact forces
  draw_ground_forces(data.xf(:,i),data.ug(:,i))
  
  axis(axis_range);
  daspect([1 1 1]);
  
  title(['Time = ', sprintf('%.2f',data.t(i)), ' s'])
  xlabel('x (m)')
  ylabel('y (m)')
  zlabel('z (m)')
  
  view(45,20) % iso
% view(90,90) % top
% view(0,0) % side
% view(-210,30)
% view(i/N*540/20,20)
% view(90,0) % front
  
  pause(1e-15)
end


% writeAnimation('C:\Users\kaush\OneDrive - Northeastern University\Documents\GitHub\Husky_model\Husky_ROM\Husky_ERG\video.gif')

%% Local functions

function draw_box(pos)
  % Draw a box
  px = pos(1:8,1);
  py = pos(9:16,1);
  pz = pos(17:24,1);
  
  % Draw all 6 faces
  face = cell(6,1);
  face{1} = [1,5,6,2]; % Front side
  face{2} = [2,6,7,3];
  face{3} = [1,2,3,4];
  face{4} = [5,6,7,8];
  face{5} = [4,8,7,3];
  face{6} = [1,5,8,4];   
  
  % front is red
  fill3(px(face{1}),py(face{1}),pz(face{1}),'r', 'FaceAlpha',0.5) 
  
  % the rest is white
  for k = 2:6
    fill3(px(face{k}),py(face{k}),pz(face{k}),'w','FaceAlpha',0.5)  
  end 
end

function draw_sphere(pos, radius, color, alpha)
  % Draw a sphere
  [x,y,z] = sphere;
  
  hSurface = surf(radius*x + pos(1), ...
                  radius*y + pos(2), ...
                  radius*z + pos(3));  
                
  % Blue circle, no edge
  set(hSurface,'FaceColor', color, ...
      'FaceAlpha',alpha,'FaceLighting','gouraud','EdgeColor','none');
end

function draw_legs(pos_hip, pos_feet, pos_fnoc)
  % Draw hip, foot and compliant foor positions
  
  hip_circ_rad = 0.025;
  
  % Draw hip positions (black)
  draw_sphere(pos_hip(1:3), hip_circ_rad, [0,0,0], 1);
  draw_sphere(pos_hip(4:6), hip_circ_rad, [0,0,0], 1);
  draw_sphere(pos_hip(7:9), hip_circ_rad, [0,0,0], 1);
  draw_sphere(pos_hip(10:12), hip_circ_rad, [0,0,0], 1);
  
  % Draw foot positions and lines with compliance (black)
  draw_sphere(pos_feet(1:3), hip_circ_rad, [0,0,0], 1);
  draw_sphere(pos_feet(4:6), hip_circ_rad, [0,0,0], 1);
  draw_sphere(pos_feet(7:9), hip_circ_rad, [0,0,0], 1);
  draw_sphere(pos_feet(10:12), hip_circ_rad, [0,0,0], 1);
  
  draw_line(pos_hip(1:3), pos_feet(1:3), [0,0,0], 1);
  draw_line(pos_hip(4:6), pos_feet(4:6), [0,0,0], 1);
  draw_line(pos_hip(7:9), pos_feet(7:9), [0,0,0], 1);
  draw_line(pos_hip(10:12), pos_feet(10:12), [0,0,0], 1);  
  
  % Draw foot positions without compliance (red)
  draw_sphere(pos_fnoc(1:3), hip_circ_rad, [1,0,0], 0.5);
  draw_sphere(pos_fnoc(4:6), hip_circ_rad, [1,0,0], 0.5);
  draw_sphere(pos_fnoc(7:9), hip_circ_rad, [1,0,0], 0.5);
  draw_sphere(pos_fnoc(10:12), hip_circ_rad, [1,0,0], 0.5);
  
  draw_line(pos_hip(1:3), pos_fnoc(1:3), [1,0,0], 0.5);
  draw_line(pos_hip(4:6), pos_fnoc(4:6), [1,0,0], 0.5);
  draw_line(pos_hip(7:9), pos_fnoc(7:9), [1,0,0], 0.5);
  draw_line(pos_hip(10:12), pos_fnoc(10:12), [1,0,0], 0.5);
  
end

function draw_line(pos1,pos2, color, alpha)
  % Draw straight line
  px = [pos1(1), pos2(1)];
  py = [pos1(2), pos2(2)];
  pz = [pos1(3), pos2(3)];
  plot3(px,py,pz,'LineWidth', 3, 'Color', [color, alpha]);
end

function draw_ground_forces(pos_feet,forces)
  % Draw ground contact forces at the feet
  
  scaler = 0.02;
  width = 0.005;
  
  
  force_tail = pos_feet - forces*scaler;
  
  % Normal forces
  mArrow3([pos_feet(1:2);force_tail(3)],pos_feet(1:3),'color','red', 'stemWidth', width);
  mArrow3([pos_feet(4:5);force_tail(6)],pos_feet(4:6),'color','red', 'stemWidth', width);
  mArrow3([pos_feet(7:8);force_tail(9)],pos_feet(7:9),'color','red', 'stemWidth', width);
  mArrow3([pos_feet(10:11);force_tail(12)],pos_feet(10:12),'color','red', 'stemWidth', width);
  
  % Translational forces
  mArrow3([force_tail(1:2);pos_feet(3)],pos_feet(1:3),'color','blue', 'stemWidth', width);
  mArrow3([force_tail(4:5);pos_feet(6)],pos_feet(4:6),'color','blue', 'stemWidth', width);
  mArrow3([force_tail(7:8);pos_feet(9)],pos_feet(7:9),'color','blue', 'stemWidth', width);
  mArrow3([force_tail(10:11);pos_feet(12)],pos_feet(10:12),'color','blue', 'stemWidth', width);


end

function draw_surface(range)
  fill3([range(1),range(2),range(2),range(1)],...
        [range(3),range(3),range(4),range(4)],...
        [0,0,0,0],'w', 'FaceAlpha',0.25)
end




