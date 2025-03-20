%% Plot Data

close all

line_t = 1; % line thickness

% Body position and orientation with their rates ==========================

figure(1)

subplot(2,2,1), 
plot(data.t(:), data.x(13:15,:)', 'LineWidth', line_t)
title('body position')
ylabel('pos. (m)')
% xlabel('time (s)')

subplot(2,2,3)
plot(data.t(:), data.rpy(:,:)'/pi*180, 'LineWidth', line_t)
hold on
plot(data.t(:), data.ue(5,:))
hold off
title('body orientation')
ylabel('angles (deg)')
xlabel('time (s)')
legend('roll','pitch','yaw', 'my')

subplot(2,2,2)
plot(data.t(:), data.x(37:39,:)', 'LineWidth', line_t)
title('body velocity')
ylabel('vel. (m/s)')
% xlabel('time (s)')

subplot(2,2,4)
plot(data.t(:), data.x(40:42,:)', 'LineWidth', line_t)
title('body ang. vel.')
ylabel('ang. vel. (rad/s)')
xlabel('time (s)')
axis([-inf,inf,-10,10])

% Joint states and rates ==================================================

figure (2)

subplot(3,2,1)
plot(data.t(:), data.x(1:4,:)', 'LineWidth', line_t)
title('hip frontal angles')
ylabel('angle (rad)')
% xlabel('time (s)')
legend('FR','FL','HR','HL')

subplot(3,2,3)
plot(data.t(:), data.x(5:8,:)', 'LineWidth', line_t)
title('hip sagittal angles')
ylabel('angle (rad)')
% xlabel('time (s)')

subplot(3,2,5)
plot(data.t(:), data.x(9:12,:)', 'LineWidth', line_t)
title('prismatic leg length')
ylabel('length (m)')
xlabel('time (s)')

subplot(3,2,2)
plot(data.t(:), data.x(25:28,:)', 'LineWidth', line_t)
title('hip frontal ang velocity')
ylabel('ang vel (rad/s)')
% xlabel('time (s)')
legend('FR','FL','HR','HL')

subplot(3,2,4)
plot(data.t(:), data.x(29:32,:)', 'LineWidth', line_t)
title('hip sagittal ang velocity')
ylabel('ang vel (rad/s)')
% xlabel('time (s)')

subplot(3,2,6)
plot(data.t(:), data.x(33:36,:)', 'LineWidth', line_t)
title('prismatic leg length rate')
ylabel('length rate (m/s)')
xlabel('time (s)')



% Leg positions and rates =================================================

x_foot = [data.xf(1,:);data.xf(4,:);data.xf(7,:);data.xf(10,:)];
y_foot = [data.xf(2,:);data.xf(5,:);data.xf(8,:);data.xf(11,:)];
z_foot = [data.xf(3,:);data.xf(6,:);data.xf(9,:);data.xf(12,:)];
xd_foot = [data.vf(1,:);data.vf(4,:);data.vf(7,:);data.vf(10,:)];
yd_foot = [data.vf(2,:);data.vf(5,:);data.vf(8,:);data.vf(11,:)];
zd_foot = [data.vf(3,:);data.vf(6,:);data.vf(9,:);data.vf(12,:)];

figure(3)

subplot(3,2,1)
plot(data.t(:), x_foot, 'LineWidth', line_t)
title('foot x positions')
ylabel('pos (m)')
% xlabel('time (s)')
legend('FR','FL','HR','HL')

subplot(3,2,3)
plot(data.t(:), y_foot, 'LineWidth', line_t)
title('foot y positions')
ylabel('pos (m)')
% xlabel('time (s)')

subplot(3,2,5)
plot(data.t(:), z_foot, 'LineWidth', line_t)
title('foot z positions')
ylabel('pos (m)')
xlabel('time (s)')

subplot(3,2,2)
plot(data.t(:), xd_foot, 'LineWidth', line_t)
title('foot x velocity')
ylabel('vel (m/s)')
% xlabel('time (s)')
legend('FR','FL','HR','HL')

subplot(3,2,4)
plot(data.t(:), yd_foot, 'LineWidth', line_t)
title('foot y velocity')
ylabel('vel (m/s)')
% xlabel('time (s)')

subplot(3,2,6)
plot(data.t(:), zd_foot, 'LineWidth', line_t)
title('foot z velocity')
ylabel('vel (m/s)')
xlabel('time (s)')

% Ground contact forces ===================================================

figure(4)

axis_range = [data.t([1,N]), -10,10];

ns = 101;

subplot(2,2,1)

plot(data.t(:), [smooth(data.ug(4,:),ns), smooth(data.ug(5,:),ns), smooth(data.ug(6,:),ns)], 'LineWidth', line_t)
axis(axis_range)
title('FL foot GCF')
ylabel('force (N)')
xlabel('time (s)')
legend('x','y','z')

subplot(2,2,2)
plot(data.t(:), data.ug(1:3,:)', 'LineWidth', line_t)
axis(axis_range)
title('FR foot GCF')
ylabel('force (N)')
xlabel('time (s)')

subplot(2,2,3)
plot(data.t(:), data.ug(10:12,:)', 'LineWidth', line_t)
axis(axis_range)
title('HL foot GCF')
ylabel('force (N)')
xlabel('time (s)')

subplot(2,2,4)
plot(data.t(:), data.ug(7:9,:)', 'LineWidth', line_t)
axis(axis_range)
title('HR foot GCF')
ylabel('force (N)')
xlabel('time (s)')



% External force and MBDO =================================================
% 
% figure(5)
% 
% subplot(3,2,1), hold on
% plot(data.t(:), data.MBDO(1,:), 'LineWidth', line_t)
% plot(data.t(:), data.ue(1,:),'-.', 'LineWidth', line_t)
% title('MBDO vs ext. force x')
% ylabel('force (N)')
% axis([-inf,inf,-20,5])
% 
% subplot(3,2,3), hold on
% plot(data.t(:), data.MBDO(2,:), 'LineWidth', line_t)
% plot(data.t(:), data.ue(2,:),'-.', 'LineWidth', line_t)
% title('MBDO vs ext. force y')
% ylabel('force (N)')
% axis([-inf,inf,-5,5])
% 
% subplot(3,2,5), hold on
% plot(data.t(:), data.MBDO(3,:), 'LineWidth', line_t)
% plot(data.t(:), data.ue(3,:),'-.', 'LineWidth', line_t)
% title('MBDO vs ext. force z')
% ylabel('force (N)')
% xlabel('time (s)')
% axis([-inf,inf,-5,5])
% 
% subplot(3,2,2), hold on
% plot(data.t(:), data.MBDO(4,:), 'LineWidth', line_t)
% plot(data.t(:), data.ue(4,:),'-.', 'LineWidth', line_t)
% title('MBDO vs ext. torque x')
% ylabel('torque (N.m)')
% axis([-inf,inf,-4,4])
% 
% subplot(3,2,4), hold on
% plot(data.t(:), data.MBDO(5,:), 'LineWidth', line_t)
% plot(data.t(:), data.ue(5,:),'-.', 'LineWidth', line_t)
% title('MBDO vs ext. torque y')
% ylabel('torque (N.m)')
% axis([-inf,inf,-4,4])
% 
% subplot(3,2,6), hold on
% plot(data.t(:), data.MBDO(6,:), 'LineWidth', line_t)
% plot(data.t(:), data.ue(6,:),'-.', 'LineWidth', line_t)
% title('MBDO vs ext. torque z')
% ylabel('torque (N.m)')
% xlabel('time (s)')
% axis([-inf,inf,-4,4])

% Leg deflections =========================================================
% 
% figure(6)
% 
% subplot(2,2,1)
% plot(data.t(:), data.x([44,48],:)'*1000, 'LineWidth', line_t), hold on
% % plot(data.t(:), data.del_ref([2,6],:)'*1000,'-.', 'LineWidth', line_t)
% title('Compliant Deflection FL')
% legend('dx', 'dy')
% axis([-inf,inf,-20,20])
% ylabel('disp (mm)')
% xlabel('time (s)')
% 
% subplot(2,2,2)
% plot(data.t(:), data.x([43,47],:)'*1000, 'LineWidth', line_t), hold on
% % plot(data.t(:), data.del_ref([1,5],:)'*1000,'-.', 'LineWidth', line_t)
% title('Compliant Deflection FR')
% axis([-inf,inf,-20,20])
% ylabel('disp (mm)')
% xlabel('time (s)')
% 
% subplot(2,2,3)
% plot(data.t(:), data.x([46,50],:)'*1000, 'LineWidth', line_t), hold on
% % plot(data.t(:), data.del_ref([4,8],:)'*1000,'-.', 'LineWidth', line_t)
% title('Compliant Deflection HL')
% axis([-inf,inf,-20,20])
% ylabel('disp (mm)')
% xlabel('time (s)')
% 
% subplot(2,2,4)
% plot(data.t(:), data.x([45,49],:)'*1000, 'LineWidth', line_t), hold on
% % plot(data.t(:), data.del_ref([3,7],:)'*1000,'-.', 'LineWidth', line_t)
% title('Compliant Deflection HR')
% axis([-inf,inf,-20,20])
% ylabel('disp (mm)')
% xlabel('time (s)')

%%
if use_ERG==1
    figure()
    subplot(321)
    % plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    plot(data.t, data.x(13,:), data.t, data.xc_ref(1,:), 'r--',data.t, data.xw_ref(1,:), 'b-')
    title('xref');
    legend('x','xrref','xwref');
    subplot(322)
    % plot(data.t, data.x(37:39,:), data.t, data.xcd_ref, 'r--',data.t, data.xwd_ref, 'b--')
    plot(data.t, data.x(37,:), data.t, data.xcd_ref(1,:), 'r--',data.t, data.xwd_ref(1,:), 'b-')
    axis([-inf,inf,-1,1])
    title('xdref');
    legend('xd','xdrref','xwdref');
    subplot(323)
    % plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    plot(data.t, data.x(14,:), data.t, data.xc_ref(2,:), 'r--',data.t, data.xw_ref(2,:), 'b-')
    title('yref');
    legend('x','yrref','ywref');
    subplot(324)
    % plot(data.t, data.x(37:39,:), data.t, data.xcd_ref, 'r--',data.t, data.xwd_ref, 'b--')
    plot(data.t, data.x(38,:), data.t, data.xcd_ref(2,:), 'r--',data.t, data.xwd_ref(2,:), 'b-')
    axis([-inf,inf,-1,1])
    title('ydref');
    legend('yd','ydrref','ydwref');
    subplot(325)
    % plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    plot(data.t, data.x(15,:), data.t, data.xc_ref(3,:), 'r--',data.t, data.xw_ref(3,:), 'b-')
    title('zref');
    legend('z','zrref','zwref');
    subplot(326)
    % plot(data.t, data.x(37:39,:), data.t, data.xcd_ref, 'r--',data.t, data.xwd_ref, 'b--')
    plot(data.t, data.x(39,:), data.t, data.xcd_ref(3,:), 'r--',data.t, data.xwd_ref(3,:), 'b-')
    axis([-inf,inf,-1,1])
    title('zdref');
    legend('zd','zdrref','zdwref');

%%
    figure()
    
    subplot(321)

    plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    plot(data.t, data.hr(1,:),'r--', data.t, data.hw(1,:),'b-')
    title('h');
    legend('hr','hw');
    axis([-inf,inf,-50,50]);grid on
    subplot(322)
    % plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    plot(data.t, data.hr(2,:),'r--', data.t, data.hw(2,:),'b-')
    title('h');
    legend('hr','hw');
    axis([-inf,inf,-50,50]);grid on
    subplot(323)
    % plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    plot(data.t, data.hr(3,:),'r--', data.t, data.hw(3,:),'b-')
    title('h');
    legend('hr','hw');
    axis([-inf,inf,-50,50]);grid on
    subplot(324)
    % plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    plot(data.t, data.hr(4,:),'r--', data.t, data.hw(4,:),'b-')
    title('h');
    legend('hr','hw');axis([-inf,inf,-50,50]);grid on
    subplot(325)
    % plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    plot(data.t, data.hr(5,:),'r--', data.t, data.hw(5,:),'b-')
    title('h');
    legend('hr','hw');axis([-inf,inf,-50,50]);grid on
    subplot(326)
    % plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    plot(data.t, data.hr(6,:),'r--', data.t, data.hw(6,:),'b-')
    title('h');
    legend('hr','hw');axis([-inf,inf,-50,50]);grid on

    figure()
    grid on
    % plot(data.t, data.x(13:15,:), data.t, data.xc_ref, 'r--',data.t, data.xw_ref, 'b--')
    subplot(121)
    plot(data.t, data.fe(1:3,:));
    axis([-inf,inf,-50,50]);
    title('fe');
    subplot(122)
    plot(data.t, data.fe(4:6,:));
    axis([-inf,inf,-50,50]);
    title('fe');
end
figure()
subplot(511)
plot(data.t, data.TD(1,:));axis([-inf,inf,0,1.2]);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('FR');
subplot(512)
plot(data.t, data.TD(2,:));axis([-inf,inf,0,1.2]);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('FL');
subplot(513)
plot(data.t, data.TD(3,:));axis([-inf,inf,0,1.2]);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('BR');
subplot(514)
plot(data.t, data.TD(4,:));axis([-inf,inf,0,1.2]);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('BL');
subplot(515)
plot(data.t, data.s);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('s');

figure()
subplot(511)
plot(data.t, data.leg_states(1,:));axis([-inf,inf,0,2.2]);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('leg state FR');
subplot(512)
plot(data.t, data.leg_states(2,:));axis([-inf,inf,0,2.2]);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('leg state FL');
subplot(513)
plot(data.t, data.leg_states(3,:));axis([-inf,inf,0,2.2]);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('leg state BR');
subplot(514)
plot(data.t, data.leg_states(4,:));axis([-inf,inf,0,2.2]);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('leg state BL');
subplot(515)
plot(data.t, data.s);
line([p.gait_t0,p.gait_t0],[0,1],'Color','red')
title('s');

%%
figure()

subplot(3,1,1)
plot(data.t, smooth(data.ug_wrench(1,:)), 'LineWidth',1.5);
hold on
plot(data.t, data.rk(1,:), 'LineWidth',1.5);
title('Fx')
ylabel('N')
legend('Actual', 'Estimated','Orientation','horizontal');

subplot(3,1,2)
plot(data.t, smooth(data.ug_wrench(2,:)),'LineWidth',1.5);
hold on 
plot(data.t, data.rk(2,:), 'LineWidth',1.5);
title('Fy')
ylabel("N")
legend('Actual', 'Estimated','Orientation','horizontal');
subplot(3,1,3)
plot(data.t, smooth(data.ug_wrench(3,:)), 'LineWidth',1.5);
hold on 
plot(data.t, data.rk(3,:), 'LineWidth',1.5);
ylabel('N')
title('Fz')
xlabel('Time (s)')
legend('Actual', 'Estimated','Orientation','horizontal');

sgtitle('Estimated total Generalized Forces vs Actual Total Forces')

figure()

subplot(3,1,1)
plot(data.t, smooth(data.ug_wrench(4,:)), 'LineWidth',1.5);
hold on
plot(data.t, data.rk(4,:), 'LineWidth',1.5);
title('Fx')
ylabel('N')
legend('Actual', 'Estimated','Orientation','horizontal');

subplot(3,1,2)
plot(data.t, smooth(data.ug_wrench(5,:)),'LineWidth',1.5);
hold on 
plot(data.t, data.rk(5,:), 'LineWidth',1.5);
title('Fy')
ylabel("N")
legend('Actual', 'Estimated','Orientation','horizontal');

subplot(3,1,3)
plot(data.t, smooth(data.ug_wrench(6,:)), 'LineWidth',1.5);
hold on 
plot(data.t, data.rk(6,:), 'LineWidth',1.5);
ylabel('N')
title('Fz')
xlabel('Time (s)')
legend('Actual', 'Estimated','Orientation','horizontal');

sgtitle('Estimated total Generalized Torques vs Actual Total Torques')