function PlotAircraftSim(time, aircraft_state_array, control_input_array,fig, col)

%n = size(time);

x = aircraft_state_array(:,1);
y = aircraft_state_array(:,2);
z = aircraft_state_array(:,3);


% intertial position vectors
figure(fig(1));
subplot(311);
plot(time, x, col); 
xlabel ('Time (s)');
ylabel ('X-Position');
title ('Inertial Position [x] vs. Time');
hold on;
subplot(312);
plot(time, y, col); 
xlabel ('Time (s)');
ylabel ('Y-Position');
title ('Inertial Position [y] vs. Time');
hold on;
subplot(313);
plot(time, z, col); 
xlabel ('Time (s)');
ylabel ('Z-Position');
title ('Inertial Position [z] vs. Time');
hold on;


% Euler angles in body frame
figure(fig(2));
subplot(311);
plot(time, aircraft_state_array(:,4), col); 
xlabel ('Time (s)');
ylabel ('Roll Angle');
title ('Euler Angle [\phi] along body x-axis vs. Time');
hold on;
subplot(312);
plot(time, aircraft_state_array(:,5), col); 
xlabel ('Time (s)');
ylabel ('Pitch Angle');
title ('Euler Angle [\theta] along body y-axis vs. Time');
hold on;
subplot(313);
plot(time, aircraft_state_array(:,6), col); 
xlabel ('Time (s)');
ylabel ('Yaw Angle');
title ('Euler Angle [\psi] along body z-axis vs. Time');
hold on;

% Velocities inertial frame
figure(fig(3));
subplot(311);
plot(time, aircraft_state_array(:,7), col); 
xlabel ('Time (s)');
ylabel ('X-velocity');
title ('Inertial X-Velocity [u] vs. Time');
hold on;
subplot(312);
plot(time, aircraft_state_array(:,8), col); 
xlabel ('Time (s)');
ylabel ('Y-velocity');
title ('Inertial Y-Velocity [v] vs. Time');
hold on;
subplot(313);
plot(time, aircraft_state_array(:,9), col); 
xlabel ('Time (s)');
ylabel ('Z-velocity');
title ('Inertial Z-Velocity [w] vs. Time');
hold on;


% roll, pitch, yaw rates in body frame
figure(fig(4));
subplot(311);
plot(time, aircraft_state_array(:,10), col); 
xlabel ('Time (s)');
ylabel ('Roll Rate');
title ('Roll Rate [p] along body x-axis vs. Time');
hold on;
subplot(312);
plot(time, aircraft_state_array(:,11), col); 
xlabel ('Time (s)');
ylabel ('Pitch Rate');
title ('Pitch Rate [q] along body y-axis vs. Time');
hold on;
subplot(313);
plot(time, aircraft_state_array(:,12), col); 
xlabel ('Time (s)');
ylabel ('Yaw Rate');
title ('Yaw Rate [r] along body z-axis vs. Time');
hold on;

% Figure for control inputs in body frame
figure(fig(5));
subplot(411)
plot(time, control_input_array(:,1), col)
xlabel ('Time (s)');
ylabel ('Thrust Force (Zc)');
title ('Total Thrust Force [Zc] along body z-axis vs. Time');
hold on;
subplot(412)
plot(time, control_input_array(:,2), col)
xlabel ('Time (s)');
ylabel ('Roll Moment (Lc)');
title ('Roll Moment [Lc] along body x-axis vs. Time');
hold on;
subplot(413)
plot(time, control_input_array(:,3), col)
xlabel ('Time (s)');
ylabel ('Pitch Moment (Mc)');
title ('Pitch Moment [Mc] along body y-axis vs. Time');
hold on;
subplot(414)
plot(time, control_input_array(:,4), col)
xlabel ('Time (s)');
ylabel ('Yaw Moment (Nc)');
title ('Yaw Moment [Nc] along body z-axis vs. Time');

% 3D plot of the motion path at hovering state
figure(fig(6));
plot3(x, y, z);
xlabel ('X-Position');
ylabel ('Y-Position');
zlabel ('Z-Position')
title ('3D Plot representing [x,y,z] positions of Quadrotor at Hovering State');


end

%[phi_dot, theta_dot, psi_dot] = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); 0 cos(phi) -sin(phi); 0 sin(phi)*sec(theta) cos(phi)*sec(theta)]*[p;
% q; r];