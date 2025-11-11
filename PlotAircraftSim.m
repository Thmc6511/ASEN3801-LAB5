function UpdatedAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
% PlotAircraftSim — Lab 5 version
% - Control inputs: δe, δa, δr plotted in degrees; δt in [0–1] fraction.
% - Keeps your existing figure layout and colors.

t = time(:);
n = numel(t);

% Ensure arrays are [state_or_input x time]
if size(aircraft_state_array,2) ~= n
    aircraft_state_array = aircraft_state_array.';
end
if size(control_input_array,2) ~= n
    control_input_array = control_input_array.';
end

% Expecting controls as [δe; δa; δr; δt] with surfaces in radians.
de_deg = rad2deg(control_input_array(1,:));
da_deg = rad2deg(control_input_array(2,:));
dr_deg = rad2deg(control_input_array(3,:));
dt = control_input_array(4,:);

%dt = max(0, min(1, dt));

%% Figure 1 – Inertial Position
figure(fig(1));
subplot(3,1,1); hold on; plot(t, aircraft_state_array(1,:), col); ylabel('x (m)'); grid on;
subplot(3,1,2); hold on; plot(t, aircraft_state_array(2,:), col); ylabel('y (m)'); grid on;
subplot(3,1,3); hold on; plot(t, aircraft_state_array(3,:), col); ylabel('z (m)'); xlabel('Time (s)'); grid on;
sgtitle('Inertial Position (Fixed-Wing)');

%% Figure 2 – Euler Angles
figure(fig(2));
subplot(3,1,1); hold on; plot(t, aircraft_state_array(4,:), col); ylabel('\phi (rad)'); grid on;
subplot(3,1,2); hold on; plot(t, aircraft_state_array(5,:), col); ylabel('\theta (rad)'); grid on;
subplot(3,1,3); hold on; plot(t, aircraft_state_array(6,:), col); ylabel('\psi (rad)'); xlabel('Time (s)'); grid on;
sgtitle('Euler Angles (Fixed-Wing)');

%% Figure 3 – Body-Frame Velocity
figure(fig(3));
subplot(3,1,1); hold on; plot(t, aircraft_state_array(7,:), col); ylabel('u (m/s)'); grid on;
subplot(3,1,2); hold on; plot(t, aircraft_state_array(8,:), col); ylabel('v (m/s)'); grid on;
subplot(3,1,3); hold on; plot(t, aircraft_state_array(9,:), col); ylabel('w (m/s)'); xlabel('Time (s)'); grid on;
sgtitle('Body-Frame Velocity (Fixed-Wing)');

%% Figure 4 – Angular Velocity
figure(fig(4));
subplot(3,1,1); hold on; plot(t, aircraft_state_array(10,:), col); ylabel('p (rad/s)'); grid on;
subplot(3,1,2); hold on; plot(t, aircraft_state_array(11,:), col); ylabel('q (rad/s)'); grid on;
subplot(3,1,3); hold on; plot(t, aircraft_state_array(12,:), col); ylabel('r (rad/s)'); xlabel('Time (s)'); grid on;
sgtitle('Angular Velocity (Fixed-Wing)');

%% Figure 5 – Control Inputs (per Lab 5)
figure(fig(5));
subplot(4,1,1); hold on; plot(t, de_deg, col); ylabel('\delta_e (deg)'); grid on;
subplot(4,1,2); hold on; plot(t, da_deg, col); ylabel('\delta_a (deg)'); grid on;
subplot(4,1,3); hold on; plot(t, dr_deg, col); ylabel('\delta_r (deg)'); grid on;
subplot(4,1,4); hold on; plot(t, dt,     col); ylabel('\delta_t (0–1)'); xlabel('Time (s)'); grid on;
sgtitle('Control Inputs: Surfaces in Degrees, Throttle as Fraction');

%% Figure 6 – 3D Path
figure(fig(6));
hold on;
plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), aircraft_state_array(3,:), col, 'LineWidth', 1.2);
plot3(aircraft_state_array(1,1), aircraft_state_array(2,1), aircraft_state_array(3,1), 'go', 'MarkerFaceColor','g');
plot3(aircraft_state_array(1,end), aircraft_state_array(2,end), aircraft_state_array(3,end), 'ro', 'MarkerFaceColor','r');
grid on; axis vis3d; xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); view(3);
title('3D Aircraft Path (Fixed-Wing)');
end
