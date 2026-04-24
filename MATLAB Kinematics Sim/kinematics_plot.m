%% 1. Define Geometry
L  = 0.09125;    % Top platform radius
L1 = 0.075;      % Upper link length
L2 = 0.05;       % Lower link length
L3 = 0.075;      % Base radius

% Inputs
h = 0.10;        % Platform Height
tilt_theta = 0;  % Tilt magnitude
tilt_phi = 0;    % Tilt direction

tilt_theta_rad = deg2rad(tilt_theta);
tilt_phi_rad = deg2rad(tilt_phi);

alpha = sin(tilt_theta_rad) * cos(tilt_phi_rad);
beta = sin(tilt_theta_rad) * sin(tilt_phi_rad);
gamma = cos(tilt_theta_rad);

n = [alpha, beta, gamma]; % Normal vector

%% 2. Calculate Motor Angles
angle1 = leg1_angle(n, h, L, L1, L2, L3);
angle2 = leg2_angle(n, h, L, L1, L2, L3);
angle3 = leg3_angle(n, h, L, L1, L2, L3);

thetas_deg = [angle1, angle2, angle3];
thetas_rad = deg2rad(thetas_deg);
disp(thetas_deg)

if any(isnan(thetas_deg))
    disp('IK resulted in NaN');
    return;
end

%% 3. Joint Coordinates
% --- Motor Coordinates ---
theta_base = [0, 2*pi/3, 4*pi/3];

XB = L3 * cos(theta_base);
YB = L3 * sin(theta_base);
ZB = zeros(1, 3);

% --- Resolve L2 to Horizontal/Vertical ---
R_hori = L2 * cos(thetas_rad);
R_vert = L2 * sin(thetas_rad);

% --- Revolute Joints XYZ ---
XR = XB + R_hori .* cos(theta_base);
YR = YB + R_hori .* sin(theta_base);
ZR = ZB + R_vert;

%% 4. Plot
figure('Position', [100, 100, 1000, 800]); 
hold on;
grid on;
axis equal; 
view(40, 30);

% --- Base ---
plot3([XB, XB(1)], [YB, YB(1)], [ZB, ZB(1)], 'k--', 'LineWidth', 2, 'DisplayName', 'Base');
scatter3(XB, YB, ZB, 80, 'k', 'filled', 'DisplayName', 'Motors');

% --- L2 at Given Motor Angle ---
for i = 1:3
    % Plot line from Base Point to Revolute Point
    plot3([XB(i), XR(i)], [YB(i), YR(i)], [ZB(i), ZR(i)], 'b-', 'LineWidth', 3, 'DisplayName', ['Leg ', num2str(i),]); 
end

% --- Revolute Joints ---
scatter3(XR, YR, ZR, 50, 'r', 'filled', 'DisplayName', 'Revolute Joints');
patch(XR, YR, ZR, 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Tilt Plane');

xlabel('X-axis (m)');
ylabel('Y-axis (m)');
zlabel('Z-axis (m)');
title(['Height = ', num2str(h, 3), 'm,  Tilt = ', num2str(tilt_theta), ' deg,  Direction = ', num2str(tilt_phi), ' deg,  Motor Angles: ', num2str([angle1, angle2, angle3], 3), ' deg'], 'FontSize', 14);
legend('-DynamicLegend', 'Location', 'Best');
hold off;

%% https://uk.mathworks.com/help/matlab/ref/plot3.html