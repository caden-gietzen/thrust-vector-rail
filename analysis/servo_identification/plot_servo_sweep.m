% plot_servo_sweep.m
% Plot servo sweep data from Pico CSV log.
% Expected CSV columns: t_ms, t_s, servo_us, count

clear; clc; close all;

% ----------------------------
% User-selected data file
% ----------------------------

dataFile = "servo_sweep_log.csv";

% ----------------------------
% Locate project paths
% ----------------------------

repoRoot = findRepoRoot();

dataPath = fullfile(repoRoot, "data", "raw", "servo_identification", "servo_sweep", dataFile);
plotDir = fullfile(repoRoot, "plots", "servo_identification");

if ~isfolder(plotDir)
    mkdir(plotDir);
end

% ----------------------------
% Load data
% ----------------------------

T = readtable(dataPath);

t = T.t_s;
servo_us = T.servo_us;
count = T.count;

count_zeroed = count - count(1);

% ----------------------------
% Optional command-to-angle estimate
% ----------------------------

SERVO_CENTER_US = 1450;
K_THETA_RAD_PER_US = 0.00157;
RAD_TO_DEG = 180/pi;

theta_cmd_rad = (servo_us - SERVO_CENTER_US) * K_THETA_RAD_PER_US;
theta_cmd_deg = theta_cmd_rad * RAD_TO_DEG;

% ----------------------------
% Plot servo command vs time
% ----------------------------

figure;
plot(t, servo_us, "LineWidth", 1.5);
grid on;
xlabel("Time (s)");
ylabel("Servo Command (\mus)");
title("Servo Command vs Time");

% ----------------------------
% Plot encoder count vs time
% ----------------------------

figure;
plot(t, count_zeroed, "LineWidth", 1.5);
grid on;
xlabel("Time (s)");
ylabel("Encoder Count - Initial Count");
title("Encoder Count vs Time");

% ----------------------------
% Plot static sweep
% ----------------------------

figure;
plot(servo_us, count_zeroed, "o-", "LineWidth", 1.2);
grid on;
xlabel("Servo Command (\mus)");
ylabel("Encoder Count - Initial Count");
title("Static Servo Sweep: Command vs Encoder Count");

% ----------------------------
% Plot approximate angle relationship
% ----------------------------

figure;
plot(theta_cmd_deg, count_zeroed, "o-", "LineWidth", 1.2);
grid on;
xlabel("Approx. Commanded Servo Angle (deg)");
ylabel("Encoder Count - Initial Count");
title("Static Servo Sweep: Commanded Angle vs Encoder Count");

% ----------------------------
% Linear fit
% ----------------------------

p = polyfit(servo_us, count_zeroed, 1);
count_fit = polyval(p, servo_us);

figure;
plot(servo_us, count_zeroed, "o", "LineWidth", 1.2);
hold on;
plot(servo_us, count_fit, "-", "LineWidth", 1.5);
grid on;
xlabel("Servo Command (\mus)");
ylabel("Encoder Count - Initial Count");
title("Linear Fit: Servo Command to Encoder Count");
legend("Measured", "Linear Fit", "Location", "best");

% ----------------------------
% Print summary
% ----------------------------

fprintf("\nServo sweep summary:\n");
fprintf("Data file: %s\n", dataPath);
fprintf("Samples: %d\n", height(T));
fprintf("Servo command range: %.1f to %.1f us\n", min(servo_us), max(servo_us));
fprintf("Encoder count range: %.1f to %.1f counts\n", min(count_zeroed), max(count_zeroed));
fprintf("Linear fit: count = %.6f * servo_us + %.6f\n", p(1), p(2));
fprintf("Counts per microsecond: %.6f\n", p(1));

p_angle = polyfit(theta_cmd_deg, count_zeroed, 1);
fprintf("Approx counts per commanded degree: %.6f\n", p_angle(1));

% ----------------------------
% Helper function
% ----------------------------

function repoRoot = findRepoRoot()
    currentDir = fileparts(mfilename("fullpath"));

    while true
        gitDir = fullfile(currentDir, ".git");

        if isfolder(gitDir)
            repoRoot = currentDir;
            return;
        end

        parentDir = fileparts(currentDir);

        if strcmp(parentDir, currentDir)
            error("Could not find repo root. No .git folder found above this script.");
        end

        currentDir = parentDir;
    end
end