% build_pid_model.m
%
% Programmatically creates position_pid.slx: a SISO PID position controller
% with the nonlinear plant (same MATLAB Function as the LQR models).
%
% Plant:   x = [p; v; theta; T],  u = [theta_cmd_rad; T_cmd_N]
% Loop:    error = p_ref - p  →  PID  →  theta_cmd_rad  →  plant
% Thrust:  fixed at T_nom = 2.574 N (feedforward constant)
%
% After running:
%   1. open_system('<path>/position_pid')
%   2. Right-click the PID block → Tune
%   3. PID Tuner linearizes the nonlinear plant; drag bandwidth to 0.3–0.5 Hz
%      (servo identification limits useful closed-loop bandwidth to ≤ 1 Hz)

here       = fileparts(mfilename('fullpath'));
model_name = 'position_pid';
save_path  = fullfile(here, model_name);

%% Create Model
if bdIsLoaded(model_name); close_system(model_name, 0); end
new_system(model_name);
set_param(model_name, ...
    'Solver',   'ode45', ...
    'StopTime', '3',    ...
    'MaxStep',  '0.005');
set_param(model_name, 'InitFcn', ...
    'addpath(fileparts(get_param(bdroot, ''FileName'')))');

%% Add Blocks

% Reference
add_block('simulink/Sources/Constant', [model_name '/p_ref'], ...
    'Position', [30, 165, 80, 185], 'Value', '0');

% Error sum: p_ref - p
add_block('simulink/Math Operations/Sum', [model_name '/sum_error'], ...
    'Position', [110, 162, 135, 188], 'Inputs', '+-');

% PID Controller (tunable via PID Tuner app — right-click → Tune)
add_block('simulink/Continuous/PID Controller', [model_name '/PID'], ...
    'Position', [160, 148, 315, 208]);

% Servo saturation: ±60°
add_block('simulink/Discontinuities/Saturation', [model_name '/sat_servo'], ...
    'Position', [345, 158, 405, 198], ...
    'UpperLimit', 'pi/3', 'LowerLimit', '-pi/3');

% Thrust feedforward
add_block('simulink/Sources/Constant', [model_name '/T_nom'], ...
    'Position', [345, 228, 405, 252], 'Value', '2.574');

% Mux: u = [theta_cmd_rad; T_nom]
add_block('simulink/Signal Routing/Mux', [model_name '/Mux_u'], ...
    'Position', [435, 153, 450, 258], 'Inputs', '2');

% Nonlinear plant (EOM identical to LQR models)
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/plant'], ...
    'Position', [480, 120, 640, 220]);

% State integrator — IC: 150 mm displaced, at rest, servo 0°, nominal thrust
add_block('simulink/Continuous/Integrator', [model_name '/integrator'], ...
    'Position',         [480, 290, 560, 330], ...
    'InitialCondition', '[0.15; 0; 0; 2.574]');

% State demux
add_block('simulink/Signal Routing/Demux', [model_name '/Demux_x'], ...
    'Position', [610, 285, 625, 345], 'Outputs', '4');

% Scope: position with reference overlay
add_block('simulink/Signal Routing/Mux', [model_name '/Mux_p'], ...
    'Position', [700, 268, 710, 292], 'Inputs', '2');

add_block('simulink/Sinks/Scope', [model_name '/Scope_states'], ...
    'Position', [780, 258, 840, 372], 'NumInputPorts', '4');
add_block('simulink/Sinks/Scope', [model_name '/Scope_inputs'], ...
    'Position', [680, 115, 740, 185], 'NumInputPorts', '2');

%% Set Plant MATLAB Function Script
set_mf_script(model_name, 'plant', plant_code());

%% Wire — Control Path
add_line(model_name, 'p_ref/1',     'sum_error/1', 'autorouting', 'smart');
add_line(model_name, 'sum_error/1', 'PID/1',        'autorouting', 'smart');
add_line(model_name, 'PID/1',       'sat_servo/1',  'autorouting', 'smart');
add_line(model_name, 'sat_servo/1', 'Mux_u/1',      'autorouting', 'smart');
add_line(model_name, 'T_nom/1',     'Mux_u/2',      'autorouting', 'smart');
add_line(model_name, 'Mux_u/1',     'plant/2',      'autorouting', 'smart');

%% Wire — Plant Loop
add_line(model_name, 'integrator/1', 'plant/1',      'autorouting', 'smart');
add_line(model_name, 'plant/1',      'integrator/1', 'autorouting', 'smart');
add_line(model_name, 'integrator/1', 'Demux_x/1',    'autorouting', 'smart');

%% Wire — Feedback
add_line(model_name, 'Demux_x/1', 'sum_error/2', 'autorouting', 'smart');

%% Wire — Scopes
lh = add_line(model_name, 'Demux_x/1',   'Mux_p/1', 'autorouting', 'smart');
set_param(lh, 'Name', 'p (m)');
lh = add_line(model_name, 'p_ref/1',     'Mux_p/2', 'autorouting', 'smart');
set_param(lh, 'Name', 'p_{ref} (m)');
add_line(model_name, 'Mux_p/1', 'Scope_states/1', 'autorouting', 'smart');

lh = add_line(model_name, 'Demux_x/2', 'Scope_states/2', 'autorouting', 'smart');
set_param(lh, 'Name', 'v (m/s)');
lh = add_line(model_name, 'Demux_x/3', 'Scope_states/3', 'autorouting', 'smart');
set_param(lh, 'Name', 'theta (rad)');
lh = add_line(model_name, 'Demux_x/4', 'Scope_states/4', 'autorouting', 'smart');
set_param(lh, 'Name', 'T (N)');

lh = add_line(model_name, 'sat_servo/1', 'Scope_inputs/1', 'autorouting', 'smart');
set_param(lh, 'Name', 'u_{servo} (rad)');
lh = add_line(model_name, 'T_nom/1',     'Scope_inputs/2', 'autorouting', 'smart');
set_param(lh, 'Name', 'u_{thrust} (N)');

set_param([model_name '/Scope_states'], 'ShowLegend', 'on');
set_param([model_name '/Scope_inputs'], 'ShowLegend', 'on');

%% Save
save_system(model_name, save_path);
fprintf('Saved: %s.slx\n\n', save_path);
fprintf('To tune:\n');
fprintf('  open_system(''%s'')\n', save_path);
fprintf('  Right-click PID block → Tune  (aim for 0.3–0.5 Hz bandwidth)\n');

%% ── Helpers ──────────────────────────────────────────────────────────────

function set_mf_script(mdl, blk, script)
    rt     = sfroot;
    obj    = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', mdl);
    charts = obj.find('-isa', 'Stateflow.EMChart');
    for k = 1:numel(charts)
        parts = strsplit(charts(k).Path, '/');
        if strcmp(parts{end}, blk)
            charts(k).Script = script;
            return;
        end
    end
    error('MATLAB Function block "%s" not found in "%s"', blk, mdl);
end

function s = plant_code()
lines = {
    'function xdot = plant(x, u)'
    '% Nonlinear EOM: x = [p; v; theta; T],  u = [theta_cmd_rad; T_cmd_N]'
    'm          = 0.4536;'
    'tau_servo  = 0.0244;'
    'tau_thrust = 0.0781;'
    'mu_c       = 0.8158;'
    'v_sign = tanh(x(2) / 0.05);   % smooth sign: avoids chattering at v≈0 (ε = 5 mm/s)'
    'xdot = [x(2);'
    '        (x(4)*sin(x(3)) - mu_c*v_sign) / m;'
    '        (u(1) - x(3)) / tau_servo;'
    '        (u(2) - x(4)) / tau_thrust];'
    'end'
};
s = strjoin(lines, newline);
end
