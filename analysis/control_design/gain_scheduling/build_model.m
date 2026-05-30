% build_model.m
%
% Programmatically creates Simulink models for all LQR tiers.
% Run trim_analysis.m first to generate the .mat files, then run this
% script once. Open any resulting .slx to simulate.
%
%   Tier 1 → fixed_lqr.slx         (fixed-gain, no scheduling input)
%   Tier 2 → theta_sched_lqr.slx   (gain scheduled on theta)
%   Tier 3 → pair_sched_lqr.slx    (gain scheduled on theta + T)
%
% Model structure (Tier 2 shown):
%
%   p_ref ──────────────────────────────────────────────────────────────────┐
%   theta ───────────────────────────────────────────────┐                  │
%                                                         │                  │
%   x ──────────────────────── [controller] ──► u ──► [plant] ──► xdot     │
%   │                                                          │             │
%   │   [Scope_inputs] ◄── u              [integrator] ◄──────┘             │
%   └──── [Demux_x] ──► [Scope_states]                                ──────┘

%% Tier Configurations
% Edit ic_str here to change simulation initial conditions.
% x = [p (m); v (m/s); theta (rad); T (N)]
tier_configs = {
    struct('tier', 1, 'model_name', 'fixed_lqr',      'mat_file', 'gain_schedule_tier1.mat', 'ic_str', '[0.15; 0; 0; 2.574]');  % displaced position, at rest, theta*=0, nominal thrust
    struct('tier', 2, 'model_name', 'theta_sched_lqr', 'mat_file', 'gain_schedule_tier2.mat', 'ic_str', '[0.15; 0; 0; 2.574]');  % displaced, at rest, nominal thrust
    struct('tier', 3, 'model_name', 'pair_sched_lqr',  'mat_file', 'gain_schedule_tier3.mat', 'ic_str', '[0.15; 0; 0; 2.574]');  % at rest, nominal thrust
};

%% Reference Signal
% p_ref(t) = p_bias + p_amp * sin(2*pi*p_freq * t + p_phase)
% Set p_amp = 0 for regulation (step from IC to origin); nonzero for tracking.
p_amp   = 0.0;   % m    — sinusoid amplitude
p_freq  = 0.5;   % Hz   — sinusoid frequency
p_phase = 0.0;   % rad  — sinusoid phase offset
p_bias  = 0.0;   % m    — DC offset (steady-state position target)

here = fileparts(mfilename('fullpath'));

for ti = 1:numel(tier_configs)
    cfg        = tier_configs{ti};
    TIER       = cfg.tier;
    model_name = cfg.model_name;
    mat_file   = cfg.mat_file;
    ic_str     = cfg.ic_str;

    mat_path  = fullfile(here, mat_file);
    save_path = fullfile(here, model_name);

    if ~isfile(mat_path)
        error('%s not found — run trim_analysis.m first.', mat_file);
    end

    %% Create Model
    if bdIsLoaded(model_name); close_system(model_name, 0); end
    new_system(model_name);
    set_param(model_name, ...
        'Solver',   'ode45', ...
        'StopTime', '3',    ...
        'MaxStep',  '0.005');

    % Add model directory to path so coder.load can find gain_schedule_tier{N}.mat
    set_param(model_name, 'InitFcn', ...
        'addpath(fileparts(get_param(bdroot, ''FileName'')))');

    %% Add Shared Blocks
    add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/controller'], ...
        'Position', [235 120 395 220]);

    add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/plant'], ...
        'Position', [495 120 655 220]);

    add_block('simulink/Continuous/Integrator', [model_name '/integrator'], ...
        'Position',         [495 290 575 330], ...
        'InitialCondition', ic_str);

    add_block('simulink/Signal Routing/Demux', [model_name '/Demux_x'], ...
        'Position', [625 285 640 345], 'Outputs', '4');
    add_block('simulink/Signal Routing/Demux', [model_name '/Demux_u'], ...
        'Position', [625 135 640 165], 'Outputs', '2');

    add_block('simulink/Sources/Sine Wave', [model_name '/p_ref'], ...
        'Position',   [55 375 115 405], ...
        'Amplitude',  num2str(p_amp),        ...
        'Frequency',  num2str(2*pi*p_freq),  ...
        'Phase',      num2str(p_phase),      ...
        'Bias',       num2str(p_bias),       ...
        'SampleTime', '0');

    add_block('simulink/Signal Routing/Mux', [model_name '/Mux_p'], ...
        'Position', [715 268 725 292], 'Inputs', '2');

    add_block('simulink/Sinks/Scope', [model_name '/Scope_states'], ...
        'Position', [800 258 860 372], 'NumInputPorts', '4');
    add_block('simulink/Sinks/Scope', [model_name '/Scope_inputs'], ...
        'Position', [700 115 760 185], 'NumInputPorts', '2');

    %% Set MATLAB Function Scripts
    set_mf_script(model_name, 'plant',      plant_code());
    set_mf_script(model_name, 'controller', controller_code(TIER, mat_file));

    %% Wire — Tier-Specific
    if TIER == 1
        add_line(model_name, 'integrator/1', 'controller/1', 'autorouting', 'smart');
        add_line(model_name, 'p_ref/1',      'controller/2', 'autorouting', 'smart');
        add_line(model_name, 'controller/1', 'plant/2',      'autorouting', 'smart');
        add_line(model_name, 'integrator/1', 'plant/1',      'autorouting', 'smart');
        add_line(model_name, 'plant/1',      'integrator/1', 'autorouting', 'smart');

        lh = add_line(model_name, 'Demux_x/3', 'Scope_states/3', 'autorouting', 'smart');
        set_param(lh, 'Name', 'theta (rad)');

    elseif TIER == 2
        add_line(model_name, 'integrator/1', 'controller/1', 'autorouting', 'smart');
        add_line(model_name, 'Demux_x/3',   'controller/2', 'autorouting', 'smart');
        add_line(model_name, 'p_ref/1',      'controller/3', 'autorouting', 'smart');
        add_line(model_name, 'controller/1', 'plant/2',      'autorouting', 'smart');
        add_line(model_name, 'integrator/1', 'plant/1',      'autorouting', 'smart');
        add_line(model_name, 'plant/1',      'integrator/1', 'autorouting', 'smart');

        lh = add_line(model_name, 'Demux_x/3', 'Scope_states/3', 'autorouting', 'smart');
        set_param(lh, 'Name', 'theta (rad)');

    elseif TIER == 3
        add_line(model_name, 'integrator/1', 'controller/1', 'autorouting', 'smart');
        add_line(model_name, 'Demux_x/3',   'controller/2', 'autorouting', 'smart');
        add_line(model_name, 'Demux_x/4',   'controller/3', 'autorouting', 'smart');
        add_line(model_name, 'p_ref/1',      'controller/4', 'autorouting', 'smart');
        add_line(model_name, 'controller/1', 'plant/2',      'autorouting', 'smart');
        add_line(model_name, 'integrator/1', 'plant/1',      'autorouting', 'smart');
        add_line(model_name, 'plant/1',      'integrator/1', 'autorouting', 'smart');

        lh = add_line(model_name, 'Demux_x/3', 'Scope_states/3', 'autorouting', 'smart');
        set_param(lh, 'Name', 'theta (rad)');
    end

    %% Wire — Shared
    add_line(model_name, 'integrator/1', 'Demux_x/1', 'autorouting', 'smart');

    lh = add_line(model_name, 'Demux_x/1', 'Mux_p/1', 'autorouting', 'smart');
    set_param(lh, 'Name', 'p (m)');
    lh = add_line(model_name, 'p_ref/1',   'Mux_p/2', 'autorouting', 'smart');
    set_param(lh, 'Name', 'p_{ref} (m)');
    add_line(model_name, 'Mux_p/1', 'Scope_states/1', 'autorouting', 'smart');

    lh = add_line(model_name, 'Demux_x/2', 'Scope_states/2', 'autorouting', 'smart');
    set_param(lh, 'Name', 'v (m/s)');
    lh = add_line(model_name, 'Demux_x/4', 'Scope_states/4', 'autorouting', 'smart');
    set_param(lh, 'Name', 'T (N)');

    add_line(model_name, 'controller/1', 'Demux_u/1', 'autorouting', 'smart');
    lh = add_line(model_name, 'Demux_u/1', 'Scope_inputs/1', 'autorouting', 'smart');
    set_param(lh, 'Name', 'u_{servo} (rad)');
    lh = add_line(model_name, 'Demux_u/2', 'Scope_inputs/2', 'autorouting', 'smart');
    set_param(lh, 'Name', 'u_{thrust} (N)');

    set_param([model_name '/Scope_states'], 'ShowLegend', 'on');
    set_param([model_name '/Scope_inputs'], 'ShowLegend', 'on');

    %% Save
    save_system(model_name, save_path);
    fprintf('Saved: %s.slx\n', save_path);
    fprintf('Open it with:  open_system(''%s'')\n\n', save_path);
end

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

function s = controller_code(tier, mat_file)
switch tier
    case 1
        s = controller_code_tier1(mat_file);
    case 2
        s = controller_code_tier2(mat_file);
    case 3
        s = controller_code_tier3(mat_file);
    otherwise
        error('Unknown TIER: %d', tier);
end
end

function s = controller_code_tier1(mat_file)
lines = {
    'function u = controller(x, p_ref)'
    '% Fixed-gain LQR. Single operating point: theta*=0, T*=T_r_nom.'
    '% Control law: u = u_star - K_fixed * (x - x_star),  x_star(1) = p_ref'
    'persistent K_fixed x_star u_star'
    'if isempty(K_fixed)'
    ['    tmp    = coder.load(''' mat_file ''');']
    '    K_fixed = tmp.K_fixed;'
    '    x_star  = tmp.x_star;'
    '    u_star  = tmp.u_star;'
    'end'
    'x_star(1) = p_ref;'
    'u = u_star - K_fixed * (x - x_star);'
    'u(1) = max(min(u(1), pi/3), -pi/3);  % servo ±60°'
    'u(2) = max(min(u(2), 4.17), 0);      % thrust 0–4.17 N (static sweep at 1950 µs)'
    'end'
};
s = strjoin(lines, newline);
end

function s = controller_code_tier2(mat_file)
lines = {
    'function u = controller(x, theta, p_ref)'
    '% Angle-scheduled LQR. K(theta) interpolated from gain table using live theta state.'
    '% Scheduling variable is x(3) (actual servo angle), not a commanded reference.'
    '% Control law: u = u_star - K * (x - x_star),  x_star(1) = p_ref'
    'persistent theta_s K_s'
    'if isempty(theta_s)'
    ['    tmp     = coder.load(''' mat_file ''');']
    '    theta_s = tmp.theta_sched;'
    '    K_s     = tmp.K_sched;'
    'end'
    'K = zeros(2, 4);'
    'for i = 1:2'
    '    for j = 1:4'
    '        K(i,j) = interp1(theta_s, reshape(K_s(i,j,:),[],1), theta, ''linear'', ''extrap'');'
    '    end'
    'end'
    'T_nom  = 2.574;'
    'x_star = [p_ref; 0; 0; T_nom];'
    'u_star = [0; T_nom];'
    'u = u_star - K * (x - x_star);'
    'u(1) = max(min(u(1), pi/3), -pi/3);  % servo ±60°'
    'u(2) = max(min(u(2), 4.17), 0);      % thrust 0–4.17 N (static sweep at 1950 µs)'
    'end'
};
s = strjoin(lines, newline);
end

function s = controller_code_tier3(mat_file)
lines = {
    'function u = controller(x, theta, T_meas, p_ref)'
    '% Pair-scheduled LQR. K(theta, T) interpolated from 2D gain table using live states.'
    '% Scheduling variables are x(3) (servo angle) and x(4) (thrust).'
    '% Control law: u = u_star - K * (x - x_star),  x_star(1) = p_ref'
    'persistent theta_s T_s K_s'
    'if isempty(theta_s)'
    ['    tmp     = coder.load(''' mat_file ''');']
    '    theta_s = tmp.theta_sched;'
    '    T_s     = tmp.T_sched;'
    '    K_s     = tmp.K_sched;'
    'end'
    'K = zeros(2, 4);'
    'for i = 1:2'
    '    for j = 1:4'
    '        K(i,j) = interp2(T_s, theta_s, squeeze(K_s(i,j,:,:)), T_meas, theta, ''linear'');'
    '    end'
    'end'
    'T_nom  = 2.574;'
    'x_star = [p_ref; 0; 0; T_nom];'
    'u_star = [0; T_nom];'
    'u = u_star - K * (x - x_star);'
    'u(1) = max(min(u(1), pi/3), -pi/3);  % servo ±60°'
    'u(2) = max(min(u(2), 4.17), 0);      % thrust 0–4.17 N (static sweep at 1950 µs)'
    'end'
};
s = strjoin(lines, newline);
end
