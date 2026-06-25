function p = params()
    % --- toggles ---
    p.tog.friction = 1; % whether to include friction disturbance
    p.tog.meas_noise = 1; % whether to include measurement noise

    % --- seeds ---
    p.seed.friction = 20000; % friction jitter seed
    p.seed.meas = p.seed.friction + 2000; % measurement noise seed

    % --- reference trajectory ---
    p.ref.type = 2; % 1=step  2=sine  3=ramp  4=smooth step  5=PRPS
    p.ref.amp = 0.1; % amplitude / target [m]
    p.ref.freq = 1*(2*pi); % frequency [rad/s]
    [~, ~, ~, p.ref.prps] = make_prps_reference([], 0.05, 5001);

    % --- filters ---
    p.filt.vel = 0.025; % low-pass filter for measured velocity

    % --- physical / shared ---
    p.des.m = 0.5; % mass [kg]
    p.des.T = 4.2; % thrust [N], constant per run

    % --- initial conditions ---
    p.ic.p = 0.0; % initial position [m]
    p.ic.v = 0.0; % initial velocity [m/s]

    % --- DESIGN model (what controller assumes) ---
    p.des.reldeg = 2; % relative degree of the system
    p.des.omega = 0.6; % rad/s, first hardware crude-stabilizer tune
    p.des.k1 = 3*p.des.omega^2; % triple real pole at -omega
    p.des.k2 = 3*p.des.omega;
    p.des.ki = 0.5*p.des.omega^3; % integrator gain, to be tuned
    p.des.k_theta = 0.001556;
    

    % --- TRUTH model (what we test against; controller blind to this) ---
    p.truth.m = 0.5; % mass [kg]
    p.truth.T = 4.2; % thrust [N], constant per run
    p.truth.tau_theta = 0.0244; % servo lag [s]
    p.truth.L_theta = 0.0288; % transport delay [s]
    p.truth.k_theta = 0.001556;
    p.truth.d_max = 1.5; % friction bound [m/s^2 equiv, in accel terms]
    p.truth.asym = 0.3; % +/- direciton asymmetry in fraction
    p.truth.friction = default_friction_struct(p.truth.m, p.truth.d_max, p.truth.asym);

    % Pressing "Run" on this file (no output captured) loads p into the base
    % workspace so the model picks it up -- no separate setup.m call needed.
    % When called as p = params() it stays a pure function with no side effects.
    if nargout == 0
        assignin('base', 'p', p);
        fprintf('params loaded into base workspace (p).\n');
    end
end

function friction = default_friction_struct(massKg, dMaxMps2, asym)
    meanForce = massKg * dMaxMps2;
    friction = struct();
    friction.case_name = "params_default";
    friction.mass_kg = massKg;
    friction.b_pos_Ns_per_m = 0;
    friction.b_neg_Ns_per_m = 0;
    friction.mu_pos_N = meanForce * (1 + asym);
    friction.mu_neg_N = meanForce * (1 - asym);
    friction.breakaway_low_N = 0;
    friction.breakaway_high_N = 0;
    friction.v_epsilon_mps = 5e-3;
    friction.spatial_centers_m = zeros(5, 1);
    friction.spatial_widths_m = ones(5, 1);
    friction.spatial_amplitudes_N = zeros(5, 1);
    friction.jitter_amp_N = 0.05;
    friction.d_max_mps2 = dMaxMps2;
    friction.asym = asym;
end

% later, perturb p.truth terms while p.des is fixed, to see how robust 
% the controller is to model mismatch.
