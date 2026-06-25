function L = loop_margin(varargin)
%LOOP_MARGIN  Linearize the tvr feedback loop and show Bode + stability margins.
%   L = LOOP_MARGIN() linearizes tvr_sim at the controller output (theta_cmd),
%   plots the open-loop Bode with gain/phase margins, and returns the open-loop
%   transfer function L. Run it from the MATLAB Command Window after any change
%   to params.m or the model.
%
%   L = LOOP_MARGIN('des.omega',1.8,'filt.vel',0.025, ...) applies temporary
%   dotted-field overrides into p for exploration WITHOUT editing params.m.
%   The gains (k1,k2,ki) are re-derived from des.omega unless you override
%   them explicitly. The default integral gain uses the same softened value
%   as params.m and docs/crude_stabilizer_design.md: ki = 0.5*omega^3.
%
%   Examples:
%     loop_margin                                  % current params
%     loop_margin('filt.vel',0.025)               % tighten velocity filter
%     loop_margin('des.omega',1.8,'des.ki',5)     % lower bandwidth, soft integral
%
%   Notes:
%     * Friction and measurement noise are forced OFF for the analysis: they are
%       disturbances, not loop gain, and the friction tanh has an artificially
%       huge slope at v=0 that would pollute the linearization.
%     * The reference amplitude is forced to 0 so the trim point is the origin
%       (theta~0, sin theta ~ theta). The loop gain is independent of the
%       reference anyway.
%     * Requires Simulink Control Design (linearize / getLoopTransfer / linio).

    mdl = 'tvr_sim';
    if ~bdIsLoaded(mdl), load_system(mdl); end

    % The Transport Delay block linearizes to Pade order 0 (NO delay) by
    % default, which flatters the phase margin. Give it a real Pade order so
    % the actuator transport delay shows up in the loop transfer.
    dblk = find_system(mdl,'BlockType','TransportDelay');
    for i = 1:numel(dblk), set_param(dblk{i},'PadeOrder','3'); end

    p = params();                            % fresh params (pure function call)

    % --- temporary dotted-field overrides ---------------------------------
    names = varargin(1:2:end);
    for k = 1:2:numel(varargin)
        f = strsplit(varargin{k}, '.');
        p = setfield(p, f{:}, varargin{k+1});           %#ok<SFLD>
    end
    % re-derive triple-pole gains from omega unless explicitly overridden
    w = p.des.omega;
    if ~ismember('des.k1',names), p.des.k1 = 3*w^2; end
    if ~ismember('des.k2',names), p.des.k2 = 3*w;   end
    if ~ismember('des.ki',names), p.des.ki = 0.5*w^3; end

    % --- clean LTI loop ---------------------------------------------------
    p.tog.friction   = 0;
    p.tog.meas_noise = 0;
    p.ref.amp        = 0;
    assignin('base','p',p);

    % --- break the loop at the controller output (theta_cmd, port 1) ------
    io    = linio([mdl '/ctrl_fblin'], 1, 'output');   % port-1 analysis point
    sllin = slLinearizer(mdl, io);
    pts   = getPoints(sllin);                % canonical point name(s)
    if iscell(pts), pt = pts{1}; else, pt = pts; end
    L  = -getLoopTransfer(sllin, pt);        % negate: getLoopTransfer here uses the
                                             % positive-feedback convention for this loop
    L  = minreal(tf(L));                     % clean up pole/zero cancellations

    % --- report + plot ----------------------------------------------------
    [Gm,Pm,~,wcp] = margin(L);
    fprintf('omega=%.3f  tauv=%.4f  k2=%.3f  ->  wc=%.3f rad/s | PM=%.2f deg | GM=%.2f dB\n', ...
            p.des.omega, p.filt.vel, p.des.k2, wcp, Pm, 20*log10(Gm));

    figure; margin(L); grid on
    title(sprintf('tvr loop:  \\omega=%.2f, \\tau_v=%.3f  \\rightarrow  \\omega_c=%.2f rad/s, PM=%.1f\\circ', ...
          p.des.omega, p.filt.vel, wcp, Pm));
end
