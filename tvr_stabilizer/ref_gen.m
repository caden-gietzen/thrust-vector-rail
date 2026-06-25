function [r, rdot, rddot] = ref_gen(t, type, amp, freq)
%REF_GEN  Reference trajectory and its derivatives from a single definition.
%   One place defines r(t); rdot and rddot are the analytic derivatives, so the
%   controller feedforward can never desync from the position reference. Switch
%   trajectory by changing p.ref.type (and p.ref.amp / p.ref.freq) -- no block
%   swapping, no editing three sources.
%
%   type : 1 = step / constant setpoint (amp held from t=0)
%          2 = sine
%          3 = ramp           (slope = amp*freq, so freq sets the rate)
%          4 = smooth step    (tanh(freq*t) toward amp; C-inf, zero initial rate)
%          5 = PRPS position reference (default seed-5001 validation plan)
%   amp  : amplitude / target [m]
%   freq : angular frequency [rad/s] (sine) or rate [1/s] (ramp, smooth step)
    w = freq;
    switch type
        case 1                          % step / constant setpoint
            r     = amp;
            rdot  = 0;
            rddot = 0;
        case 2                          % sine
            r     =  amp*sin(w*t);
            rdot  =  amp*w*cos(w*t);
            rddot = -amp*w^2*sin(w*t);
        case 3                          % ramp
            r     = amp*w*t;
            rdot  = amp*w;
            rddot = 0;
        case 4                          % smooth step (tanh)
            sgm   = tanh(w*t);
            r     =  amp*sgm;
            rdot  =  amp*w*(1 - sgm^2);
            rddot = -2*amp*w^2*sgm*(1 - sgm^2);
        case 5                          % PRPS position reference, fixed default plan
            tau = mod(max(t, 0), 40.0);
            peak = 8.1845835005000751;
            raw = 0;
            rawdot = 0;
            rawddot = 0;

            w = 2*pi*0.0500000000000000; a = w*tau + 1.0946418472895860;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.0750000000000000; a = w*tau + 5.5800748155082456;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.1000000000000000; a = w*tau + 0.2865366937119958;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.1250000000000000; a = w*tau + 4.9430623902752560;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.1500000000000000; a = w*tau + 2.3991241489220894;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.1750000000000000; a = w*tau + 5.8047445368753730;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.2250000000000000; a = w*tau + 6.0313825955002178;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.2750000000000000; a = w*tau + 2.0168867469714620;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.3500000000000000; a = w*tau + 3.7206529805881345;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.4250000000000000; a = w*tau + 5.5399711165939278;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.5250000000000000; a = w*tau + 5.5209901217078743;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.6500000000000000; a = w*tau + 3.0362019124652537;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*0.8000000000000000; a = w*tau + 5.6352576025042511;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);
            w = 2*pi*1.0000000000000000; a = w*tau + 5.8122408685105320;
            raw = raw + sin(a); rawdot = rawdot + w*cos(a); rawddot = rawddot - w^2*sin(a);

            r = amp * raw / peak;
            rdot = amp * rawdot / peak;
            rddot = amp * rawddot / peak;
        otherwise                       % safe default: hold zero
            r     = 0;
            rdot  = 0;
            rddot = 0;
    end
end
