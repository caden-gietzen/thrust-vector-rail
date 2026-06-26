function s = encoderAngleScale()
%ENCODERANGLESCALE Single source of truth for the servo encoder count-to-angle scale.
%
% The servo output shaft and the quadrature encoder are coupled by a GT2
% timing belt with EQUAL pulleys (16T : 16T), giving an exact 1:1 rotational
% ratio. The conversion is therefore spec-derived and exact -- not empirically
% calibrated:
%
%   - GT2 is a toothed belt: it does not slip, and equal integer tooth counts
%     give an exact 1:1 angular ratio. Belt tension does not change
%     counts-per-degree (that only matters for a rotation->translation drive,
%     e.g. the linear rail encoder, where pitch-line diameter introduces a real
%     scale error -- see experiments/encoder_calibration/results.md).
%   - The encoder is 600 PPR; the PIO quadrature decoder counts all 4 edges:
%       counts_per_rev = 600 * 4 = 2400.
%
%   theta_deg = count * 360 / 2400 = count * 0.15  (exact)
%
% Backlash (~2.4 deg gear lost-motion) is a SEPARATE additive offset, not a
% scale error on this ratio; it is carried as its own truth-model block.
%
% A half-revolution sweep is retained only as a coarse no-tooth-skip sanity
% check (fast moves can drop counts; see the encoder-mismatch validation), NOT
% as the calibration basis. The prior "~2414 counts/rev" figure was a ~1 deg
% protractor misread of a sweep that was actually ~181 deg, not 180 deg.
%
% Returned struct fields:
%   counts_per_rev   - encoder counts per full revolution (= encoder_ppr * 4)
%   encoder_ppr      - encoder pulses per revolution (pre-quadrature)
%   belt_ratio       - encoder revs per servo rev (1.0 for 16T:16T)
%   deg_per_count    - servo output angle per count (deg)
%   rad_per_count    - servo output angle per count (rad)
%
% Example:
%   s = encoderAngleScale();
%   theta_deg = count_delta * s.deg_per_count;

    s.encoder_ppr    = 600;
    s.belt_ratio     = 1.0;                       % 16T : 16T GT2, exact 1:1
    s.counts_per_rev = s.encoder_ppr * 4 * s.belt_ratio;   % 2400

    s.deg_per_count  = 360.0 / s.counts_per_rev;  % 0.15 deg/count (exact)
    s.rad_per_count  = 2.0 * pi / s.counts_per_rev;
end
