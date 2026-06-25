function [pdot, vdot] = plant(theta_actual, v, m, T, d)
  % returns derivatives only — no integration inside
  pdot = v;
  vdot = (T/m)*sin(theta_actual) - d;   % d = friction disturbance
end