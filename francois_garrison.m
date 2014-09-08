function range = francois_garrison(T, S, z, pH, f)
% Based on Lurton Sec. 2.3
%
% Examples:
%   FRANCOIS_GARRISON
%     Default parameters  
%   FRANCOIS_GARRISON(14, 38.5, 0, 7, 100)
%     Mediterranean approximation at surface from Lurton
%
% See also ATTENUATE
%
% Copyright 2014 Sidharth Iyer (246964@gmail.com)

  % Default parameters:
  if nargin == 0
    T = 25;   % degrees Celsius
    S = 35;   % practical salinity units
    z = 0;    % metres
    pH = 7;
    f = 100;  % kHz
  end
  
  tic

  c = calc_c(T, S, z);

  % Boric acid:
  A1 = calc_A1(c, pH);
  P1 = 1;
  f1 = calc_f1(S, T);

  % Magnesium sulphate:
  A2 = calc_A2(S, c, T);
  P2 = calc_P2(z);
  f2 = calc_f2(T, S);

  % Pure water:
  P3 = calc_P3(z);
  A3 = calc_A3(T);

  % Attenuation in dB/km:
  alpha = calc_zCoeff(z) * calc_alpha(A1, P1, f1, f, A2, P2, f2, A3, P3);
  display(alpha);

  % Define acceptable loss in intensity in dB:
  SL = 145;       % Source level
  DT = 0;         % Detection threshold
  TL = SL - DT;   % Maximum acceptable trasmission loss

  syms R positive;

  % Two-way transmission loss:
  range = eval(solve(TL == 2 * (20 * log10(R) + alpha * 10 ^ -3 * R), ...
                     R, 'Real', true, 'PrincipalValue', true));

  % Two-way transmission loss (more intuitive result?):
  % range = eval(solve(TL == 20 * log10(2 * R) + alpha * 0.001 * 2 * R, ...
  %                    R, 'Real', true, 'PrincipalValue', true));

	% One-way transmission loss:
  % range = eval(solve(TL == 20 * log10(R) + alpha * 0.001 * R, ...
  %                    R, 'Real', true, 'PrincipalValue', true));

  toc

end

function c = calc_c(T, S, z)
  c = 1412 + 3.21 * T + 1.19 * S + 0.0167 * z;
end

function A1 = calc_A1(c, pH)
  A1 = (8.86 / c) * 10 ^ (0.78 * pH - 5);
end

function f1 = calc_f1(S, T)
  f1 = 2.8 * sqrt(S / 35) * 10 ^ (4 - 1245 / (T + 273));
end

function A2 = calc_A2(S, c, T)
  A2 = 21.44 * (S / c) * (1 + 0.025 * T);
end

function P2 = calc_P2(z)
  P2 = 1 - 1.37e-4 * z + 6.2e-9 * z ^ 2;
end

function f2 = calc_f2(T, S)
  f2 = (8.17 * 10 ^ (8 - 1990 / (T + 273))) / (1 + 0.0018 * (S - 35));
end

function P3 = calc_P3(z)
  P3 = 1 - 3.83e-5 * z + 4.9e-10 * z ^ 2;
end

function A3 = calc_A3(T)
  % Taking Lurton's T < 20 to mean T <= 20:
  if T <= 20
    A3 = 4.937e-4 - 2.59e-5 * T + 9.11e-7 * T ^ 2 - 1.5e-8 * T ^ 3;
  else
    A3 = 3.964e-4 - 1.146e-5 * T + 1.45e-7 * T ^ 2 - 6.5e-10 * T ^ 3;
  end
end

function alpha = calc_alpha(A1, P1, f1, f, A2, P2, f2, A3, P3)
  alpha = A1 * P1 * f1 * f ^ 2 / (f1 ^ 2 + f ^ 2) ...
        + A2 * P2 * f2 * f ^ 2 / (f2 ^ 2 + f ^ 2) ...
        + A3 * P3 * f ^ 2;
end

function zCoeff = calc_zCoeff(z)
  H = [ 0 500 1000 1500 2000 2500 3000 3500 4000 4500 5000 5500 6000 ];

  P2_H = [ 1 0.93 0.87 0.81 0.75 0.7 0.64 0.6 0.55 0.51 0.47 0.43 0.4 ];
  zCoeff = interp1(H, P2_H, z, 'pchip', 'extrap');

  % A_H = [ 1 0.97 0.93 0.9 0.86 0.83 0.79 0.76 0.73 0.69 0.66 0.62 0.59 ];
  % zCoeff = interp1(H, A_H, z, 'pchip', 'extrap');

  % plot(H, P2_H, H, A_H);
  % set(legend('P_2(H)', 'A(H)'));
end
