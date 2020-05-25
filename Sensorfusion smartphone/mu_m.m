function [x, P] = mu_m(x, P, mag, m0, Rm)
%       Inputs:
%       x      - [n x 1] Prior mean
%       P      - [n x n] Prior covariance
%       mag    - [m x 1] Measured magnetometer for time k
%       m0     - [1 x 1] Earth magnetic field in world coordinates
%       Ra     - [3 x 1] Measurement noise covariance

%       Outputs:
%       x      - [n x 1] Posterior mean
%       P      - [n x n] Posterior covariance

% Define measurement model functions h and H
fm_k = [0 0 0]'; % Assume no other interfering sources of magnetic fields
h = Qq(x)' * (m0 + fm_k);
[Q0, Q1, Q2, Q3] = dQqdq(x);
hPrim = [Q0'*m0, Q1'*m0, Q2'*m0, Q3'*m0];

% Innovation covariance and Kalman gain
S = hPrim * P * hPrim.' + Rm;
K = P * hPrim' / S;

% Calculate updated states and covariance
x = x + K*(mag - h);
P = P - K*S*K';


end