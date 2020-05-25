function [x, P] = mu_g(x, P, yacc, Ra, g0)
%       Inputs:
%       x      - [n x 1] Prior mean
%       P      - [n x n] Prior covariance
%       yacc   - [m x 1] Measured accelerometer for time k
%       Ra     - [1 x 1] Measurement noise covariance
%       g0     - [3 x 1] Nominal gravity vector

%       Outputs:
%       x      - [n x 1] Posterior mean
%       P      - [n x n] Posterior covariance

% Define measurement model functions h and H
fa_k = [0 0 0]'; % Assume no acceleration
h = Qq(x)' * (g0 + fa_k);
[Q0, Q1, Q2, Q3] = dQqdq(x);
H = [Q0'*g0, Q1'*g0, Q2'*g0, Q3'*g0];

% Innovation covariance and Kalman gain
S = H * P * H' + Ra;
K = P * H' /S;

% Calculate updated states and covariance
x = x + K*(yacc - h);
P = P - K*S*K.';



end