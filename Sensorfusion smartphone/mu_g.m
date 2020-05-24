function [x, P] = mu_g(x, P, yacc, Ra, g0)
%       Inputs:
%       x      - [n x 1] Prior mean
%       P      - [n x n] Prior covariance
%       yacc   - [m x 1] Measured accelerometer for time k
%       Ra     - [1 x 1] Time since last measurement
%       g0     - [m x m] Process noise covariance

%       Outputs:
%       x      - [n x 1] Posterior mean
%       P      - [n x n] Posterior covariance

% fa_k assumed 0, since phone lying flat 
fa_k = 0;
h = Qq(x)' * (g0 + fa_k);
[Q0, Q1, Q2, Q3] = dQqdq(x);
hPrim = [Q0'*g0, Q1'*g0, Q2'*g0, Q3'*g0];

% Calculate innovation covariance and Kalman gain
S = hPrim * P * hPrim' + Ra;
K = P * hPrim' / S;

% Calculate updated states and covariance
x = x + K*(yacc-h);
P = P - K*S*K';

end