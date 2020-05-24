function [x, P] = tu_qw(x, P, omega, T, Rw)
%       Inputs:
%       x      - [n x 1] Prior mean
%       P      - [n x n] Prior covariance
%       omega  - [m x 1] Measured angular rate
%       T      - [1 x 1] Time since last measurement
%       Rw     - [m x m] Process noise covariance

%       Outputs:
%       x      - [n x 1] Posterior mean
%       P      - [n x n] Posterior covariance

F = (eye(4) + 1/2*T*Somega(omega));
G = 1/2*T*Sq(x); 

x = F*x;

P = F*P*F' + G*Rw*G';

 
end
