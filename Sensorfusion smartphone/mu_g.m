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

% fa_k assumed 0, since phone lying flat 
% fa_k = [0 0 0]';
% h = (Qq(x) * (g0 + fa_k));
% [Q0, Q1, Q2, Q3] = dQqdq(x);
% H = [Q0'*g0, Q1'*g0, Q2'*g0, Q3'*g0];
% 
% % Calculate innovation covariance and Kalman gain
% S = H * P * H' + Ra;
% K = P * H' * inv(S);
% 
% % Calculate updated states and covariance
% x = x + K*(yacc - H*x);
% P = P - K*H*P';
% % [x, P] = mu_normalizeQ(x, P); % Normalize quaternion
% % J = dQqdq(x);
% % P = (eye(4) - K*H) * P;
% % P = J*P*J';

% Define h and H
h = Qq(x)'*g0;
[Q0, Q1, Q2, Q3] = dQqdq(x);
H = [Q0'*g0, Q1'*g0, Q2'*g0, Q3'*g0];

% Innovation covariance and Kalman gain
S = H * P * H' + Ra;
K = P * H' /S;

% Calculate updated states and covariance
x = x + K*(yacc - h);
P = P - K*S*K.';



%             % Calculate measurement model and Jacobian evaluated at x
%             [hx, Hx] = h(x);
%             % Calculate innovation covariance
%             S = Hx*P*Hx' + R;
%             % Calculate kalman gain
%             K = P*Hx'/S;
%             % Updated state mean
%             x = x + K*(y-hx);
%             % Updated state covariance
%             P = P - K*S*K';
end