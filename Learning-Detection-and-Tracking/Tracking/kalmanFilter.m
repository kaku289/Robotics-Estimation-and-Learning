function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state:[position_x, position_y, velocity_x,
%   velocity_y]'

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0]';
        param.P = diag([10,10,10000,10000]);
        param.Q = diag([0.2,0.2,1,1]);
        param.R = 0.5*eye(2);
        param.Phi = [1 0 0.0334 0;
                   0 1 0 0.0334;
                   0 0 1  0;
                   0 0 0  1];
        param.Gamma = [0.0334         0    0.0006         0;
                       0    0.0334         0    0.0006;
                       0         0    0.0334         0;
                       0         0         0    0.0334];
        param.C = [1 0 0 0;
                   0 1 0 0];
        predictx = x;
        predicty = y;
        param.dt = 0.0334;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    xk1_k = param.Phi*state;
    Pk1_k = param.Phi*param.P*param.Phi'+param.Gamma*param.Q*param.Gamma';
    K = Pk1_k*param.C'/(param.R+param.C*Pk1_k*param.C');
    state = xk1_k+K*([x;y] - param.C*xk1_k);
    param.P = param.P -K*param.C*param.P;
    % You should replace this code
% %     vx = (x - state(1)) / (t - previous_t);
% %     vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
%     predictx = state(1) + state(3) * 0.330;
%     predicty = state(2) + state(4) * 0.330;
    predictx = state(1);
    predicty = state(2);
% %     predictx = x + vx * 0.330;
% %     predicty = y + vy * 0.330;
% %     % State is a four dimensional element
% %     state = [x, y, vx, vy];
end
