function quat_estimate = quaternionEKF(z, angular_velo, dt, i)
    %
    %
    persistent R
    persistent Sigma_w W
    persistent P
    persistent firstRun
    
    global quat_history
    
    if isempty(firstRun)        
      R = [5 0 0; 0 5 0; 0 0 5]; % acc sensor noise
      P = 10*eye(4);  
      Sigma_w = [0.05^2 0 0; 0 0.05^2 0; 0 0 0.05^2];
      firstRun = 1;
    end


    % Prediction Step
    quat = quat_history(:, i);
    
    quat_pred = fx(quat, angular_velo, dt);
    F = Fjacob(angular_velo, dt);
    W = dt/2 * [-quat(2) -quat(3) -quat(4); quat(1) -quat(4) quat(3); quat(4) quat(1) -quat(2); -quat(3) quat(2) quat(1)];
    Q = W * Sigma_w * W';
    
    Pp = F*P*F' + Q;
    
    % Correction Step
    H = h_jacob(quat_pred);
    K = (Pp*H')/(H*Pp*H' + R); % Kalman gain
    quat = quat_pred + K*(z - h(quat_pred));
    quat = quat/norm(quat);
    P = Pp - K*H*Pp;
    quat_estimate = quat;

end


%------------------------------
function qp = fx(qhat, angular_velo, dt)
    % Prediction Step
    % Calculate quaternion prediction using attitude kinematics
    % First order EKF
    qw = qhat(1);
    qx = qhat(2);
    qy = qhat(3);
    qz = qhat(4);

    wx = angular_velo(1);
    wy = angular_velo(2);
    wz = angular_velo(3);

    qp = zeros(4, 1);
    qp(1)= qw - dt/2*wx*qx - dt/2*wy*qy - dt/2*wz*qz;
    qp(2) = qx + dt/2*wx*qw - dt/2*wy*qz + dt/2*wz*qy;
    qp(3) = qy + dt/2*wx*qz + dt/2*wy*qw - dt/2*wz*qx;
    qp(4) = qz - dt/2*wx*qy + dt/2*wy*qx + dt/2*wz*qw;
end


%------------------------------
function F = Fjacob(angular_velo, dt)
    % Calculate df/dq (derivative of state transition function at q)
    % Used for computing coveriance prediction
    % First order EKF
    F = zeros(4, 4);
    wx = angular_velo(1);
    wy = angular_velo(2);
    wz = angular_velo(3);

    F(1, :) = [1, -dt/2*wx, -dt/2*wy, -dt/2*wz];
    F(2, :) = [dt/2*wx, 1, dt/2*wz, -dt/2*wy];
    F(3, :) = [dt/2*wy, -dt/2*wz, 1, dt/2*wx];
    F(4, :) = [dt/2*wz, dt/2*wy, -dt/2*wx, 1];
end

function acc_hat = h(qp)
 % Measurement model
 % relation b.t. acc measurement and quaternion prediction.
 
 % Let's use ENU (East - North - Up)
 % In this case the accelerometer reads [0 0 9.8] at initial attitude
 
 gx = 0;
 gy = 0;
 gz = 9.8;
 
 qw = qp(1); qx = qp(2); qy = qp(3); qz = qp(4);
 
 acc_hat = zeros(3, 1);
 acc_hat(1) = 2 * (gx*(1/2-qy^2-qz^2) + gy*(qw*qz+qx*qy) + gz*(qx*qz-qw*qy));
 acc_hat(2) = 2 * (gx*(qx*qy-qw*qz) + gy*(1/2-qx^2-qz^2) + gz*(qw*qx+qy*qz));
 acc_hat(3) = 2 * (gx*(qw*qy+qx*qz) + gy*(qy*qz-qw*qx) + gz*(1/2-qx^2-qy^2));
end

function dacc_hat = h_jacob(qp)
    % linearize measurement model
    % Let's use ENU (East - North - Up)
    % In this case the accelerometer reads [0 0 9.8] at initial attitude

    gx = 0;
    gy = 0;
    gz = 9.8;
     
    qw = qp(1); qx = qp(2); qy = qp(3); qz = qp(4);
    
    dacc_hat = zeros(3, 4);
    dacc_hat(1, :) = 2*[gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz, -gx*qy+gy*qx-gz*qw, -gx*qz+gy*qw+gz*qx];
    dacc_hat(2, :) = 2*[-gx*qz+gy*qw+gz*qx, gx*qy-gy*qx+gz*qw, gx*qx+gy*qy+gz*qz, -gx*qw-gy*qz+gz*qy];
    dacc_hat(3, :) = 2*[gx*qy-gy*qx+gz*qw, gx*qz-gy*qw-gz*qx, gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz];
end