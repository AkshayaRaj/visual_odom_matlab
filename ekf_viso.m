function [state, P] = ekf_viso(state,P,z_viso,z_viso_prev,dt)
% state = [x, y, theta, v, omega]'
% P = covariance matrix .. initially pass an identity matrix to ekf
% z_viso = [x_viso, y_viso] 
% Goal: Use the new Viso measurement to update the EKF state and covariance

% Set constants (Or pass into the function)
%dt = 0.1;          % 10 Hz But here I'm taking dt between two viso frames

%Q = diag([0.01, 0.01, 0.001, 0.3, 0.3]);
Q = diag([0.001 0.001 0.001 0.000000003 0.000000003 ]);
Qk = Q*(dt); % <- System (Process) Noise

R_viso = [700.1^2   0; 
          0    700.1^2];
Rk_viso = R_viso*dt; % <- Measurement Noise of viso (tuning param)
% tr_x=z_viso(1);
% tr_y=z_viso(2);
% tr = (tr_x^2+tr_y^2)^ -(1/2);  

% Extended Kalman Filter:
[state_pre, P_pre] = system_update(state, P, Qk, dt);
[state, P] = meas_update_viso(state_pre, P_pre, z_viso,z_viso_prev,Rk_viso, dt);

end


function[state_pre,P_pre] = system_update(state,P, Qk,dt)
% state = [x,y,theta,v,omega]

% Update state: x_pre= f(x,u) , where x= state and u=0 
state_pre = [state(1)- state(4)*dt*sin(state(3));
             state(2)+state(4)*dt*cos(state(3));
             state(3)+state(5)*dt;
             state(4);
             state(5);
             ];
 % Calulate Fk by taking jacobian of non-linear state equation . So
 % basically the system model is differentiated wrt each of the state
 % variables which gives a 5x5 matrix in this case
 
 Fk = [1   0   -state(4)*dt*cos(state(3))   -dt*sin(state(3))    0 ;
        0   1   -state(4)*dt*sin(state(3))   dt*cos(state(3))    0;
        0   0               1                       0           dt;
        0   0               0                       1           0;
        0   0               0                       0           1;] ;
    
  % Update covariance  P_pre = Fk * P * Fk' + Qk
  % Qk is System Noise covariance matrix (a tuning parameter). The diagonal
  % elements of Qk represent state variance during a single system update
  P_pre = Fk * P * Fk' + Qk ;
  
  
  
end

function [state_post,P_post] = meas_update_viso(state,P,z,z_prev,Rk,dt)
% state = [x,y,theta,v,omega]'
% off = [x_off,y_off) : lever arm offset
% z = [x_viso, y_viso] : actual viso reading 
USE_VISO=1; % if != 1 then filter will follow V,Omega trajectory 

% Calculate Hk
% Hk = [ 1 0 -off(1)*sin(state(3))-off(2)*cos(state(3)) 0 0;
%        0 1  off(1)*cos(state(3))-off(2)*sin(state(3)) 0 0];
%    

%take H as Identity , no state measurement model ? TODO
% H=[1 0 0 0 0 ;
%    0 1 0 0 0 ];
H=[1 0 0 0 0 ;
   0 1 0 0 0 ];

   
   
   % Find the expected measurement: h(state) .. state after the
   % measurement? 
% viso_est = [state(1) + tr*cos(yaw);
%            state(2) + tr*sin(yaw)];
       
h_z = [state(1)- state(4)*dt*sin(state(3)) ;state(2)+state(4)*dt*cos(state(3))] ; % may need to look into this
%h_z = h(x) - x(t-1) in literature 
%h_z = eye(2)*z - z_prev;



 % Find Kalman Gain
K = P*H'*(H*P*H'+Rk)^-1

% z
% h_z

%state 

% Find new state:
 state_post = state + K*(z - h_z);
%state_post = state + K*( h_z);
%state_post = state;
%state_post = state;



% Find new covariance:
P_post = P - K*H*P ;
%P_post = P;

if(USE_VISO == 0)
    state_post = state;
    P_post = P;
end

end


         
    
