close all;
IMAGE_TS=10;
IMU_TS=7;
CMD_TS=9;

for i=1:(length(image_data))   
   image_data(i,IMAGE_TS)=image_data(i,1)+image_data(i,2)*10^-9; %index 10 contains time_stamp
end 

for i=1:(length(imu_data))   
   imu_data(i,IMU_TS)=imu_data(i,1)+imu_data(i,2)*10^-9; %index 7 contains time_stamp
end 

for i=1:(length(cmd_data))   
   cmd_data(i,CMD_TS)=cmd_data(i,1)+cmd_data(i,2)*10^-9; %index 9 contains time_stamp
end 

% Fast timestamp synchronization 

for i=1:(length(image_data))
    min_dt=100;
    for j=1:(length(imu_data))
        diff=abs(image_data(i,IMAGE_TS)-imu_data(j,IMU_TS));
        if(diff<min_dt)
            min_dt=diff;
            min_dt_index=j;
        end
    end
    
    imu_sync(i,1)=imu_data(min_dt_index,1);
    imu_sync(i,2)=imu_data(min_dt_index,2);
    imu_sync(i,3)=imu_data(min_dt_index,3);
    imu_sync(i,4)=imu_data(min_dt_index,4);
    imu_sync(i,5)=imu_data(min_dt_index,5);
    imu_sync(i,6)=imu_data(min_dt_index,6);
    imu_sync(i,7)=imu_data(min_dt_index,IMU_TS); % timestamp in seconds
end

min_dt_index=1;
for i=1:(length(image_data))
    min_dt=100;
    for j=1:(length(cmd_data))
        diff=abs(image_data(i,IMAGE_TS)-cmd_data(j,CMD_TS));
        if(diff<min_dt)
            min_dt=diff;
            min_dt_index=j;
        end
    end
    
    cmd_sync(i,1)=cmd_data(min_dt_index,1); 
    cmd_sync(i,2)=cmd_data(min_dt_index,2);
    cmd_sync(i,3)=cmd_data(min_dt_index,3); % x_vel
    cmd_sync(i,4)=cmd_data(min_dt_index,8); % omega_z
    cmd_sync(i,5)=cmd_data(min_dt_index,CMD_TS); %TS
   
end

x=0;
y=0;
theta=0;

ground(1,1)=0;
ground(1,2)=0;
ground(1,3)=0;

ground_viso(1,1)=0;
ground_viso(1,2)=0;
ground_viso(1,3)=0;

weight_ground=[0.5 0.5 0.1];
weight_viso=[0.5 0.5 0.9];

% for i=2:(length(cmd_sync))
%     dt=cmd_sync(i,5)-cmd_sync(i-1,5);
%     x=x+cmd_sync(i,3)*dt*cosd(theta);
%     y=y+cmd_sync(i,3)*dt*sind(theta);
%     theta=theta+ rad2deg(cmd_sync(i,4)*dt);
%     
%     ground_viso(i,1)=vo_trans{i}(1);
%     ground_viso(i,2)=vo_trans{i}(3);
%     ground_viso(i,3)=imu_angles(i,1)-imu_angles(i-1,1);
%     
%     ground(i,1)= weight_ground(1)*x + weight_viso(1) * ground_viso(i,1);
%     ground(i,2)=weight_ground(2)*y + weight_viso(2) * ground_viso(i,2);
%     ground(i,3)=weight_ground(3)*theta + weight_viso(3) * ground_viso(i,3);
%     
%     
%     
% end

for i=2:(length(cmd_sync))
    dt=abs(cmd_sync(i,5)-cmd_sync(i-1,5));
    x=x+cmd_sync(i,3)*dt*cos(theta);
    y=y+cmd_sync(i,3)*dt*sin(theta);
    ground(i,1)=-y;
    ground(i,2)=x;
    ground(1,3)=cmd_sync(1,5);
    delta(i,1)=theta;
    %theta=cmd_sync(i,4);
    theta=theta+ (cmd_sync(i,4)*dt);
end


%% EKF Implementation 
P = eye(5);
state = [0 0 0 0 0 ]'; % initial state initialization 
x=0;
y=0;
for i=2:(length(cmd_sync)-1)
    dt=abs(cmd_sync(i,5)-cmd_sync(i-1,5));
   % z_viso=[Traj_imu(i,1); Traj_imu(i,3)];
    z_viso=[image_data(i,3); image_data(i,5)];
    z_viso_prev = [Traj_imu(i-1,1); Traj_imu(i-1,3)];
    state(4)=cmd_sync(i,3);
    state(5)=cmd_sync(i,4);
    if(dt>0.01)
        [state,P]=ekf_viso(state,P,z_viso,z_viso_prev,dt);
    end
    ekf(i,1)=state(1);
    ekf(i,2)=state(2);
    ekf(i,3)=cmd_sync(i,5); % TS
    
end

Traj_px4=Traj_imu;

%figure(2),plot(ground(:,1),ground(:,2),'b',Traj_xsens(:,1),Traj_xsens(:,3),'--b',Traj_px4(:,1),Traj_px4(:,3),'m--',Traj_px4_fused(:,1),Traj_px4_fused(:,3),'r--', ekf(:,1),ekf(:,2),'.g',image_data(:,3),image_data(:,5),'k');
%figure(2),legend('V , OMEGA','XSENS + VISO','PX4 + VISO','PX4 + MAG + VISO','EKF','VISO');
%plot(-ground(:,1),ground(:,2),'b');

plot(image_data(:,3),image_data(:,5),'k',ground(:,1),ground(:,2),'r',Traj_imu(:,1),Traj_imu(:,3),'b', ekf(:,1),ekf(:,2),'.g');
