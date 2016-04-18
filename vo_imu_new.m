%% Synchronize images with IMU data 
clear all
close all
clc 

fileID1=fopen('run2/viso.txt','r');  %%Pose_plot.txt
formatSpec1='%s';
A1=textscan(fileID1,formatSpec1);
fclose(fileID1);

fileID2=fopen('run2/mavros_filter.txt','r'); % imu_data.txt
formatSpec2='%s';
A2=textscan(fileID2,formatSpec2);
fclose(fileID2);


%% Read time stamps and values from text file 

fileID3=fopen('run2/cmd_vel.txt','r'); % cmd_vel input to husky (for ekf)
A3=textscan(fileID3,formatSpec1);
fclose(fileID3);

%% Read time stamps and values from text file 

length_cmd= length(A3{1,1})/26; % new frame starts at index 26 for cmd_vel
j=1;

for i=1:(length_cmd)
    cmd_data(i,1)=str2num(A3{1,1}{j+5});
    cmd_data(i,2)=str2num(A3{1,1}{j+7});
    cmd_data(i,3)=str2num(A3{1,1}{j+13});   %linear_x
    cmd_data(i,4)=str2num(A3{1,1}{j+15});   %linear_y
    cmd_data(i,5)=str2num(A3{1,1}{j+17});   %linear_z
    cmd_data(i,6)=str2num(A3{1,1}{j+20});   %angular_x
    cmd_data(i,7)=str2num(A3{1,1}{j+22});   %angular_y
    cmd_data(i,8)=str2num(A3{1,1}{j+24});   %angular_z
    j=j+26;
end



j=1;
for i=1:(length(A1{1,1})/121)    %121
   image_data(i,1)=str2num(A1{1,1}{j+5}); %secs
    image_data(i,2)=str2num(A1{1,1}{j+7}); %nsecs
    image_data(i,3)=str2num(A1{1,1}{j+16})  ;% Translation in X-direction 
    image_data(i,4)=str2num(A1{1,1}{j+18})  ;% Translation in Y-direction 
    image_data(i,5)=str2num(A1{1,1}{j+20})  ; % Translation in Z-direction 
    image_data(i,6)=(str2num(A1{1,1}{j+23}))  ;% orientation in x          
    image_data(i,7)=(str2num(A1{1,1}{j+25}))  ;% orientation in y         
    image_data(i,8)=(str2num(A1{1,1}{j+27})) ;% orientation in z 
    image_data(i,9)=(str2num(A1{1,1}{j+29})); % orientation in w
j=j+121;
end 
j=1;
for i=1:(length(A2{1,1})/64)
    
    imu_data(i,1)=str2num(A2{1,1}{j+5});
    imu_data(i,2)=str2num(A2{1,1}{j+7});
    imu_data(i,3)=str2num(A2{1,1}{j+12});
    imu_data(i,4)=str2num(A2{1,1}{j+14});
    imu_data(i,5)=str2num(A2{1,1}{j+16});
    imu_data(i,6)=str2num(A2{1,1}{j+18});
    j=j+64;
end 

%% Data is synchronized here


IMAGE_TS=10;
IMU_TS=7;
for i=1:(length(image_data))    %121
   image_data(i,IMAGE_TS)=image_data(i,1)+image_data(i,2)*10^-9; %index 10 contains time_stamp
end 

for i=1:(length(imu_data))    %121
   imu_data(i,IMU_TS)=imu_data(i,1)+imu_data(i,2)*10^-9; %index 7 contains time_stamp
end 


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




%% Cmd_vel Data is synchronized here




%% radians to deg's

for i=1:length(image_data)

    [yaw,pitch,roll]=quat2angle((imu_sync(i,3:6)));
    
    q=quaternion([imu_sync(i,6),imu_sync(i,3),imu_sync(i,4),imu_sync(i,5)]);
    angles=q.EulerAngles('123'); %r p y
    
%     imu_angles(i,1)=rad2deg(r)   ;% yaw angle 
%     imu_angles(i,2)=rad2deg(p)   ;% pitch
%     imu_angles(i,3)=rad2deg(y)   ;% roll
%     imu_angles(i,4)=i;

    imu_angles(i,1)=rad2deg(angles(3))   ;% yaw angle 
    imu_angles(i,2)=rad2deg(angles(2))   ;% pitch
    imu_angles(i,3)=rad2deg(angles(1))   ;% roll
    imu_angles(i,4)=i;

%     imu_angles(i,1)=(imu_sync(i,5))   ;% yaw angle 
%     imu_angles(i,2)=(imu_sync(i,4))   ;% pitch
%     imu_angles(i,3)=(imu_sync(i,3))   ;% roll
%     imu_angles(i,4)=i;

end 

%% Get VO rotation and Translation 

rot_matrix(1,:,:)=eye(3);

for i=1:length(image_data)
    
%  quat_data=[image_data(i,6),image_data(i,7),image_data(i,8),image_data(i,9)];
    
%  [roll,pitch,yaw]=quat2angle(image_data(i,6:9));
 
% vo_rotation_matrix{i}(:,:)=compose_rotation(roll,pitch,yaw);
  
 vo_rotation_matrix{i}(:,:)=compose_rotation(image_data(i,7),image_data(i,6),image_data(i,8)); 
 
 
 %   vo_rotation_matrix{i}(:,:)=compose_rotation(rad2deg(roll),rad2deg(pitch),rad2deg(yaw));
     
    translation_vector{i}(1:3)=[image_data(i,3),image_data(i,4),image_data(i,5)];   % This is from Viso2
end 
yaw_arr(1) = 0


for i=2:length(image_data)
    
    vo_trans{i}=(vo_rotation_matrix{i-1})'*(translation_vector{i}-translation_vector{i-1})';  % Transformation into local frame from Global frame 
  
   
   vo_mag(i) = norm(vo_trans{i});
   %vo_mag1(i,1)=vo_trans{i};
end    
%    vo_temp_rot{i} = (vo_rotation_matrix{i-1})'*(vo_rotation_matrix{i});
%    [vo_rot_c1,vo_rot_c2,vo_rot_c3]=decompose_rotation(vo_rotation_matrix{i-1});
%     [roll,pith,yaw] = decompose_rotation(vo_temp_rot{i});
%    
%    vo_angles(i,1)=rad2deg(vo_rot_c1);
%    vo_angles(i,2)=rad2deg(vo_rot_c2);
%    vo_angles(i,3)=rad2deg(vo_rot_c3);
%    yaw_arr(i) = rad2deg(yaw);
%end 
% for i =428:1144 %length(imu_angles)
% %     imu_angles(i,1) = -imu_angles(i,1);
% end

%% Trasform from IMU to Visual odometry
for i=1:length(image_data)-1
    % yaw is -90 and Roll is -90
    rz_yaw=[cosd(-90) -sind(-90) 0;
            sind(-90) cosd(-90) 0;
            0 0 1];
    ry_pitch=[cosd(0) 0 sind(0);
              0 1 0;
              -sind(0) 0 cosd(0)];
    rx_roll=[1 0 0;
        0 cosd(-90) -sind(-90);
        0 sind(-90) cosd(-90)];
    co_ordinate_rot1=rz_yaw*ry_pitch*rx_roll;
    Rot_matrix1=compose_rotation(0,0,imu_angles(i,1));   %% Roll and Yaw Angles 
 %   Rot_matrix1=compose_rotation(0,0,imu_angles(i,1));
    Rot_matrix2=compose_rotation(0,0,imu_angles(i+1,1));  %% Roll and Yaw Angles 
 %Rot_matrix2=compose_rotation(0,0,imu_angles(i+1,1)); 
 %   imu_angles(i,3);
    temp_rot=Rot_matrix1'*Rot_matrix2; 
    rot_matrix_imu{i+1}(:,:)=co_ordinate_rot1'*(temp_rot)*co_ordinate_rot1;    
end 

%%  get distances from Visual odometry %%
% 
% for i=1:length(image_data)
%     vo_rotation_matrix{i}(:,:)=compose_rotation(image_data(i,7),image_data(i,6),image_data(i,8));    % Here multiplication goes with pitch roll yaw 
%     
%     translation_vector{i}(1:3)=[image_data(i,3),image_data(i,4),image_data(i,5)];
% end 
% 
% vo_trans{1}=[0 0 0];
% rotation_matrix_instant{i}(:,:)=eye(3);
% 
% for i=2:length(image_data)
%     
%     rotation_matrix_instant{i}(:,:)=(vo_rotation_matrix{i-1}(:,:))'*vo_rotation_matrix{i}(:,:);
%     
% end 
% 
% for i=2:length(image_data)
% 
%    vo_trans{i}=(vo_rotation_matrix{i-1})'*(translation_vector{i}-translation_vector{i-1})';
%    
%    vo_temp_rot{i} = (vo_rotation_matrix{i-1})'*(vo_rotation_matrix{i});
%    
%    [vo_rot_c1,vo_rot_c2,vo_rot_c3]=decompose_rotation(vo_rotation_matrix{i-1});
%     
%    
% %    vo_angles(i,1)=rad2deg(vo_rot_c1);
% %    vo_angles(i,2)=rad2deg(vo_rot_c2);
% %    vo_angles(i,3)=rad2deg(vo_rot_c3);
%    
% end 
%%
Tr_total1{1}=eye(4); % IMU+VO Trajectory
Tr_total3{1}=eye(4);
Tr_imu{1}=eye(4);


%% Get Trajectory with respect to global frame  

Tr_imu{1} = eye(4);
for i=2:length(image_data)-1
    th = imu_angles(i,1)-imu_angles(i+1,1);
R = [cosd(th) 0 -sind(th); 0 1 0; sind(th) 0 cosd(th)];

% Tr=[rot_matrix_imu{i}(1,1) rot_matrix_imu{i}(1,2) rot_matrix_imu{i}(1,3) 0;
 %    rot_matrix_imu{i}(2,1) rot_matrix_imu{i}(2,2) rot_matrix_imu{i}(2,3) 0;
 %   rot_matrix_imu{i}(3,1) rot_matrix_imu{i}(3,2) rot_matrix_imu{i}(3,3) vo_mag(i);   % vo_mag(i)
  %   0 0 0 1];
  Tr = [R [0;0;vo_mag(i)];0 0 0 1];
  Tr_imu{i}=Tr_imu{i-1}*(Tr);
end

 
for i=1:length(Tr_imu)
    
    Traj_imu(i,1)=-Tr_imu{i}(1,4);
    Traj_imu(i,2)=Tr_imu{i}(2,4);
    Traj_imu(i,3)=Tr_imu{i}(3,4);
    Traj_imu(i,4)=i;
end  
%     rot_temp=[Tr_imu{i}(1,1) Tr_imu{i}(1,2) Tr_imu{i}(1,3);
%               Tr_imu{i}(2,1) Tr_imu{i}(2,2) Tr_imu{i}(2,3);
%               Tr_imu{i}(3,1) Tr_imu{i}(3,2) Tr_imu{i}(3,3)];
%     
%    [rot_c1,rot_c2,rot_c3]=decompose_rotation(rot_temp);
%     
%    rot_decomposed(i,1)=rad2deg(rot_c1);
%    rot_decomposed(i,2)=rad2deg(rot_c2);
%    rot_decomposed(i,3)=rad2deg(rot_c3);
%    
% end 
figure(1),plot(Traj_imu(:,1),Traj_imu(:,3),'r',image_data(:,3),image_data(:,5),'g');
axis('square');
% %% get a new traje 
% vo_trans{427}(1:3,1)
% vo_trans{428}(1:3,1)
% vo_trans{429}(1:3,1)
% vo_trans{430}(1:3,1)
% 
% for i=2:800
%     [y,p,r]=(decompose_rotation(vo_temp_rot{i}));
%     disp('viso');
%      [rad2deg(y),rad2deg(p),rad2deg(r)]
%     [y,p,r]=(decompose_rotation(rot_matrix_imu{i}));
%     disp('imu');
%     [rad2deg(y),rad2deg(p),rad2deg(r)]
% end 
% figure(2),plot(image_data(:,3),image_data(:,5));
% axis([-100 100 -40 100]);
