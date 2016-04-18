%plot(imu_angles(:,4),imu_angles(:,3),'r',imu_filter_angles(:,4),imu_filter_angles(:,3),'g');

fileID3=fopen('cmd_vel.txt','r'); % cmd_vel input to husky (for ekf)
A3=textscan(fileID3,'%s');
fclose(fileID3);

fileID1=fopen('viso.txt','r');  %%Pose_plot.txt
formatSpec1='%s';
A1=textscan(fileID1,formatSpec1);
fclose(fileID1);


fileID2=fopen('imu.txt','r'); % imu_data.txt
formatSpec2='%s';
A2=textscan(fileID2,formatSpec2);
fclose(fileID2);

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

for i=1:length(imu_data)

    [yaw,pitch,roll]=quat2angle((imu_data(i,3:6)));
    
    imu_full_angles(i,1)=rad2deg(roll)   ;% yaw angle 
    imu_full_angles(i,2)=rad2deg(pitch)   ;% pitch
    imu_full_angles(i,3)=rad2deg(yaw)   ;% roll
    imu_full_angles(i,4)=i;

end 

%plot cmd_vel data
%plot(cmd_data(:,1)+(cmd_data(:,2)*10^-9),cmd_data(:,3),'r',cmd_data(:,1)+(cmd_data(:,2)*10^-9),cmd_data(:,8),'g');
%plot(imu_data(:,1)+(imu_data(:,2)*10^-9),imu_full_angles(:,1),'b');