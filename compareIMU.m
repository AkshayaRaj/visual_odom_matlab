fileID2=fopen('run2/mavros_filter.txt','r'); % imu_data.txt
formatSpec2='%s';
A2=textscan(fileID2,formatSpec2);
fclose(fileID2);

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


for i=1:(length(imu_data))   
   imu_data(i,7)=imu_data(i,1)+imu_data(i,2)*10^-9; %index 7 contains time_stamp
end 


for i=1:length(imu_data)
    
    q=quaternion([imu_data(i,6),imu_data(i,3),imu_data(i,4),imu_data(i,5)]);
    euler=q.EulerAngles('123'); %r p y
    
    [yaw,pitch,roll]=quat2angle((imu_data(i,3:6)));
    
%     imu_angles(i,1)=rad2deg(roll)   ;% yaw angle 
%     if(imu_angles(i,1) < 0)
%         imu_angles(i,1)=imu_angles(i,1);
%     end
%     imu_angles(i,2)=rad2deg(pitch)   ;% pitch
%     imu_angles(i,3)=rad2deg(yaw)   ;% roll
%     imu_angles(i,4)=i;
%     imu_angles(i,5)=imu_data(i,7);

    imu_angles(i,1)=rad2deg(euler(3))   ;% yaw angle 
    if(imu_angles(i,1) < 0)
        imu_angles(i,1)=imu_angles(i,1);
    end
    imu_angles(i,2)=rad2deg(euler(2))   ;% pitch
    imu_angles(i,3)=rad2deg(euler(1))   ;% roll
    imu_angles(i,4)=i;
    imu_angles(i,5)=imu_data(i,7);

end 

%xsens=imu_angles;
mavros_filter=imu_angles;
 %mavros=imu_angles;
plot(mavros(:,5),mavros(:,1),'r',mavros_filter(:,5),mavros_filter(:,1),'b',xsens(:,5),xsens(:,1),'g');
legend('PX4','PX4+Magnetometer','Xsens');
