%if one object is moving ,it has a big chance to be detect twice
%radar should have a stable duration to synathize with camera
%% import data 
% the excek has been processed,add more 4 columns at the right position,incluing radar
% scan(x,y,z,usable)
Radar_Data= xlsread('JointPCloudList_Radar_(05.30_17.46.40)_Test2_1.xlsx');  %2-1.xlsx has 3 circles
%Radar_Data= xlsread('JointPCloudList_Radar_(05.30_17.27.55)_Test1_2.xlsx');
Frame0 = Radar_Data(1,1); % Measurement counter
Frame1 = Radar_Data(end,1);
Nstep=Frame1; 
klist=[];
%Config param
false_alarm_rate=0.15;
n_cell=5;
VelAccuracy =0.1/3.6;
w_accuracy = deg2rad(1);
hostspeed_accuracy = 0.2/3.6;
hostspeedAccu =0.1;
angleVelAccu = 0.01;
dynaminc_counter =0;
prob_counter =0;
vlimit_counter = 0;
point_counter =0;
detv2_counter =0; %检测速度大于2m/s的数量
v_threshold =0.5;
P_threshold = 0.9;
DetcPoint=struct('x',0,'y',0,'vel',0,'SNR',0);
for k = 2:Nstep
    k
    %I consider the sensorID,because they are not captured at same time
    %,for excample at 0ms capture 1,3 radar锛?20ms銆?capture 2,4radar,but this
    %diff time has a little improvement only ,so i ignore it later,but
    %keep some code here
    if mod(k,2)==1
        sensorId=1;
    else
        sensorId=2;
    end
    
%   timebar(h, k/Nstep)  %show the progress
   %just read the data from radar1 for test.
   % output:
   % [scan,scan_Cart,HostSpeed,Yaw_rate,raw_range,raw_azimuth,SNR,TargetVellist]
   [y_radar,scan_Cart,u(1,k-1),u(2,k-1),raw_range,raw_azimuth,SNR,TargetVellist,elevation]=GMapReadAScanRadar(Radar_Data,k,1,n_cell); 
   Lfront=2.6;   %wheelbase
   radar1_x=3.23; %radar pos_x
   radar1_y=0.72;
   hostspeed = u(1,k-1)/n_cell
   yawrate = u(2,k-1)
   %% check turn left or right, then the center of rotation will shift to
   %another side.
   if u(2,k-1) >0
   rotCenter = u(1,k-1)/n_cell/(u(2,k-1)+1e-5); %calculate the centor of rotation
   
   %calculate the angle of radar velocity referring to the centor of rotation
   theta_v = atan(radar1_x/(rotCenter - radar1_y));
   
   GTargVel = TargetVellist;
   det_prob = false_alarm_rate.^(1.0 ./ (1.0 + SNR));
  point_counter = point_counter + length(det_prob);
   for h=1:size(y_radar,2)
        if det_prob(h)>P_threshold
            prob_counter = prob_counter+1;
        end
        if TargetVellist(h)>v_threshold
            detv2_counter =detv2_counter +1;
        end
       v_radar_cos =cos(theta_v - raw_azimuth(h))* u(2,k-1) * sqrt(radar1_x^2 + (rotCenter -radar1_y)^2); % 远离为正数
       %if yawrate=0,then just use it ,not use Arkermann,because it cause error
       if u(2,k-1) ==0
           v_radar_cos = u(1,k-1)/n_cell*cos(raw_azimuth(h));
       end
       v_rela = -TargetVellist(h)* cosd(elevation(h));  %v_rela 远离为负，靠近为正数
       GTargVel(h) = abs(v_radar_cos) - abs(v_rela);
       if abs(GTargVel(h))>v_threshold
           vlimit_counter =  vlimit_counter+1;
       end
       
       if abs(GTargVel(h))>v_threshold && det_prob(h)>P_threshold
           GTargVel(h)
           klist=[klist,k];
           disp('dynamic Target!')
           dynaminc_counter=dynaminc_counter+1;
       end
   end 
   
   
   %% turn right
   else
         rotCenter = abs(u(1,k-1)/n_cell/(u(2,k-1)+1e-5)); %calculate the centor of rotation
   
   %calculate the angle of radar velocity referring to the centor of rotation
   theta_v = atan(radar1_x/(rotCenter + radar1_y));
   
   GTargVel = TargetVellist;
   det_prob = false_alarm_rate.^(1.0 ./ (1.0 + SNR));
  point_counter = point_counter + length(det_prob);
   for h=1:size(y_radar,2)
        if det_prob(h)> P_threshold
            prob_counter = prob_counter+1;
        end
        if TargetVellist(h)>v_threshold
            detv2_counter =detv2_counter +1;
        end
       v_radar_cos =cos(theta_v + raw_azimuth(h))* u(2,k-1) * sqrt(radar1_x^2 + (rotCenter +radar1_y)^2);
       
       if u(2,k-1) ==0
           v_radar_cos = u(1,k-1)/n_cell*cos(raw_azimuth(h));
       end
       v_rela = -TargetVellist(h)* cosd(elevation(h));
       GTargVel(h) = abs(v_radar_cos) - abs(v_rela);
       if abs(GTargVel(h))>v_threshold
           vlimit_counter =  vlimit_counter+1;
       end
       
       if abs(GTargVel(h))>v_threshold && det_prob(h)>P_threshold
           GTargVel(h)
           klist=[klist,k];
           disp('dynamic Target!')
           dynaminc_counter=dynaminc_counter+1;
       end
   end 
       
   end
end
 dynaminc_counter
 prob_counter
 vlimit_counter
 detv2_counter 
 point_counter
klist;
pre = 1- dynaminc_counter/point_counter