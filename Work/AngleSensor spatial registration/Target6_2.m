%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   目标生成文件6.2
%   3个光学站
%   光学站有角度误差和位置误差
%   光学站在移动
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear 
clc 

Obv1x = 1000;       %空中站1 X轴初始位置
Obv1y = 10000;      %空中站1 Y轴初始位置
Obv2x = 10000;      %空中站2 X轴初始位置
Obv2y = 5000;       %空中站2 Y轴初始位置
Obv3x = 3000;       %空中站3 X轴初始位置
Obv3y = 4000;       %空中站3 Y轴初始位置
A1 = 1/57.3;  	%空中站2的角度误差
A2 = 1/57.3;  	%空中站2的角度误差
A3 = 1/57.3;   	%空中站3的角度误差 

T=0.1;
L=100/T;
t = 0.1:T:L*T;

Obv1x_meas = Obv1x+7*t;      %空中站1 X轴位置
Obv1y_meas = Obv1y+3.5*t;    %空中站1 Y轴位置
Obv2x_meas = Obv2x+3.5*t;    %空中站2 X轴位置
Obv2y_meas = Obv2y-5*t;      %空中站2 Y轴位置
Obv3x_meas = Obv3x+2*t;      %空中站3 X轴位置
Obv3y_meas = Obv3y+5*t;      %空中站3 Y轴位置

Obv1x = Obv1x_meas;   	%空中站1 X轴位置
Obv1y = Obv1y_meas;  	%空中站1 Y轴位置
Obv2x = Obv2x_meas;  	%空中站2 X轴位置
Obv2y = Obv2y_meas;   	%空中站2 Y轴位置
Obv3x = Obv3x_meas;   	%空中站3 X轴位置
Obv3y = Obv3y_meas;   	%空中站3 Y轴位置

% 存储传感器的真实位置再添加噪声
SensorRealPos(1,:) = Obv1x_meas;
SensorRealPos(2,:) = Obv1y_meas;
SensorRealPos(3,:) = Obv2x_meas;
SensorRealPos(4,:) = Obv2y_meas;
SensorRealPos(5,:) = Obv3x_meas;
SensorRealPos(6,:) = Obv3y_meas;
SensorMeasPos = SensorRealPos+200;  % +randn(6,L)

% 导出传感器位置数据，以方便后续使用
Obv1x_meas = SensorMeasPos(1,:);      %空中站1 X轴位置
Obv1y_meas = SensorMeasPos(2,:);      %空中站1 Y轴位置
Obv2x_meas = SensorMeasPos(3,:);      %空中站2 X轴位置
Obv2y_meas = SensorMeasPos(4,:);      %空中站2 Y轴位置
Obv3x_meas = SensorMeasPos(5,:);      %空中站3 X轴位置
Obv3y_meas = SensorMeasPos(6,:);      %空中站3 Y轴位置

% plot(Obv1x_meas,Obv1y_meas,'r');
% hold on;
% plot(Obv2x_meas,Obv2y_meas,'b');
% hold on;
% plot(Obv3x_meas,Obv3y_meas,'g');

%% 第二步生成目标的运动轨迹
x = 1000*cos(t)+6000;
y = 1000*sin(t)+7500;
realPos(1,:) = x;
realPos(2,:) = y;
% hold on;
% plot(x,y,'k');
% legend('传感器A位置','传感器B位置','传感器C位置','目标运动轨迹');

%% 第三步：生成观测值
%除观测站1外其他观测站加入位置误差和角度误差
obvPos(1,:) = atan2(x-Obv1x_meas, y-Obv1y_meas)+A1;      %空中站1对目标的角度观测
obvPos(2,:) = atan2(Obv2x_meas-x, Obv2y_meas-y)+A2;      %空中站2对目标的角度观测
obvPos(3,:) = atan2(x-Obv3x_meas, y-Obv3y_meas)+A3;      %空中站3对目标的角度观测
w(1:3,:) = 0.01/57.3*randn(3,L);
measPos = obvPos + w;

%认为的理想情况下的观测值
ErrPos(1,:) = atan2(x-Obv1x, y-Obv1y);      %空中站1对目标的角度观测
ErrPos(2,:) = atan2(Obv2x-x, Obv2y-y);      %空中站2对目标的角度观测
ErrPos(3,:) = atan2(x-Obv3x, y-Obv3y);      %空中站3对目标的角度观测
ErrData = obvPos-ErrPos;

save('Data/TargetMeasureErr6_2.mat','ErrData');
save('Data/SensorRealPos6_2.mat','SensorRealPos');
save('Data/SensorMeasPos6_2.mat','SensorMeasPos');
save('Data/TargetRealPos6_2.mat','realPos');
save('Data/TargetMeasurePos6_2.mat','measPos');
disp('程序运行结束，目标数据6已产生');
