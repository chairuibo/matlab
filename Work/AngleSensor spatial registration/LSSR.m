%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 程序名称：空间配准程序，样本生成函数target6_2.m
% 说    明：空中站有角度误差和位置误差
%           空中站在移动
%           添加采用奇异值分解法 最小二乘法
% 版    本：第6.13版
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

%% 载入数据
sensorRealPos = load('Data/SensorRealPos.mat','SensorRealPos');
sensorRealPos = sensorRealPos.SensorRealPos;
sensorMeasPos = load('Data/SensorMeasPos.mat','SensorMeasPos');
sensorMeasPos = sensorMeasPos.SensorMeasPos;
realData = load('Data/TargetRealPos.mat');
realData = realData.realPos;
measData = load('Data/TargetMeasurePos.mat');
measData = measData.measPos;
ErrData = load('Data/TargetMeasureErr.mat');
ErrData = ErrData.ErrData;

L = 1000;
%% 数据
Alpha = measData(1,:);
Beita = measData(2,:);
Gamma = measData(3,:);

%% 空中站进行角度配准
AngleErr(:,1)=[0.001;0.001;0.002];
P_k = diag([1 1 1]);
for i=1:L
    % 数据输入
    alpha = Alpha(1,i);
    beita = Beita(1,i);
    gamma = Gamma(1,i);

    obvX_A = sensorMeasPos(1,i);      %空中站1 X轴位置
    obvY_A = sensorMeasPos(2,i);      %空中站1 Y轴位置
    obvX_B = sensorMeasPos(3,i);      %空中站2 Y轴位置
    obvY_B = sensorMeasPos(4,i);      %空中站2 Y轴位置
    obvX_C = sensorMeasPos(5,i);      %空中站3 X轴位置
    obvY_C = sensorMeasPos(6,i);      %空中站3 Y轴位置
    
    % 构造H矩阵
    H_temp(i,1) = (obvX_C-obvX_B+(obvY_B-obvY_A)*tan(beita)+(obvY_A-obvY_C)*tan(gamma))*sec(alpha)^2;
    H_temp(i,2) = (obvX_A-obvX_C+(obvY_B-obvY_A)*tan(alpha)+(obvY_C-obvY_B)*tan(gamma))*sec(beita)^2;
    H_temp(i,3) = (obvX_B-obvX_A+(obvY_C-obvY_B)*tan(beita)+(obvY_A-obvY_C)*tan(alpha))*sec(gamma)^2;      
    H = H_temp;
    % 构造伪量测
    Z_1 = (obvX_C-obvX_B)*tan(alpha)+(obvX_A-obvX_C)*tan(beita)+(obvX_B-obvX_A)*tan(gamma);
    Z_2 = (obvY_B-obvY_C)*tan(beita)*tan(gamma)+(obvY_C-obvY_A)*tan(alpha)*tan(gamma)+(obvY_A-obvY_B)*tan(alpha)*tan(beita);
    Z(i,1) = Z_1-Z_2;
        
    % 计算特征值和条件数
    J = H'*H;
    eigenvalue(:,i) = eig(J);
    MaxData = max(eigenvalue(:,i));
    MinData = min(eigenvalue(:,i));
    cond(1,i) = MaxData/MinData;
    hhh=sort(eigenvalue(:,i));
    [U,S,V] = svd(J);
    
    % TSVD算法
%     j = length(S);
%     for k = 1:j
%         if S(k,k) <= 0
%             S(k,k) = 0;
%         else 
%             S(k,k) = 1/S(k,k);
%         end
%     end

    % MSVD算法
%     j = length(S);
%     for k = 1:j
%         S(k,k) = S(k,k)/(S(k,k)^2+0200);
%     end
    
    % CSVD算法
    trunk = 1;
    j = length(S);
    for k = 1:j
        if S(k,k) == 0
            break;
        elseif S(k,k)<=trunk
            S(k,k) = S(1,1)/trunk;
        end
        S(k,k) = 1/S(k,k);
    end  

    % 计算误差值
    AngleErr(:,i+1) = V*S*U'*H'*Z;
end

%% 利用估计得到的空中站位置对目标进行估计
Angle_Err = num2cell(AngleErr(:,2:L+1),[1,L]);
Meas_B.Z = num2cell(measData(1:3,:),[1,L]);
Meas_B.L = L;
Meas_B.AngleErr= Angle_Err;
Meas_B.SensorPos = num2cell(sensorMeasPos,[1,L]);
ModelB.x0 =[5000;0;0;7500;0;0];
ModelB.p0 =diag([5000,50,5,5000,50,5]);
B= [(0.1^3)/6; (0.1^2)/2; 0.1]; 
B = [B           zeros(3,1)   
     zeros(3,1)  B];
ModelB.Q = 5.3^2*(B*B');%5.3 0.1
ModelB.R = diag([0.01/57.3,0.01/57.3,0.01/57.3].^2);
X_fun = @XFun5_2;
Z_fun = @ZFun10;
BasePos_Btemp = Kalman_CKF_10(ModelB,Meas_B,X_fun,Z_fun);
BasePos_B = cell2mat(BasePos_Btemp.X);

%% 结果输出
% 数据准备
t = 0.1:0.1:L/10;
angleErr(1:3,:) = AngleErr(1:3,:).*57.3;

figure(1);
subplot(211);
plot(t(100:end),BasePos_B(1,100:end),'b',t(100:end),realData(1,100:end),'r');
title('目标X轴运动轨迹');
xlabel('时间 t/s');ylabel('X轴位置 m');
legend('目标位置估计值','目标位置实际值');
subplot(212);
plot(t(100:end),realData(1,100:end)-BasePos_B(1,100:end),'g');
xlabel('时间 t/s');ylabel('误差 m');
legend('目标位置估计值误差');

figure(2);
subplot(211);
plot(t(100:end),BasePos_B(4,100:end),'b',t(100:end),realData(2,100:end),'r');
title('目标Y轴运动轨迹');
xlabel('时间 t/s');ylabel('Y轴位置 m');
legend('目标位置估计值','目标位置实际值');
subplot(212);
plot(t(100:end),realData(2,100:end)-BasePos_B(4,100:end),'g');
xlabel('时间 t/s');ylabel('误差 m');
legend('目标位置估计值误差');

ErrData = ErrData.*57.3;
figure(3);
subplot(211);
plot(t(10:end),angleErr(1,11:end),'b',t(10:end),ErrData(1,10:end),'r');
title('传感器A角度误差估计结果');
xlabel('时间 t/s');ylabel('误差 度');
legend('估计的误差值','实际的误差值');
subplot(212)
plot(t,angleErr(1,2:end)-ErrData(1,:));
title('传感器A角度误差估计值的误差');
xlabel('时间 t/s');ylabel('误差 度');
legend('估计误差值的误差');

figure(4);
subplot(211);
plot(t(10:end),angleErr(2,11:end),'b',t(10:end),ErrData(2,10:end),'r');
title('传感器B角度误差估计结果');
xlabel('时间 t/s');ylabel('误差 度');
legend('估计的误差值','实际的误差值');
subplot(212)
plot(t,angleErr(2,2:end)-ErrData(2,:));
title('传感器B角度误差估计值的误差');
xlabel('时间 t/s');ylabel('误差 度');
legend('估计误差值的误差');

figure(5);
subplot(211);
plot(t(10:end),angleErr(3,11:end),'b',t(10:end),ErrData(3,10:end),'r');
title('传感器C角度误差估计结果');
xlabel('时间 t/s');ylabel('误差 度');
legend('估计的误差值','实际的误差值');
subplot(212)
plot(t,angleErr(3,2:end)-ErrData(3,:));
title('传感器C角度误差估计值的误差');
xlabel('时间 t/s');ylabel('误差 度');
legend('估计误差值的误差');

figure(6)
plot(cond(10:end));
axes('position',[0.3,0.3 0.2 0.3]);
hold on;
plot(t(10:20),cond(10:20));