%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 程序名称：空间配准程序，样本生成函数target6_2.m
% 说    明：空中站有角度误差和位置误差
%           空中站在移动
%           添加采用奇异值分解法 EX算法
% 版    本：v1.0  (18235107312 CRB)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

%% 载入数据
SensorRealPos = load('Data/SensorRealPos.mat','SensorRealPos');
SensorRealPos = SensorRealPos.SensorRealPos;
SensorMeasPos = load('Data/SensorMeasPos.mat','SensorMeasPos');
SensorMeasPos = SensorMeasPos.SensorMeasPos;
TargetRealData = load('Data/TargetRealPos.mat');
TargetRealData = TargetRealData.realPos;
TargetMeasData = load('Data/TargetMeasurePos.mat');
TargetMeasData = TargetMeasData.measPos;
ErrData = load('Data/TargetMeasureErr.mat');
ErrData = ErrData.ErrData;

%% 建立目标运动模型
x0=[6995,100,1,8000,100,1]';
Model_AB = buildModel_CA(0.1,2,x0,[1,1]);
Model_AB.R = diag([0.01/57.3,0.01/57.3].^2);
Model_BC = buildModel_CA(0.1,2,x0,[0.01,0.01]);
Model_BC.R = Model_AB.R;
%% 两两传感器数据滤波
X_fun = @XFun_EXSR;
Z_fun = @ZFun_EXSR;
%传感器AB数据滤波
MeasData_AB.Z = num2cell(TargetMeasData(1:2,:),1);
MeasData_AB.L = length(MeasData_AB.Z);
MeasData_AB.SensorPos = num2cell(SensorMeasPos,1);
ExtiValue_AB = Kalman_CKF_EXSR(Model_AB,MeasData_AB,X_fun,Z_fun);
%传感器AC数据滤波
MeasData_AC.Z = num2cell(TargetMeasData([1 3],:),1);
MeasData_AC.L = length(MeasData_AC.Z);
MeasData_AC.SensorPos = MeasData_AB.SensorPos;
ExtiValue_AC = Kalman_CKF_EXSR(Model_AB,MeasData_AC,X_fun,Z_fun);
%传感器BC数据滤波
MeasData_BC.Z = num2cell(TargetMeasData([2 3],:),1);
MeasData_BC.L = length(MeasData_BC.Z);
MeasData_BC.SensorPos = MeasData_AB.SensorPos;
ExtiValue_BC = Kalman_CKF_EXSR(Model_BC,MeasData_BC,X_fun,Z_fun);

%% 仿真结果输出
q = outputData(0,Model_AB,TargetMeasData,TargetRealData,ExtiValue_AB.X);
q = outputData(q,Model_AB,TargetMeasData,TargetRealData,ExtiValue_AC.X);
q = outputData(q,Model_AB,TargetMeasData,TargetRealData,ExtiValue_BC.X);