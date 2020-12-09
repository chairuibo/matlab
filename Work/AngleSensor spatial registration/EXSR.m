%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �������ƣ��ռ���׼�����������ɺ���target6_2.m
% ˵    ��������վ�нǶ�����λ�����
%           ����վ���ƶ�
%           ��Ӳ�������ֵ�ֽⷨ EX�㷨
% ��    ����v1.0  (18235107312 CRB)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

%% ��������
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

%% ����Ŀ���˶�ģ��
x0=[6995,100,1,8000,100,1]';
Model_AB = buildModel_CA(0.1,2,x0,[1,1]);
Model_AB.R = diag([0.01/57.3,0.01/57.3].^2);
Model_BC = buildModel_CA(0.1,2,x0,[0.01,0.01]);
Model_BC.R = Model_AB.R;
%% ���������������˲�
X_fun = @XFun_EXSR;
Z_fun = @ZFun_EXSR;
%������AB�����˲�
MeasData_AB.Z = num2cell(TargetMeasData(1:2,:),1);
MeasData_AB.L = length(MeasData_AB.Z);
MeasData_AB.SensorPos = num2cell(SensorMeasPos,1);
ExtiValue_AB = Kalman_CKF_EXSR(Model_AB,MeasData_AB,X_fun,Z_fun);
%������AC�����˲�
MeasData_AC.Z = num2cell(TargetMeasData([1 3],:),1);
MeasData_AC.L = length(MeasData_AC.Z);
MeasData_AC.SensorPos = MeasData_AB.SensorPos;
ExtiValue_AC = Kalman_CKF_EXSR(Model_AB,MeasData_AC,X_fun,Z_fun);
%������BC�����˲�
MeasData_BC.Z = num2cell(TargetMeasData([2 3],:),1);
MeasData_BC.L = length(MeasData_BC.Z);
MeasData_BC.SensorPos = MeasData_AB.SensorPos;
ExtiValue_BC = Kalman_CKF_EXSR(Model_BC,MeasData_BC,X_fun,Z_fun);

%% ���������
q = outputData(0,Model_AB,TargetMeasData,TargetRealData,ExtiValue_AB.X);
q = outputData(q,Model_AB,TargetMeasData,TargetRealData,ExtiValue_AC.X);
q = outputData(q,Model_AB,TargetMeasData,TargetRealData,ExtiValue_BC.X);