%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Ŀ�������ļ�6.2
%   3����ѧվ
%   ��ѧվ�нǶ�����λ�����
%   ��ѧվ���ƶ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear 
clc 

Obv1x = 1000;       %����վ1 X���ʼλ��
Obv1y = 10000;      %����վ1 Y���ʼλ��
Obv2x = 10000;      %����վ2 X���ʼλ��
Obv2y = 5000;       %����վ2 Y���ʼλ��
Obv3x = 3000;       %����վ3 X���ʼλ��
Obv3y = 4000;       %����վ3 Y���ʼλ��
A1 = 1/57.3;  	%����վ2�ĽǶ����
A2 = 1/57.3;  	%����վ2�ĽǶ����
A3 = 1/57.3;   	%����վ3�ĽǶ���� 

T=0.1;
L=100/T;
t = 0.1:T:L*T;

Obv1x_meas = Obv1x+7*t;      %����վ1 X��λ��
Obv1y_meas = Obv1y+3.5*t;    %����վ1 Y��λ��
Obv2x_meas = Obv2x+3.5*t;    %����վ2 X��λ��
Obv2y_meas = Obv2y-5*t;      %����վ2 Y��λ��
Obv3x_meas = Obv3x+2*t;      %����վ3 X��λ��
Obv3y_meas = Obv3y+5*t;      %����վ3 Y��λ��

Obv1x = Obv1x_meas;   	%����վ1 X��λ��
Obv1y = Obv1y_meas;  	%����վ1 Y��λ��
Obv2x = Obv2x_meas;  	%����վ2 X��λ��
Obv2y = Obv2y_meas;   	%����վ2 Y��λ��
Obv3x = Obv3x_meas;   	%����վ3 X��λ��
Obv3y = Obv3y_meas;   	%����վ3 Y��λ��

% �洢����������ʵλ�����������
SensorRealPos(1,:) = Obv1x_meas;
SensorRealPos(2,:) = Obv1y_meas;
SensorRealPos(3,:) = Obv2x_meas;
SensorRealPos(4,:) = Obv2y_meas;
SensorRealPos(5,:) = Obv3x_meas;
SensorRealPos(6,:) = Obv3y_meas;
SensorMeasPos = SensorRealPos+200;  % +randn(6,L)

% ����������λ�����ݣ��Է������ʹ��
Obv1x_meas = SensorMeasPos(1,:);      %����վ1 X��λ��
Obv1y_meas = SensorMeasPos(2,:);      %����վ1 Y��λ��
Obv2x_meas = SensorMeasPos(3,:);      %����վ2 X��λ��
Obv2y_meas = SensorMeasPos(4,:);      %����վ2 Y��λ��
Obv3x_meas = SensorMeasPos(5,:);      %����վ3 X��λ��
Obv3y_meas = SensorMeasPos(6,:);      %����վ3 Y��λ��

% plot(Obv1x_meas,Obv1y_meas,'r');
% hold on;
% plot(Obv2x_meas,Obv2y_meas,'b');
% hold on;
% plot(Obv3x_meas,Obv3y_meas,'g');

%% �ڶ�������Ŀ����˶��켣
x = 1000*cos(t)+6000;
y = 1000*sin(t)+7500;
realPos(1,:) = x;
realPos(2,:) = y;
% hold on;
% plot(x,y,'k');
% legend('������Aλ��','������Bλ��','������Cλ��','Ŀ���˶��켣');

%% �����������ɹ۲�ֵ
%���۲�վ1�������۲�վ����λ�����ͽǶ����
obvPos(1,:) = atan2(x-Obv1x_meas, y-Obv1y_meas)+A1;      %����վ1��Ŀ��ĽǶȹ۲�
obvPos(2,:) = atan2(Obv2x_meas-x, Obv2y_meas-y)+A2;      %����վ2��Ŀ��ĽǶȹ۲�
obvPos(3,:) = atan2(x-Obv3x_meas, y-Obv3y_meas)+A3;      %����վ3��Ŀ��ĽǶȹ۲�
w(1:3,:) = 0.01/57.3*randn(3,L);
measPos = obvPos + w;

%��Ϊ����������µĹ۲�ֵ
ErrPos(1,:) = atan2(x-Obv1x, y-Obv1y);      %����վ1��Ŀ��ĽǶȹ۲�
ErrPos(2,:) = atan2(Obv2x-x, Obv2y-y);      %����վ2��Ŀ��ĽǶȹ۲�
ErrPos(3,:) = atan2(x-Obv3x, y-Obv3y);      %����վ3��Ŀ��ĽǶȹ۲�
ErrData = obvPos-ErrPos;

save('Data/TargetMeasureErr6_2.mat','ErrData');
save('Data/SensorRealPos6_2.mat','SensorRealPos');
save('Data/SensorMeasPos6_2.mat','SensorMeasPos');
save('Data/TargetRealPos6_2.mat','realPos');
save('Data/TargetMeasurePos6_2.mat','measPos');
disp('�������н�����Ŀ������6�Ѳ���');
