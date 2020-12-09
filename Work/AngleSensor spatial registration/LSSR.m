%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �������ƣ��ռ���׼�����������ɺ���target6_2.m
% ˵    ��������վ�нǶ�����λ�����
%           ����վ���ƶ�
%           ��Ӳ�������ֵ�ֽⷨ ��С���˷�
% ��    ������6.13��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

%% ��������
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
%% ����
Alpha = measData(1,:);
Beita = measData(2,:);
Gamma = measData(3,:);

%% ����վ���нǶ���׼
AngleErr(:,1)=[0.001;0.001;0.002];
P_k = diag([1 1 1]);
for i=1:L
    % ��������
    alpha = Alpha(1,i);
    beita = Beita(1,i);
    gamma = Gamma(1,i);

    obvX_A = sensorMeasPos(1,i);      %����վ1 X��λ��
    obvY_A = sensorMeasPos(2,i);      %����վ1 Y��λ��
    obvX_B = sensorMeasPos(3,i);      %����վ2 Y��λ��
    obvY_B = sensorMeasPos(4,i);      %����վ2 Y��λ��
    obvX_C = sensorMeasPos(5,i);      %����վ3 X��λ��
    obvY_C = sensorMeasPos(6,i);      %����վ3 Y��λ��
    
    % ����H����
    H_temp(i,1) = (obvX_C-obvX_B+(obvY_B-obvY_A)*tan(beita)+(obvY_A-obvY_C)*tan(gamma))*sec(alpha)^2;
    H_temp(i,2) = (obvX_A-obvX_C+(obvY_B-obvY_A)*tan(alpha)+(obvY_C-obvY_B)*tan(gamma))*sec(beita)^2;
    H_temp(i,3) = (obvX_B-obvX_A+(obvY_C-obvY_B)*tan(beita)+(obvY_A-obvY_C)*tan(alpha))*sec(gamma)^2;      
    H = H_temp;
    % ����α����
    Z_1 = (obvX_C-obvX_B)*tan(alpha)+(obvX_A-obvX_C)*tan(beita)+(obvX_B-obvX_A)*tan(gamma);
    Z_2 = (obvY_B-obvY_C)*tan(beita)*tan(gamma)+(obvY_C-obvY_A)*tan(alpha)*tan(gamma)+(obvY_A-obvY_B)*tan(alpha)*tan(beita);
    Z(i,1) = Z_1-Z_2;
        
    % ��������ֵ��������
    J = H'*H;
    eigenvalue(:,i) = eig(J);
    MaxData = max(eigenvalue(:,i));
    MinData = min(eigenvalue(:,i));
    cond(1,i) = MaxData/MinData;
    hhh=sort(eigenvalue(:,i));
    [U,S,V] = svd(J);
    
    % TSVD�㷨
%     j = length(S);
%     for k = 1:j
%         if S(k,k) <= 0
%             S(k,k) = 0;
%         else 
%             S(k,k) = 1/S(k,k);
%         end
%     end

    % MSVD�㷨
%     j = length(S);
%     for k = 1:j
%         S(k,k) = S(k,k)/(S(k,k)^2+0200);
%     end
    
    % CSVD�㷨
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

    % �������ֵ
    AngleErr(:,i+1) = V*S*U'*H'*Z;
end

%% ���ù��Ƶõ��Ŀ���վλ�ö�Ŀ����й���
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

%% ������
% ����׼��
t = 0.1:0.1:L/10;
angleErr(1:3,:) = AngleErr(1:3,:).*57.3;

figure(1);
subplot(211);
plot(t(100:end),BasePos_B(1,100:end),'b',t(100:end),realData(1,100:end),'r');
title('Ŀ��X���˶��켣');
xlabel('ʱ�� t/s');ylabel('X��λ�� m');
legend('Ŀ��λ�ù���ֵ','Ŀ��λ��ʵ��ֵ');
subplot(212);
plot(t(100:end),realData(1,100:end)-BasePos_B(1,100:end),'g');
xlabel('ʱ�� t/s');ylabel('��� m');
legend('Ŀ��λ�ù���ֵ���');

figure(2);
subplot(211);
plot(t(100:end),BasePos_B(4,100:end),'b',t(100:end),realData(2,100:end),'r');
title('Ŀ��Y���˶��켣');
xlabel('ʱ�� t/s');ylabel('Y��λ�� m');
legend('Ŀ��λ�ù���ֵ','Ŀ��λ��ʵ��ֵ');
subplot(212);
plot(t(100:end),realData(2,100:end)-BasePos_B(4,100:end),'g');
xlabel('ʱ�� t/s');ylabel('��� m');
legend('Ŀ��λ�ù���ֵ���');

ErrData = ErrData.*57.3;
figure(3);
subplot(211);
plot(t(10:end),angleErr(1,11:end),'b',t(10:end),ErrData(1,10:end),'r');
title('������A�Ƕ������ƽ��');
xlabel('ʱ�� t/s');ylabel('��� ��');
legend('���Ƶ����ֵ','ʵ�ʵ����ֵ');
subplot(212)
plot(t,angleErr(1,2:end)-ErrData(1,:));
title('������A�Ƕ�������ֵ�����');
xlabel('ʱ�� t/s');ylabel('��� ��');
legend('�������ֵ�����');

figure(4);
subplot(211);
plot(t(10:end),angleErr(2,11:end),'b',t(10:end),ErrData(2,10:end),'r');
title('������B�Ƕ������ƽ��');
xlabel('ʱ�� t/s');ylabel('��� ��');
legend('���Ƶ����ֵ','ʵ�ʵ����ֵ');
subplot(212)
plot(t,angleErr(2,2:end)-ErrData(2,:));
title('������B�Ƕ�������ֵ�����');
xlabel('ʱ�� t/s');ylabel('��� ��');
legend('�������ֵ�����');

figure(5);
subplot(211);
plot(t(10:end),angleErr(3,11:end),'b',t(10:end),ErrData(3,10:end),'r');
title('������C�Ƕ������ƽ��');
xlabel('ʱ�� t/s');ylabel('��� ��');
legend('���Ƶ����ֵ','ʵ�ʵ����ֵ');
subplot(212)
plot(t,angleErr(3,2:end)-ErrData(3,:));
title('������C�Ƕ�������ֵ�����');
xlabel('ʱ�� t/s');ylabel('��� ��');
legend('�������ֵ�����');

figure(6)
plot(cond(10:end));
axes('position',[0.3,0.3 0.2 0.3]);
hold on;
plot(t(10:20),cond(10:20));