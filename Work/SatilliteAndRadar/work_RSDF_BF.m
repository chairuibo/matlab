%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵�����첽�˲�
%           �˶�ģ�Ͳ���CAģ��
%�汾˵�� 1.0 ��2020-01-12 CRB��    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

%%%����ģ��
Model_CA_1 = buildModel_CA(1,3,2000);        %���Ȳ�,Ƶ�ʸ�
Model_CA_11 = buildModel_CA(1,2,2000);       %���Ȳ�,Ƶ�ʸ�
Model_CA_2 = buildModel_CA(10,2,5);         %���ȸ�,Ƶ�ʵ�
Model_CA_21 = buildModel_CA(1,2,5);         %���ȸ�,Ƶ�ʵ�  70�Ȳ����˲��������һ��
Model_CA_22 = buildModel_CA(10,3,5);         %���ȸ�,Ƶ�ʵ�
Model_CA = [Model_CA_11;Model_CA_2];                           

%%%��������
%Ŀ����ʵ����
TargetRealData = load('work_RSDF/TargetRealData3.mat');     %���ȸ�
TargetRealData.L = length(TargetRealData.TPos);
TargetRealData.Z = cell(1,TargetRealData.L/10);
TargetRealData.Z2 = cell(1,TargetRealData.L/100);
for i=10:10:TargetRealData.L
    TargetRealData.Z{i/10} = TargetRealData.TPos(:,i);
end
for i=1:100:TargetRealData.L
    TargetRealData.Z2{(i+99)/100} = TargetRealData.TPos(:,i);
end
TargetRealData.L2 =length(TargetRealData.Z2);
TargetRealData.L = TargetRealData.L/10;
% TargetRealData = rmfield(TargetRealData,'TPos');

%�״��������
RadarMasureData = load('work_RSDF/RadarMasureData3.mat'); %���Ȳ�
RadarRealData = load('work_RSDF/RadarRealData3.mat');
RadarMasureData.L = length(RadarMasureData.RMPos);
RadarMasureData.Z = cell(1,RadarMasureData.L/10);
for i=10:10:RadarMasureData.L
    RadarMasureData.Z{i/10} = RadarMasureData.RMPos(1,i);
    RadarMasureData.Z{i/10} = [RadarMasureData.Z{i/10};RadarMasureData.RMPos(3,i)];
end
RadarMasureData.L = RadarMasureData.L/10;
% RadarMasureData = rmfield(RadarMasureData,'RMPos');

%���ǲ�������
SatMasureData = load('work_RSDF/SatMasureData3.mat');     %���ȸ�,Ƶ�ʵ�
SatRealData = load('work_RSDF/SatRealData3.mat');
% SatMasureData3_add = load('work_RSDF/SatMasureData3_add.mat');���ǵ����ݼ��״��Y������
SatMasureData.L = length(SatMasureData.SMPos);
% SatMasureData3_add.L = length(SatMasureData3_add.SMPos);      ���ǵ����ݼ��״��Y������
SatMasureData.Z = cell(1,SatMasureData.L);
for i=1:SatMasureData.L
    SatMasureData.Z{i} = SatMasureData.SMPos(:,i);
    SatRealData.Z{i} = SatRealData.RealSMPos(1,i);
    SatRealData.Z{i} = [SatRealData.Z{i};0;SatRealData.RealSMPos(2,i)];
%     SatMasureData3_add.Z{i} = SatMasureData3_add.SMPos(:,i);  ���ǵ����ݼ��״��Y������
end
% SatMasureData = rmfield(SatMasureData,'SMPos');

%%%�˲��㷨
%���������������˲�
EstimateRadarValue = Kalman_Filter_4(Model_CA_11,RadarMasureData);
EstimateSatValue = Kalman_Filter_4(Model_CA_2,SatMasureData);
EstimateValue.LowFrequence = EstimateSatValue;
EstimateValue.HighFrequence = EstimateRadarValue;

%�������������ں��˲�  Ƶ�ʵ͵Ĵ���������У��
MeasureValue.LowFrequence = SatMasureData;
MeasureValue.HighFrequence= RadarMasureData;
%ֱ��ʹ�ò����˲���
% EstimateFusionValue_01 = EstiFusion_CentralizedParallel_2(Model_CA,MeasureValue,2);
% �����˲��㷨����ʵ���������Ĳ����˲��㷨���ϱ߲��ǣ�����������õ�Ƶ׼ȷ����Ϣ������Ӧλ�ø�Ƶ����Ϣ��
EstimateFusionValue_02 = AsynChroFusion_highAndlowFrequencies_02(Model_CA,MeasureValue,2);
% �ֲ�ʽ��Ƶ��������Ԥ��
EstimateFusionValue_03 = AsynChroFusion_PredictFusion(Model_CA_21,EstimateValue,EstimateFusionValue_02,2);

%%%���������
%�����˲��ںϽ��
% Title_01.x = '�����˲��ںϽ��(��������) x�᷽��';
% Title_01.z = '�����˲��ںϽ��(��������) z�᷽��';
% q = output_DemoCKParallel(0,Model_CA_1,TargetRealData.Z2,MeasureValue,EstimateFusionValue_01,Title_01);
%�򵥲����ںϽ���͵������״����ݱȽ�
Title_02.x = 'Parallel Filtering Result X-axis';
Title_02.z = 'Parallel Filtering Result Z-axis';
Title_02.picture = 'Parallel Filtering Results';
q = output_workRSDF(0,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_02.x,2,Title_02);
%�������������ŷֲ�ʽ�ںϽ���͵������״����ݱȽ�
Title_03.x = 'Distributed Filtering Result X-axis';
Title_03.z = 'Distributed Filtering Result Z-axis';
Title_03.picture = 'Distributed Filtering Results';
q = output_workRSDF(q,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_03,2,Title_03);

%��������������ʹ��kalman�˲������
for i=10:10:RadarMasureData.L*10
    RadarMasureData.Z{i/10} = RadarMasureData.RMPos(1,i);
    RadarMasureData.Z{i/10} = [RadarMasureData.Z{i/10};RadarMasureData.RMPos(3,i)];
    RadarMasureData.Z{i/10} = [RadarMasureData.Z{i/10};RadarMasureData.RMPos(2,i)];
end
EstimateRadarValue.x = Kalman_Filter(Model_CA_1,RadarMasureData);
Title_04.x = 'Radar Data Filtering Result x-axis';
Title_04.z = 'Radar Data Filtering Result Z-axis';
Title_04.y = 'Radar Data Filtering Result Y-axis';
Title_04.picture = 'Radar Data Filtering Results';
q = output_workRSDF(q,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateRadarValue.x,3,Title_04);
Title_05.x = 'Sat Data Filtering Result x-axis';
Title_05.z = 'Sat Data Filtering Result z-axis';
Title_05.picture = 'Sat Data Filtering Results';
q = output_workRSDF(q,Model_CA_2,SatRealData.Z,SatMasureData,EstimateSatValue.x,2,Title_05);

%���������ǵ����ݼ��״��Y�����ݽ����˲�
% Title_05.x = 'Sat Data Filtering Result x-axis';
% Title_05.z = 'Sat Data Filtering Result z-axis';
% Title_05.y = 'Radar Data Filtering Result Y-axis';
% Title_05.picture = 'Sat Data Filtering Results';
% EstimateSat3Value.x = Kalman_Filter(Model_CA_22,SatMasureData3_add);
% q = output_workRSDF(0,Model_CA_22,TargetRealData.Z2,SatMasureData3_add,EstimateSat3Value.x,3,Title_05);
