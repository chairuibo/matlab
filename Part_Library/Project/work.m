%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵����  �첽�˲�
%           �˶�ģ�Ͳ���CAģ��
%�汾˵��   1.0 ��2019-12-29 CRB��    ������˲����̵Ļ������޸ĵõ�
%           1.1��2019-12-29 CRB��    �����ά�޷���������
%           1.2 (2019-12-30 CRB)     ��������ںϷ�����highAndLow...�ķ���Ч���ȽϺ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

Model_CA_1 = buildModel_CA(1,2,2000);        %���Ȳ�,Ƶ�ʸ�
Model_CA_2 = buildModel_CA(10,2,15);         %���ȸ�,Ƶ�ʵ�
Model_CA = [Model_CA_1;Model_CA_2];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
%%%%%Ŀ����ʵ����
TargetRealData = load('work/TargetRealData.mat');     %���ȸ�
TargetRealData.L = length(TargetRealData.TPos);
TargetRealData.Z = cell(1,TargetRealData.L);
TargetRealData.Z2 = cell(1,TargetRealData.L/10);
for i=1:TargetRealData.L
    TargetRealData.Z{i} = TargetRealData.TPos(:,i);
end
for i=10:10:TargetRealData.L
    TargetRealData.Z2{i/10} = TargetRealData.TPos(:,i);
end
TargetRealData.L2 =length(TargetRealData.Z2);
TargetRealData = rmfield(TargetRealData,'TPos');

%%%%%�״��������
RadarMasureData = load('work/RadarMasureData.mat'); %���Ȳ�
RadarMasureData.L = length(RadarMasureData.RMPos);
RadarMasureData.Z = cell(1,RadarMasureData.L);
for i=1:RadarMasureData.L
    RadarMasureData.Z{i} = RadarMasureData.RMPos(1,i);
    RadarMasureData.Z{i} = [RadarMasureData.Z{i};RadarMasureData.RMPos(3,i);];
end
RadarMasureData = rmfield(RadarMasureData,'RMPos');

%%%%%���ǲ�������
SatMasureData = load('work/SatMasureData.mat');     %���ȸ�,Ƶ�ʵ�
SatMasureData.L = length(SatMasureData.SMPos);
SatMasureData.Z = cell(1,SatMasureData.L);
for i=1:SatMasureData.L
    SatMasureData.Z{i} = SatMasureData.SMPos(:,i);
end
SatMasureData = rmfield(SatMasureData,'SMPos');

%%%���������������˲�
% MeasureValue = [RadarMasureData.Z(10:10:end)',SatMasureData.Z'];
% MeasureValue = [RadarMasureData.RMPos;SatMasureData.SMPos];
EstimateRadarValue = Filter_Kalman(Model_CA_1,RadarMasureData);
EstimateSatValue = Filter_Kalman(Model_CA_2,SatMasureData);

%%%�������������ں��˲�  Ƶ�ʵ͵Ĵ���������У��
MeasureValue.LowFrequence = SatMasureData;
MeasureValue.HighFrequence= RadarMasureData;

EstimateFusionValue = Kalman_CentralizedFusionParallel_2(Model_CA,MeasureValue,2);
EstimateFusionValue_01 = Kalman_highAndlowFrequencies(Model_CA,MeasureValue,2);

EstimateFusionValue_temp = cell2mat(EstimateFusionValue);
EstimateFusionValue_temp(2,:) = EstimateFusionValue_temp(4,:);
MeasureValue.LowFrequence.Z = num2cell(EstimateFusionValue_temp(1:2,:),1);
EstimateFusionValue_02 = Kalman_highAndlowFrequencies(Model_CA,MeasureValue,2);
% EstimateFusionValue_02 = Kalman_ISMAsynchronousFusion(Model_CA,MeasureValue,2);
% EstimateFusionValue_03 = Kalman_PredictFusion(Model_CA,MeasureValue,2);


%%%%%���������
q = output_work(1,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateRadarValue);
q = output_work(q+1,Model_CA_2,TargetRealData.Z2,SatMasureData,EstimateSatValue);
q = output_DemoCKParallel(q+1,Model_CA_1,TargetRealData.Z2,MeasureValue,EstimateFusionValue);
q = output_work(q+1,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_02);
MeasureValue.LowFrequence = SatMasureData;
MeasureValue.HighFrequence= RadarMasureData;
q = output_work(q+1,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_01);
% q = output_work(q+1,Model_CA_1,TargetRealData.Z,RadarMasureData,EstimateFusionValue_03);
% output_work(2,SatMasureData.K,TargetRealData.TPos(1,10:10:end),SatMasureData.Z,EstimateSatValue);
% output_temp(3,SatMasureData.K,TargetRealData.TPos(1,10:10:end),MeasureValue,EstimateFusionValue);
