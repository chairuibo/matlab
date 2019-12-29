%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵����  �첽�˲�
%           �˶�ģ�Ͳ���CAģ��
%�汾˵��   1.0 ��2019-12-29 CRB��    ������˲����̵Ļ������޸ĵõ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

Model_CA_1 = buildModel_CA(1,200);        %���Ȳ�
Model_CA_2 = buildModel_CA(10,2);        %���ȸ�
Model = [Model_CA_1;Model_CA_2];
TargetRealData = load('work/TargetRealData.mat');

RadarMasureData = load('work/RadarMasureData.mat'); %���Ȳ�
RadarMasureData.K = length(RadarMasureData.RMPos);
RadarMasureData.Z = RadarMasureData.RMPos(1,:);
RadarMasureData = rmfield(RadarMasureData,'RMPos');

SatMasureData = load('work/SatMasureData.mat');     %���ȸ�
SatMasureData.K = length(SatMasureData.SMPos);
SatMasureData.Z = SatMasureData.SMPos(1,:);
SatMasureData = rmfield(SatMasureData,'SMPos');

%%%���������������˲�
MeasureValue = [RadarMasureData.Z(10:10:end)',SatMasureData.Z'];
% MeasureValue = [RadarMasureData.RMPos;SatMasureData.SMPos];
EstimateRadarValue = Filter_Kalman(Model_CA_1,RadarMasureData);
EstimateSatValue = Filter_Kalman(Model_CA_2,SatMasureData);

%%%�������������ں��˲�  Ƶ�ʵ͵Ĵ�����ʱ���Ͻ����ں�
EstimateFusionValue = Kalman_CentralizedFusionParallel_2(Model,MeasureValue,2);

output_work(1,RadarMasureData.K,TargetRealData.TPos,RadarMasureData.Z,EstimateRadarValue);
output_work(2,SatMasureData.K,TargetRealData.TPos(1,10:10:end),SatMasureData.Z,EstimateSatValue);
output_temp(3,SatMasureData.K,TargetRealData.TPos(1,10:10:end),MeasureValue,EstimateFusionValue);
% output_DemoCKParallel(Model_CA_1,TargetRealData,MeasureValue,EstimateValue);
