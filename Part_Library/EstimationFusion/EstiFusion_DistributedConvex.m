function [ estimate_data ] = EstiFusion_DistributedConvex( localEstimate_data )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵��:�ֲ�ʽ�ں�ϵͳ����͹����ں��㷨
%����˵��:measure_value ���д���������������
%           -measure_value���ݸ�ʽҪ�󣺰����������� 
%           -LΪ���ݳ��ȣ�
%           -countΪ����������
%           -dataΪÿ���������ֲ��������ݣ�����x��P��
%�汾˵����1.0 ��2020-01-10 CRB��    �����ļ�
%         1.1 ��2020-01-13 CRB��    �޸�SingleStepMultiple����
%                   �еĴ���P��������ʹ����P�����½���պ��෴
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    count = localEstimate_data.count;
    Data_L = localEstimate_data.L;
    estimate_data = cell(Data_L,1);
    measure_value.x = cell(Data_L,count);
    measure_value.P = cell(Data_L,count);
  
 %%%��������   
    for i=1:Data_L
        for j = 1:count 
            measure_value.z{i,j} = localEstimate_data.data(j).x{i};
            measure_value.P{i,j} = localEstimate_data.data(j).P{i};
        end
    end
    
 %%%�˲�������  
    for i=1:Data_L
        estimate_data{i} = SingleStepMultiple(measure_value.z(i,:),measure_value.P(i,:),count);
    end
end

%��������,�㷨������Ԫ
function  x_temp = SingleStepMultiple( x_PartEstimateValue,P_PartEstimateValue,SensorCount)
    x_temp = 0;
    P_temp = 0;
    Px_temp = 0;
    for j=1:SensorCount
        P_temp = P_temp + inv(P_PartEstimateValue{j});
        Px_temp = Px_temp + P_PartEstimateValue{j}\x_PartEstimateValue{j};
    end
    x_temp = P_temp\Px_temp;
end

