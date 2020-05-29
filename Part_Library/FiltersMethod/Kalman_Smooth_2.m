function SmoothValue = Kalman_Smooth_2(model,EstimateValue)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵��:����������ƽ������
%�������˵��:1��model  �˶�ģ�� 
%               model.F ״̬ת�ƾ���
%            2��EstimateValue  �˲����
%                1)EstimateValue.x ״̬�������˲�ֵ
%                2)EstimateValue.x_p ״̬������Ԥ��ֵ
%                3)EstimateValue.P �������Э������P(k/k)
%                4)EstimateValue.P_p �������Э������P(k+1/k)
%     EstimateValue����ÿ��ʱ�̵����ݷ���һ��Ԫ�ذ����У�
%     n��ʱ������Ϊһ��n��
%�������˵����SmoothValue  ���ƽ�����
%               SmoothValue.x ״̬������ƽ��ֵ
%     �����ʽ��1)ÿ��ʱ�̵Ľ������һ��Ԫ�ذ�����
%              3)nʱ������Ϊһ��n��
%              2)ÿ��������״̬����Ϊn��һ��
%�汾˵��:1.0 ��2019-4-8 CRB 18235107312�������ļ�
%��Ȩ��Ϣ�������󾫵���ӵ�б���������Ȩ������ѧϰʹ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    L=length(EstimateValue.x);
    x_smooth=cell(1,L);
    
    x_f=EstimateValue.x;
    x_p=EstimateValue.x_p;
    P_f=EstimateValue.P;
    P_p=EstimateValue.P_p;
    x_smooth{L}=x_f{L};
    
    for k=2:L
        x_smooth{k-1} = smooth_multiple(model,x_p{k},x_f{k-1},x_f{k},P_p{k},P_f{k-1});
    end
    SmoothValue.x = x_smooth(1:L-1);
end
%Kalman smooth
function x_smooth = smooth_multiple(model,xPredict,xLFilter,xFilter,PPredict,PFilter)
    F = model.F;
    xL = xLFilter;
    xF = xFilter;
    xP = xPredict;
    PP = PPredict;
    PF = PFilter;
    
    As = PF*F'/PP;
    x_smooth = xL + As*(xF - xP);
end
