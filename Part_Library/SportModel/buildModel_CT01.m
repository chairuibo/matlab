function model = buildModel_CT01(time,dim,x0,omiga,w_precision)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�������ƣ�model = buildModel_CA(time,dim,x0,w_precision)
%����˵����Ŀ����ٶ�δ֪��CT�˶�ģ�ͣ�ģ�͵Ĺ۲�����������Ҫ���ⲿ����
%�������˵����1��time  ��������
%             2��dim ģ��״̬��������
%             3��x0 ϵͳ��ֵ
%             4��Omiga Ŀ��Ľ��ٶ�
%             4��w_precision ϵͳ���(��Ҫ��������ά���Ĳ���)
%�������˵����model ������ģ��
%               model.T ��������
%               model.F ״̬ת�ƾ���
%               model.Q ϵͳ������
%               model.H �۲����
%               model.x0״̬������ʼֵ
%               model.p0.^2;Э��������ʼֵ
%�汾˵����1.0 ��2019-12-25 CRB 18235107312��    �����ļ�
%��Ȩ˵���������󾫵���ӵ�б���������Ȩ������ѧϰʹ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % dynamical model parameters (CT model)
    model.T= time;                %sampling period
    model.Sigma_v = w_precision;  
    model.Omiga = omiga;
    % initial
    model.x0=x0;                  %state initial value
    
    % basic parameters
    model.x_dim= dim;   %dimension of state vector        
    model.sinwT = sin(model.Omiga*model.T);
    model.coswT = cos(model.Omiga*model.T);
    
    %multi-state variables 
    if model.x_dim ==2
        model.p0=diag([100 1]).^2; %process noise covariance    
        model.p0 = [model.p0 zeros(3) 
                    zeros(3) model.p0 ];
        model.B0= [(model.T^2)/2; model.T];  %process noise parameters
        model.H0 = [ 1 0 ];    %observation matrix
        
        model.F = [1    model.sinwT/model.Omiga     0   -(1-model.coswT)/model.Omiga 
                   0    model.coswT                 0   -model.sinwT
                   0    (1-model.coswT)/model.Omiga 1   model.sinwT/model.Omiga
                   0    model.sinwT                 0   model.coswT];
        
        model.B = [model.Sigma_v(1)*model.B0  zeros(2,1) 
                   zeros(2,1)   model.Sigma_v(2)*model.B0  ];
        model.H = [ model.H0      zeros(1,2)
                    zeros(1,2)    model.H0 ];    %observation matrix
    elseif model.x_dim ==3
        model.F= [1     model.sinwT/model.Omiga     (1-model.sinwT)/(model.Omiga^2)
                  0     model.coswT                 model.sinwT/model.Omiga
                  0     -model.sinwT*model.Omiga    model.coswT ];  %state transfer matrix
        model.B0= [(model.T^3)/6; (model.T^2)/2; model.T];  %process noise parameters
        model.H0 = [ 1 0 0 ];    %observation matrix
        model.p0=diag([100 10 1]).^2; %process noise covariance
        model.p0=[model.p0 zeros(3) zeros(3)
                  zeros(3) model.p0   zeros(3)   
                  zeros(3) zeros(3)   model.p0];
        model.F = [model.F     zeros(3)    zeros(3)
                   zeros(3)    model.F    zeros(3)
                   zeros(3)    zeros(3)   model.F];
        model.B = [model.Sigma_v(1)*model.B0      zeros(3,1)  zeros(3,1)
                   zeros(3,1)   model.Sigma_v(2)*model.B0     zeros(3,1)
                   zeros(3,1)   zeros(3,1)  model.Sigma_v(3)*model.B0];    
        model.H = [ model.H0       zeros(1,3)    zeros(1,3)
                    zeros(1,3)    model.H0       zeros(1,3)
                    zeros(1,3)    zeros(1,3)    model.H0];    %observation matrix
    end
    model.Q= model.B*model.B';   %process noise covariance
end
