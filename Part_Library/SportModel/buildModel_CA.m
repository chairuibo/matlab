function model = buildModel_CA(time,dim,x0,w_precision)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�������ƣ�model = buildModel_CA(time,dim,x0,w_precision)
%����˵��������Ŀ���˶�ģ�ͣ�ģ�͵Ĺ۲�����������Ҫ���ⲿ����
%�������˵����1��time  ��������
%             2��dim ģ��״̬��������
%             3��x0 ϵͳ��ֵ
%             4��w_precision ϵͳ���(��Ҫ��������ά���Ĳ���)
%�������˵����model ������ģ��
%               model.T ��������
%               model.F ״̬ת�ƾ���
%               model.Q ϵͳ������
%               model.H �۲����
%               model.x0״̬������ʼֵ
%               model.p0.^2;Э��������ʼֵ
%�汾˵����1.0 ��2019-12-25 CRB 18235107312��    �����ļ�
%         1.1��2019-12-29 CRB��  ��Ӳ������޸Ĳ��ֽṹ���ĵ������Ϊ��ѡ����
%         1.2 (2020-06-23 CRB)   ��Ӳ���
%		  1.3 (2020-12-08 CRB)	 ɾ���������������ز�������Ϊ�����⸳ֵ
%��Ȩ˵���������󾫵���ӵ�б���������Ȩ������ѧϰʹ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % dynamical model parameters (CA model)
    model.T= time;                %sampling period
    model.sigma_v = w_precision;  
    % initial
    model.x0=x0;                  %state initial value
    model.p0=diag([100 10 1]).^2; %process noise covariance    
    % basic parameters
    model.x_dim= dim*3;   %dimension of state vector
    model.w_dim= dim*3;   %dimension of observation noise
    %dynamical model parameters         
    model.F= [1 model.T (model.T^2)/2
              0     1   model.T
              0     0      1];  %state transfer matrix
    model.B0= [(model.T^3)/6; (model.T^2)/2; model.T];  %process noise parameters
    % observation model parameters 
    model.H= [ 1 0 0 ];    %observation matrix
    %multi-state variables 
    if dim ==1
        model.B = model.sigma_v*model.B0;
    elseif dim ==2
        model.p0 = [model.p0 zeros(3) 
                    zeros(3) model.p0 ];
        model.F = [model.F    zeros(3)   
                    zeros(3)  model.F ];
        model.B = [model.sigma_v(1)*model.B0  zeros(3,1) 
                   zeros(3,1)   model.sigma_v(2)*model.B0  ];
        model.H = [ model.H     zeros(1,3)
                    zeros(1,3)    model.H ];    %observation matrix
    elseif dim ==3
        model.p0=[model.p0 zeros(3) zeros(3)
                  zeros(3) model.p0   zeros(3)   
                  zeros(3) zeros(3)   model.p0];
        model.F = [model.F     zeros(3)    zeros(3)
                   zeros(3)    model.F    zeros(3)
                   zeros(3)    zeros(3)   model.F];
        model.B = [model.sigma_v(1)*model.B0      zeros(3,1)  zeros(3,1)
                   zeros(3,1)   model.sigma_v(2)*model.B0     zeros(3,1)
                   zeros(3,1)   zeros(3,1)  model.sigma_v(3)*model.B0];    
        model.H = [ model.H       zeros(1,3)    zeros(1,3)
                    zeros(1,3)    model.H       zeros(1,3)
                    zeros(1,3)    zeros(1,3)    model.H];    %observation matrix
    end
    model.Q= model.B*model.B';   %process noise covariance
end
