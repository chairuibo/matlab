function model = buildModel_CV(time,dim,x0,w_precision)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�������ƣ�model = buildModel_CV(time,dim,x0,w_precision)
%����˵��������CV�˶�ģ��,ģ�͵Ĺ۲�����������Ҫ���ⲿ����
%����˵����1��time  ����ʱ��
%         2��dim ģ��״̬��������
%         3��x0 ��ֵ
%         4��w_precision ϵͳ���(��Ҫ��������ά���Ĳ���)
%�汾˵��   1.0 ��2019-12-25 CRB��    �����ļ�
%           1.1 ��2020-01-13 CRB��  ������������������޸Ĳ��ֽṹ���ĵ������Ϊ��ѡ����
%           1.2 ��2020-05-24 CRB��  ���ϵͳ������
%           1.3  (2020-12-12 CRB)   ���������
%��Ȩ˵���������󾫵���ӵ�б���������Ȩ������ѧϰʹ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % dynamical model parameters (CV model)
    model.T= time;   
    model.sigma_v = w_precision; %process noise parameter
    % initial value
    model.x0=x0;
    model.p0=diag([100 100]).^2;
    % basic parameters
    model.x_dim= dim;   %dimension of state vector
    %dynamical model parameters         
    model.F= [1  model.T
              0     1  ];  %state transfer matrix
    model.B= [(model.T^2)/2; model.T];  
    model.Q= (model.sigma_v)^2* model.B*model.B';   %process noise covariance
    % observation model parameters 
    model.H= [ 1 0 ];    %observation matrix

    %multi-state variables 
    if model.x_dim == 1
        model.B0= model.sigma_v*model.B;
    elseif model.x_dim ==2
        model.p0 = [model.p0 zeros(2) 
                    zeros(2) model.p0 ];
        model.F = [model.F    zeros(2)   
                    zeros(2)  model.F ];
        model.B = [model.sigma_v(1)*model.B  zeros(2,1) 
                   zeros(2,1)   model.sigma_v(2)*model.B  ];    
        model.H= [model.H   zeros(1,2) 
                  zeros(1,2)   model.H   ];
    elseif model.x_dim ==3
        model.p0=[model.p0  zeros(2) zeros(2)
                  zeros(2)  model.p0  zeros(2)   
                  zeros(2)  zeros(2)  model.p0];
        model.F = [model.F     zeros(2)    zeros(2)
                   zeros(2)    model.F    zeros(2)
                   zeros(2)    zeros(2)   model.F];
        model.B = [model.sigma_v(1)*model.B      zeros(2,1)  zeros(2,1)
                   zeros(2,1)   model.sigma_v(2)*model.B     zeros(2,1)
                   zeros(2,1)   zeros(2,1)  model.sigma_v(3)*model.B];    
        model.H= [model.H       zeros(1,2)  zeros(1,2)
                  zeros(1,2)   model.H     zeros(1,2)
                  zeros(1,2)   zeros(1,2)  model.H];
    end
    model.Q= model.B*model.B';   %process noise covariance
end
