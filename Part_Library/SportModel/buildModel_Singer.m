function model = buildModel_Singer(time,dim,x0,alpha,w_precision)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�������ƣ�model = buildModel_Singer(time,dim,x0,w_precision)
%����˵��������CV�˶�ģ��,ģ�͵Ĺ۲�����������Ҫ���ⲿ����
%����˵����1��time  ����ʱ��
%         2��dim ģ��״̬��������
%         3��x0 ��ֵ
%         4��alpha ����ʱ�䳣���ĵ�������1/60ΪĿ�������ת�� 1/20~30 ΪĿ��رܲ���
%         5��w_precision ϵͳ��� sigma��ƽ�� 
%               ���㹫ʽ��w_precision=A_max^2/3*(1+4*P_max-P0) A_max Ŀ�������ٶ�
%             P_max Ŀ�������ٵĸ���       P0  Ŀ��û�м��ٵĸ���
%�汾˵��   1.0 ��2019-12-25 CRB 18235107312��    �����ļ�
%           1.1 ��2020-01-13 CRB��    ����Ŀ��״̬ά�������������
%��Ȩ˵���������󾫵���ӵ�б���������Ȩ������ѧϰʹ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % dynamical model parameters (Singer model)
    model.T= time;   
    model.sigma_v = w_precision;
    model.alpha=alpha; 
    model.AS = model.sigma_v.*model.alpha;
    % initial value
    model.x0=x0;
    model.p0=diag([2 2 2]).^2;
    % basic parameters
    model.x_dim = dim;   %dimension of state vector

    %dynamical model parameters         
    Alpha_T = model.alpha*model.T;
    Exp_Alpha_T = exp(-Alpha_T);
    model.F = [1 model.T  (Exp_Alpha_T+Alpha_T-1)/(model.alpha^2)
              0     1    (1-Exp_Alpha_T)/model.alpha
              0     0    Exp_Alpha_T];  %state transfer matrix
          
    q11 = (1-exp(-Alpha_T*2)+2*Alpha_T+2/3*Alpha_T^3-2*Alpha_T^2-4*Alpha_T*Exp_Alpha_T)/(2*model.alpha^5);
    q12 = (exp(-Alpha_T*2)+1-2*Exp_Alpha_T+2*Alpha_T*Exp_Alpha_T-2*Alpha_T+Alpha_T^2)/(2*model.alpha^4);
    q13 = (1-exp(-Alpha_T*2)-2*Alpha_T*Exp_Alpha_T)/(2*model.alpha^3);
    q22 = (4*Exp_Alpha_T-3-exp(-Alpha_T*2)+2*Alpha_T)/(2*model.alpha^3);
    q23 = (exp(-Alpha_T*2)+1-2*Exp_Alpha_T)/(2*model.alpha^2);
    q33 = (1-exp(-Alpha_T*2))/(2*model.alpha);          
    model.B0 = [q11 q12 q13
           q12 q22 q23
           q13 q23 q33];   %process noise covariance
    model.H= [ 1 0 0 ];    %observation matrix

    %multi-state variables 
    if model.x_dim ==1
        model.B = model.AS*model.B0;   %process noise covariance
    elseif model.x_dim ==2
        model.p0 = [model.p0 zeros(3) 
                    zeros(3) model.p0 ];
        model.F = [model.F    zeros(3)   
                    zeros(3)  model.F ];
        model.B = [model.AS(1)*model.B0  zeros(3,1) 
                   zeros(3,1)   model.AS(2)*model.B0  ];
        model.H = [ model.H     zeros(1,3)
                    zeros(1,3)    model.H ];    %observation matrix
    elseif model.x_dim ==3        
        model.p0=[model.p0 zeros(3) zeros(3)
                  zeros(3) model.p0   zeros(3)   
                  zeros(3) zeros(3)   model.p0];
        model.F = [model.F     zeros(3)    zeros(3)
                   zeros(3)    model.F    zeros(3)
                   zeros(3)    zeros(3)   model.F];
        model.B = [model.AS(1)*model.B0      zeros(3,1)  zeros(3,1)
                   zeros(3,1)   mmodel.AS*model.B0     zeros(3,1)
                   zeros(3,1)   zeros(3,1)  model.AS*model.B0];    
        model.H = [ model.H       zeros(1,3)    zeros(1,3)
                    zeros(1,3)    model.H       zeros(1,3)
                    zeros(1,3)    zeros(1,3)    model.H];    %observation matrix
    end
    model.Q= 2*model.B;   %process noise covariance
end
