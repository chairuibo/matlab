function estimate_value = Kalman_UKF(model, measure_value,X_fun,Z_fun,UT)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵��:�����������˲�����
%����˵��:1��model  �˶�ģ��
%          model.Q ϵͳ����Э������
%          model.R �۲�����Э������
%          model.p0 P���ʼֵ
%          model.x0 ״̬��������ֵ
%        2��measure_value  ���д���������������
%           measure_value.L  �������ݵĳ���
%           measure_value.Z  �����������ݣ���ʽΪ1xL�ľ������
%                            ÿ����������Nx1ά����������
%        3��X_fun ϵͳ���̺���
%        4��Z_fun ���ⷽ�̺���
%        5��UT UT�任��Ҫ�õ��Ĳ���Alpha Lamda Kappa
%            UT.Alpha UT.Lamda(���п��ޣ������޸�) UT.Kappa
%���˵����est_x_update �˲��������
%           est_x_update.X ����ʱ�̵��˲��������ʽΪ1xL�ľ������
%                          ÿ����������Nx1ά����������
%           est_x_update.P ����ʱ�̵����Э�����󣬸�ʽΪ1xL�ľ������
%                          ÿ����������NxNά���������ݣ������ںϿ��ܻ��õ���
%�汾˵��:1.0 ��2020-5-28 CRB 18235107312�������ļ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = measure_value.L;
    est_x_update = cell(1,L+1);  %ת�����󷽱�
    P_update = cell(1,L+1);

    est_temp = model.x0;
    P_temp = model.p0;
    for k=1:L
         [X_temp,w]=ut(est_temp,P_temp,UT);                             %UT�任
         [X_pre,P_pre,X_] = x_time_update(X_fun,X_temp,w,model.Q);      %ʱ�����-״̬Ԥ��
         [Z_pre,P_zz,P_xz] = z_time_update(Z_fun,X_temp,X_,w,model.R);  %ʱ�����-����Ԥ��
         [est_temp,P_temp] = measure_update(measure_value.Z{k},X_pre,P_pre,Z_pre,P_zz,P_xz);%��������
         est_x_update{k}=est_temp;
         P_update{k}=P_temp;
    end 
    %%%������
    estimate_value.X =est_x_update;
    estimate_value.P =P_update;
end

%%%UT�任  ������  ״̬������UT�任����ͼ�Ȩϵ��
function [X_temp,w]= ut(x,P,UT)
    alpha =UT.Alpha;
    kappa =UT.Kappa;
%     lambda =UT.Lamda;

    n_x= length(x);         %״̬��ά��
    lambda= alpha^2*(n_x+kappa) - n_x;
    d=(n_x+lambda)*P;       
    Psqrtm= chol(d)';       %P���cholesterol�ֽ�
    X_temp= repmat(x,[1 2*n_x+1])+ [ zeros(n_x,1) Psqrtm -Psqrtm ];
    w= [ lambda 0.5*ones(1,2*n_x) ]/(n_x+lambda);   %Ȩֵϵ��
 end

%ϵͳ״̬Ԥ��
function [X_Pre,P_Pre,X_] = x_time_update(X_fun,x_temp,w,Q)  

    X_temp =X_fun(x_temp);
    X_Pre =X_temp*w(:);        %״̬������Ԥ��ֵ
    X_ =X_temp- repmat(X_Pre,[1 length(w)]);   %��������״̬����Ԥ��ֵ���ֵ�Ĳ�
    P_Pre =X_*diag(w)*X_'+Q;  %P��Ԥ��ֵ
end

%����Ԥ��
function [Z_Pre,P_zz,P_xz] = z_time_update(Z_fun,x_temp,X_,w,R)

    Z_temp =Z_fun(x_temp);
    Z_Pre = Z_temp*w(:);        %�۲���Ԥ��ֵ
    Z_ = Z_temp- repmat(Z_Pre,[1 length(w)]); %�������Ĺ۲�����Ԥ��ֵ���ֵ�Ĳ�
    P_zz= Z_*diag(w)*Z_'+R;
    P_xz = X_*diag(w)*Z_';
end

%�˲�
function [est_temp,P_temp] = measure_update(Z_meas,x_pre,P_pre,z_pre,P_zz,P_xz)

    K = P_xz/P_zz;                          %���ƿ������������
    est_temp = x_pre+ K*(Z_meas - z_pre);   %���Ƶ�ǰʱ�̵�״̬����
    P_temp = P_pre - K*P_zz*K';             %���Ƶ�ǰʱ�̵����Э�������
end
