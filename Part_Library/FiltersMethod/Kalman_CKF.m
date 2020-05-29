function estimate_value = Kalman_CKF(model,measure_value,X_fun,Z_fun)
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
         [X_pre,P_pre] = x_time_update(X_fun,est_temp,P_temp,model.Q);      %ʱ�����-״̬Ԥ��
         [Z_pre,P_zz,P_xz] = z_time_update(Z_fun,X_pre,P_pre,model.R);      %ʱ�����-����Ԥ��
         [est_temp,P_temp] = measure_update(measure_value.Z{k},X_pre,P_pre,Z_pre,P_zz,P_xz);%��������
         est_x_update{k}=est_temp;
         P_update{k}=P_temp;
    end 
    %%%������
    estimate_value.X =est_x_update;
    estimate_value.P =P_update;
end

%ϵͳ״̬Ԥ��
function [X_Pre,P_Pre] = x_time_update(X_fun,x_est,P_temp,Q)  

    n_x =length(x_est);
    C = sqrt(n_x)*[eye(n_x) -eye(n_x)];  %%�����ݻ��㼯
    S_est = chol(P_temp,'lower');
    x_EstiTemp_mat = repmat(x_est,1,2*n_x);
    x_CabutPointSet = S_est*C+x_EstiTemp_mat;    %�����ݻ��㼯
    X_temp =X_fun(x_CabutPointSet);              %���㴫���ݻ��㼯
    X_Pre = X_temp*ones(2*n_x,1)/(2*n_x);        %����״̬������һ��Ԥ��ֵ
    P_Pre= X_temp*X_temp'/(2*n_x)-X_Pre*X_Pre'+Q;   %�������Э����һ��Ԥ��ֵ
end

%����Ԥ��
function [Z_Pre,P_zz,P_xz] = z_time_update(Z_fun,X_pre,P_pre,R)

    n_x =length(X_pre);
    C =sqrt(n_x)*[eye(n_x) -eye(n_x)];      %�����ݻ��㼯
    S_Pre =chol(P_pre,'lower');             
    X_Pre_mat =repmat(X_pre,1,2*n_x);       
    x_temp =S_Pre*C+X_Pre_mat;              %�����ݻ��㼯
    Z_temp =Z_fun(x_temp);                  %���㴫���ݻ��㼯
    Z_Pre = Z_temp*ones(2*n_x,1)/(2*n_x);   %���������һ��Ԥ��ֵ
    P_zz= Z_temp*Z_temp'/(2*n_x)-Z_Pre*Z_Pre'+R;  %������ϢЭ�������
    P_xz = x_temp*Z_temp'/(2*n_x)-X_pre*Z_Pre';   %���ƽ���Э��������һ��Ԥ��ֵ
end

%�˲�
function [est_temp,P_temp] = measure_update(Z_meas,x_pre,P_pre,z_pre,P_zz,P_xz)

    K = P_xz/P_zz;                        %���㿨�����������
    est_temp = x_pre+ K*(Z_meas - z_pre); %���Ƶ�ǰʱ�̵�״̬����
    P_temp = P_pre - K*P_zz*K';           %���Ƶ�ǰʱ�̵����Э�������
end
