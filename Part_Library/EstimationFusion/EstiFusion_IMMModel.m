function ExtimateValue = EstiFusion_IMMModel_00(IMMmodel,meas,others)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�������ƣ�model = IMMModel(time,dim,x0,w_precision)
%����˵������IMM����ʽģ���˲����Ե����۲����ö��ģ���˲����˲��������޸ġ�
%   ����������ʹ�õ���CKF�˲���������ʹ��������������Ҫɾ��ԭ�˲������е�һЩ
%   ����Ρ���Ҫ�У�1�����˲������������򲿷�ɾ����ʱ�̵���ش���
%                  2�����˲��Ӻ����������Ȼ��������Ĵ���
%                  3����ÿ��ģ���˲�ǰ����һʱ�̵��˲������ģ�͵�x0��p0��������
%  ��Ҫע����ǣ��������ںϷ������ö��������������ֵ�ںϣ����������ö��ģ�͵���
% �������������ںϣ�Ҳ���Ը�����Ҫ�ö��ģ�ͺͶ�������������ݽ����ںϡ�
%�������˵����1��IMMmodel IMMģ�ͼ�����ز���
%                   IMMmodel.pai = ģ��ת��Markov��;
%                   IMMmodel.U0 = ģ�͸���;
%                   IMMmodel.countģ�ͼ���ģ������
%             2��LocalEstiData ��ģ�͵��˲�����
%                   LocalEstiData.ModelData(i).X ��ģ�͵��˲����
%                   LocalEstiData.ModelData(i).P ��ģ���˲����Э������
%�������˵����ExtimateValue �ںϺ������
%               Estimate_value.X ����ʱ�̵��˲��������ʽΪ1xL�ľ������
%                          ÿ����������Nx1ά����������
%               Estimate_value.P ����ʱ�̵����Э�����󣬸�ʽΪ1xL�ľ������
%�汾˵����1.0 ��2020-12-16 CRB 18235107312��    �����ļ�
%           1.1 (2020-12-21 CRB )  �޸�bug
%��Ȩ˵���������󾫵���ӵ�б���������Ȩ������ѧϰʹ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % ת�Ʋ���
    Pai = IMMmodel.pai;
    U = IMMmodel.U0;
    Count = IMMmodel.count;
    L = meas.L;
    Z = meas.Z;
    X_fun = others.X_fun;
    Z_fun = others.Z_fun;
    % ����Ԥ����洢�ռ�
    Est_P_temp = cell(1,Count);
    Est_temp = cell(1,Count);
    Model = cell(1,Count);
    Like = zeros(Count,1);
    Est_mixed = cell(1,Count);
    Est_P_mixed = cell(1,Count);
    % �����˲�����ֵ
    for i=1:Count
        Est_temp{i} = IMMmodel.model{i}.x0;
        Est_P_temp{i} = IMMmodel.model{i}.p0;
        Model{i} = IMMmodel.model{i};
    end
    %% �˲�
    for k=1:L
        % ��ϸ��ʼ��� 
        C_ = Pai'*U; % nx1����
        U = (1./C_)'.*Pai.*U;  
        
        % ��Ϲ���
        for j = 1:Count     %�س�ʼ����״̬��Ϲ���
            Est_mixed{j} = 0;
            for i = 1:Count
                Est_mixed{j} = Est_mixed{j}+Est_temp{i}*U(i,j);              
            end
        end

        for j = 1:Count     %�س�ʼ����Э�������Ϲ���
            Est_P_mixed{j} = 0;
            for i = 1:Count
                Xmixed_err = Est_temp{i}-Est_mixed{j};
                Est_P_mixed{j} = Est_P_mixed{j} +(Est_P_temp{i}+Xmixed_err*Xmixed_err')*U(i,j);
            end
        end
        % ģ�������˲�
        for i=1:Count
            % ���˲���������һʱ�̵��˲����
            other.count =i;
            Model{i}.x0 = Est_temp{i};
            Model{i}.p0 = Est_P_temp{i};
            % ʹ�ø�ģ���˲������滻�������˲���������Ҫ�ڸ��������м�����Ȼ�����Ĺ�ʽ������ο�measure_update�����У� 
            [Est_temp{i},Est_P_temp{i},Like(i,1)] = Kalman_CKF(Model{i},Z{k}(1:3),X_fun,Z_fun,other);
        end

        % ģ�͸��ʸ���
        C = C_'*Like;   
        U = Like.*C_/C; %ģ��ƥ�����
        
        % �����ں�
        Est_x = cell2mat(Est_temp)*U;
%         x_err = repmat(Est_x,1,Count)-cell2mat(Est_temp);
        Est_P = 0;        
        for i=1:Count
            x_err = Est_x-Est_temp{i};
            Est_P = Est_P+(Est_P_temp{i}+x_err*x_err')*U(i);
        end
        % �洢���
        ExtimateValue.X{k} = Est_x;
        ExtimateValue.P{k} = Est_P;
        ExtimateValue.U{k} = U;

    end
end

%% CKF�˲��㷨
function [Est_temp,P_temp,Like] = Kalman_CKF(model,measure_value,X_fun,Z_fun,others)
    
    Est_temp = model.x0;
    P_temp = model.p0;

    %%% ���³���ֻ��Ķ�other�������������ָ�����Ҫ�Ķ���
    [X_pre,P_pre] = x_time_update(X_fun,Est_temp,P_temp,model,others);      %ʱ�����-״̬Ԥ��
%     others.SensorPos = measure_value.SensorPos{k};
    others.k = 1;
    [Z_pre,P_zz,P_xz] = z_time_update(Z_fun,X_pre,P_pre,model.R,others);      %ʱ�����-����Ԥ��
    [Est_temp,P_temp,Like] = measure_update(measure_value,X_pre,P_pre,Z_pre,P_zz,P_xz);%��������     

end

%% ϵͳ״̬Ԥ��
function [X_Pre,P_Pre] = x_time_update(X_fun,x_est,P_temp,model,others)  
    Q = model.Q;
    
    n_x =length(x_est);
    C = sqrt(n_x)*[eye(n_x) -eye(n_x)];  %%�����ݻ��㼯
    S_est = chol(P_temp,'lower');
    x_EstiTemp_mat = repmat(x_est,1,2*n_x);
    x_CabutPointSet = S_est*C+x_EstiTemp_mat;    %�����ݻ��㼯
    X_temp =X_fun(x_CabutPointSet,model,others);              %���㴫���ݻ��㼯
    X_Pre = X_temp*ones(2*n_x,1)/(2*n_x);        %����״̬������һ��Ԥ��ֵ
    P_Pre= X_temp*X_temp'/(2*n_x)-X_Pre*X_Pre'+Q;   %�������Э����һ��Ԥ��ֵ
end

%% ����Ԥ��
function [Z_Pre,P_zz,P_xz] = z_time_update(Z_fun,X_pre,P_pre,R,other)

    n_x =length(X_pre);
    C =sqrt(n_x)*[eye(n_x) -eye(n_x)];      %�����ݻ��㼯
    S_Pre =chol(P_pre,'lower');             
    X_Pre_mat =repmat(X_pre,1,2*n_x);       
    x_temp =S_Pre*C+X_Pre_mat;                %�����ݻ��㼯
    Z_temp =Z_fun(x_temp,other); %���㴫���ݻ��㼯
    Z_Pre = Z_temp*ones(2*n_x,1)/(2*n_x);     %���������һ��Ԥ��ֵ
    P_zz= Z_temp*Z_temp'/(2*n_x)-Z_Pre*Z_Pre'+R;  %������ϢЭ�������
    P_xz = x_temp*Z_temp'/(2*n_x)-X_pre*Z_Pre';   %���ƽ���Э��������һ��Ԥ��ֵ
end

%% �˲�
function [est_temp,P_temp,Like] = measure_update(Z_meas,x_pre,P_pre,z_pre,P_zz,P_xz)

    K = P_xz/P_zz;                        %���㿨�����������
    Z_err = Z_meas - z_pre;
    est_temp = x_pre+ K*Z_err; %���Ƶ�ǰʱ�̵�״̬����
    P_temp = P_pre - K*P_zz*K';           %���Ƶ�ǰʱ�̵����Э�������
    
    Like = det(2*3.1415926*P_zz)^(-0.5)*exp(-0.5*Z_err'/P_zz*Z_err); %��Ȼ��������Ҫ���
end