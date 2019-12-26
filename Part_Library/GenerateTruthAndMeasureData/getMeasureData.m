function meas = getMeasureData(model,truth)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����˵����  ������ʵ����������������
%����˵��   model �˶�ģ��
%           truth ��ʵֵ
%�汾˵��   1.0 ��2019-12-25 CRB��    �����ļ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %variables
    meas.K= truth.K;
    meas.Z= cell(truth.K,1);
    %generate measurements
    for k=1:truth.K
        if truth.N(k) > 0                                
            meas.Z{k}= gen_observation(model,truth.X{k},'noise'); %single target observations if detected 
        end
 %measurement is union of detections and clutter
    end
end

function Z= gen_observation(model,X,style)
    %linear observation equation (position components only)
    if ~isnumeric(style)
        if strcmp(style,'noise')
            W= model.D*randn(size(model.D,2),size(X,2));
        elseif strcmp(style,'noiseless')
            W= zeros(size(model.D,1),size(X,2));
        end
    end
    if isempty(X)
        Z= [];
    else
        Z= model.H*X+ W;
    end
end