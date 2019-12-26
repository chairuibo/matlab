function truth= getTruthData(model)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明：  根据模型生成真实数据
%参数说明   model 运动模型
%版本说明   1.0 （2019-12-25 CRB）    建立文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %variables
    truth.K= 500;                   	%length of data/number of scans
    truth.X= cell(truth.K,1);         	%ground truth for states of targets  
    truth.N= zeros(truth.K,1);          %ground truth for number of targets
%     truth.L= cell(truth.K,1);        	%ground truth for labels of targets (k,i)
%     truth.track_list= cell(truth.K,1);	%absolute index target identities (plotting)
    %target initial states and birth/death times
    targetstate = model.x0;
    for k=1:truth.K
        targetstate = gen_newstate(model,targetstate,'noiseless');
        truth.X{k}= [truth.X{k} targetstate];
%         truth.track_list{k} = [truth.track_list{k} k];
        truth.N(k) = truth.N(k) + 1;
    end 
end

function X= gen_newstate(model,Xd,style)
    if ~isnumeric(style)
        if strcmp(style,'noise')
            W= model.sigma_v*model.B*randn(size(model.B,2),size(Xd,2));
        elseif strcmp(style,'noiseless')
            W= zeros(size(model.B,1),size(Xd,2));
        end
    end
    if isempty(Xd)
        X= [];
    else
        X= model.F*Xd+W;
    end
end