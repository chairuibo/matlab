function SmoothValue = Kalman_Smooth_2(model,EstimateValue)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%程序说明:基本卡尔曼平滑方程
%输入参数说明:1、model  运动模型 
%               model.F 状态转移矩阵
%            2、EstimateValue  滤波结果
%                1)EstimateValue.x 状态向量的滤波值
%                2)EstimateValue.x_p 状态向量的预测值
%                3)EstimateValue.P 估计误差协方差阵P(k/k)
%                4)EstimateValue.P_p 估计误差协方差阵P(k+1/k)
%     EstimateValue变量每个时刻的数据放在一个元素胞组中，
%     n个时刻数据为一行n列
%输出数据说明：SmoothValue  输出平滑结果
%               SmoothValue.x 状态向量的平滑值
%     输出格式：1)每个时刻的结果放在一个元素胞组内
%              3)n时刻数据为一行n列
%              2)每个胞组中状态向量为n行一列
%版本说明:1.0 （2019-4-8 CRB 18235107312）建立文件
%版权信息：西工大精导所拥有本程序所有权，仅供学习使用
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
