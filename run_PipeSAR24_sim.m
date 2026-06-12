%% run_PipeSAR24_sim.m
% 一键运行：加载参数 -> (必要时)生成模型 -> 仿真 -> 动态/静态性能测试
%
% 仿真结束后 base workspace / out 中可得到（供你的校准算法使用）：
%   sim_D1   [num x N1]  第一级原始数字码（MSB first）
%   sim_D2   [num x N2]  第二级原始数字码
%   sim_D3   [num x N3]  第三级原始数字码
%   sim_Dout [num x 1]   数字后端输出码（默认理想重组，0 ~ 2^N-1 量级）
%   sim_Vout [num x 1]   折算输出电压
%   sim_Vres1p/sim_Vres1n 第一级余差电压（调试/基于余差的校准用）
%   sim_Vres2p/sim_Vres2n 第二级余差电压

%% 0. 校准权重保留开关
% 先在 workspace 中设 use_calibrated_W = 1 再运行本脚本，可保留当前
% workspace 中已校准的 W1/W2/W3/OFS（不被 PipeSAR_params 重置为理想值），
% 用于把估计出的权重代回 Simulink 模型重新仿真验证。默认 0 = 重置。
if ~exist('use_calibrated_W','var'), use_calibrated_W = 0; end
if use_calibrated_W
    if exist('W1','var') && exist('W2','var') && exist('W3','var') && exist('OFS','var')
        W_cal = struct('W1',W1,'W2',W2,'W3',W3,'OFS',OFS);
    else
        warning('use_calibrated_W=1 但 workspace 中没有 W1/W2/W3/OFS，改用理想权重。');
        use_calibrated_W = 0;
    end
end
clearvars -except use_calibrated_W W_cal; clc; close all;

mdl = 'PipeSAR24';

%% 1. 加载参数
PipeSAR_params;
if use_calibrated_W
    W1 = W_cal.W1; W2 = W_cal.W2; W3 = W_cal.W3; OFS = W_cal.OFS;
    fprintf('use_calibrated_W = 1：沿用 workspace 中已校准的 W1/W2/W3/OFS。\n');
end

%% 2. 生成/加载模型
if ~exist([mdl '.slx'],'file')
    build_PipeSAR24_model(mdl);
else
    if ~bdIsLoaded(mdl), load_system(mdl); end
    % 旧版模型缺少第二级余差记录/分级噪声参数，检测到后自动重建
    if getSimulinkBlockHandle([mdl '/log_Vres2p']) == -1
        fprintf('检测到旧版 %s.slx（缺少 log_Vres2p），自动重建模型...\n', mdl);
        build_PipeSAR24_model(mdl);
    end
end

%% 3. 仿真
fprintf('开始仿真：%d 个采样点 @ fs = %g kHz ...\n', num, fs/1e3);
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

getv = @(name) squeeze(out.get(name));   % 兼容 2-D / 3-D 记录格式
sim_D1   = reshapeCodes(getv('sim_D1'),  N1);
sim_D2   = reshapeCodes(getv('sim_D2'),  N2);
sim_D3   = reshapeCodes(getv('sim_D3'),  N3);
sim_Dout = getv('sim_Dout'); sim_Dout = sim_Dout(:);
sim_Vout = getv('sim_Vout'); sim_Vout = sim_Vout(:);
sim_Vres1p = getv('sim_Vres1p'); sim_Vres1p = sim_Vres1p(:);
sim_Vres1n = getv('sim_Vres1n'); sim_Vres1n = sim_Vres1n(:);
sim_Vres2p = getv('sim_Vres2p'); sim_Vres2p = sim_Vres2p(:);
sim_Vres2n = getv('sim_Vres2n'); sim_Vres2n = sim_Vres2n(:);

fprintf('仿真完成。原始码矩阵: D1 %dx%d, D2 %dx%d, D3 %dx%d\n', ...
    size(sim_D1), size(sim_D2), size(sim_D3));

%% 4. 数字校准接入点（离线方式）
% 默认调用模板 My_Digital_Calibration（当前 = 理想重组，与 Simulink 内
% Digital_Backend 结果一致）。把你的算法写进该函数即可。
Dout_cal = My_Digital_Calibration(sim_D1, sim_D2, sim_D3, N, N1, N2, N3);

% 校验：离线理想重组应与 Simulink 后端输出一致
err_check = max(abs(Dout_cal - sim_Dout));
if use_calibrated_W
    fprintf('离线理想重组与模型后端输出最大差值: %g (模型使用校准权重，差值反映校准修正量)\n', err_check);
else
    fprintf('离线重组与模型后端输出最大差值: %g (默认应为 0)\n', err_check);
end

%% 5. 动态性能测试（复用原 Dynamic_test.m）
Nsample = num;
En_plot = 1;          % 是否绘图
wid     = 0;          % 0=不加窗, 1=hamming, 2=hann
[THD,SFDR,SNR,SNDR,ENOB] = Dynamic_test(Dout_cal', fs, Nsample, En_plot, wid);
fprintf('SNDR = %.2f dB,  SFDR = %.2f dB,  ENOB = %.2f bit\n', SNDR, SFDR, ENOB);

%% 6. 静态性能测试（可选）
% 注意：24-bit 共 2^24 个码道，2^15 个采样点远不足以做直方图法 DNL/INL，
% 仅在大幅增加 num（>> 2^26）后才有意义，默认关闭。
do_static = 0;
if do_static
    [DNLmax,DNLmin,INLmax,INLmin] = Static_test(Dout_cal', N); %#ok<UNRCH>
end

%% 7. 保存原始数据供离线算法开发
save('PipeSAR24_rawdata.mat','sim_D1','sim_D2','sim_D3','sim_Dout', ...
     'sim_Vout','sim_Vres1p','sim_Vres1n','sim_Vres2p','sim_Vres2n', ...
     'fs','fin','num','N','N1','N2','N3','W1','W2','W3','OFS');
fprintf('原始码已保存到 PipeSAR24_rawdata.mat\n');

%% ---------- 本地函数 ----------
function M = reshapeCodes(M, Nb)
% 把 To Workspace 记录的码流整理成 [num x Nb]
if ndims(M) == 3
    M = squeeze(M)';   % [1 x Nb x num] -> [num x Nb]
end
if size(M,2) ~= Nb && size(M,1) == Nb
    M = M';
end
end
