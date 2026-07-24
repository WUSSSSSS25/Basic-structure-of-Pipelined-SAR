%% Static_test_DCsweep.m
% ===================================================================
%  DC 扫描法静态测试 (DNL / INL)，适用于高分辨率(24-bit)流水线 SAR
% ===================================================================
% 本脚本【直接驱动已建好的 PipeSAR24.slx 模型】完成静态测试——不再重复
% 实现任何 SAR/RA 方程，完全复用 build_PipeSAR24_model 生成的模型块。
% 做法：只把模型输入 Vinp_ts/Vinn_ts 从正弦换成【直流线性斜坡】，逐点
% 仿真读回 sim_Dout，再由传输曲线算 INL/DNL。
%
% 与直方图法(Static_test.m)的区别：DC 扫描是确定性的，每个直流输入精确
% 给出一个码，无需海量样本，天然适配 24-bit。
%
% 前提：静态测试要求确定性，脚本会强制关闭所有随机噪声
%       （T=0、Vn_sig*=0、Comp_noise*=0、Amp_noise*=0）。
%
% 注入非理想（制造校准目标）：直接在 PipeSAR_params.m 里改 sigmaCu1/Av
% 等再运行本脚本即可；模型参数为 Parameter 作用域，仿真时从 workspace
% 读取，改电容失配无需重建模型（仅改位数 N1/N2/N3 才需重建）。
%
% 说明：驱动 Simulink 逐点仿真比纯 MATLAB 向量化慢，Npts=2^16 通常可接受；
% 若嫌慢可调小 Npts（INL 形状 2^16~2^18 已足够）。
% -------------------------------------------------------------------

clear; clc; close all;
mdl = 'PipeSAR24';

%% 1. 载入参数
PipeSAR_params;

% —— 静态测试强制关噪声（即使 params 里被打开也在此覆盖，保证确定性）——
T = 0;
Vn_sig_p1=0; Vn_sig_n1=0; Vn_sig_p2=0; Vn_sig_n2=0; Vn_sig_p3=0; Vn_sig_n3=0;
Comp_noise1 = 0; Comp_noise2 = 0; Comp_noise3 = 0;
Amp_noise1  = 0; Amp_noise2  = 0;

%% 2. 生成/加载模型（与 run_PipeSAR24_sim 相同的重建保护）
if ~exist([mdl '.slx'],'file')
    build_PipeSAR24_model(mdl);
else
    if ~bdIsLoaded(mdl), load_system(mdl); end
    want_cfg = sprintf('PipeSAR24 bitcfg: N1=%d N2=%d N3=%d', N1, N2, N3);
    if getSimulinkBlockHandle([mdl '/log_Vres2p']) == -1 || ...
       ~strcmp(get_param(mdl,'Description'), want_cfg)
        fprintf('模型缺块或位分配与当前参数不符，自动重建 %s.slx ...\n', mdl);
        build_PipeSAR24_model(mdl);
    end
end

%% 3. 扫描设置
Npts = 2^16;              % 全范围扫描点数（越大 INL/DNL 分辨越细；2^16 起步）
over = 1.02;              % 过驱动系数（略超满量程，确保首末码都被覆盖）
FS_diff = Vref1;          % 差分满量程（差分输入 Vd 的量程 ≈ ±Vref1）

Vd = linspace(-over*FS_diff, over*FS_diff, Npts).';   % 差分输入线性斜坡
Vp = Vcm + Vd/2;                                      % 正端
Vn = Vcm - Vd/2;                                      % 负端
dV = Vd(2) - Vd(1);                                   % 均匀步长

%% 4. 驱动模型跑 DC 扫描，取回输出码 sim_Dout
fprintf('DC 扫描 %d 点，驱动 %s.slx（无噪声）...\n', Npts, mdl);
Dout = run_sweep(mdl, Vp, Vn);        % [Npts x 1] 数字后端输出码

%% 5. 由传输曲线计算 INL / DNL
% 5.1 剔除过驱动导致的饱和平台（首末码卡在两端轨的点）
dmin = min(Dout); dmax = max(Dout);
usable = (Dout > dmin) & (Dout < dmax);   % 只保留未饱和的中间段
idx = find(usable);  i1 = idx(1);  i2 = idx(end);
Vd_u   = Vd(i1:i2);
Dout_u = Dout(i1:i2);

% 5.2 最小二乘拟合理想直线（自动扣除增益+失调误差）
p_fit  = polyfit(Vd_u, Dout_u, 1);   % p_fit(1)=斜率(码/伏), p_fit(2)=截距
Dline  = polyval(p_fit, Vd_u);       % 每点的理想码

% 5.3 INL = 实际码 - 理想直线（单位 LSB）
INL = Dout_u - Dline;

% 5.4 DNL = 局部斜率 / 理想斜率 - 1
codes_per_step = p_fit(1) * dV;      % 每个输入步长对应的理想码增量
dDout = diff(Dout_u);                % 实际每步前进的码数
DNL   = dDout / codes_per_step - 1;

% 5.5 指标 + 单位换算
DNLmax=max(DNL); DNLmin=min(DNL);
INLmax=max(INL); INLmin=min(INL);
inl_ppm = max(abs([INLmax INLmin])) / 2^N * 1e6;   % INL 折算成满量程 ppm

fprintf('---- DC 扫描静态测试 ----\n');
fprintf('扫描点数 Npts = %d，每步 ≈ %.1f 个码 (codes/step)\n', Npts, codes_per_step);
if codes_per_step > 1.0
    fprintf(['注意：每步 > 1 个码，DNL 为“每步跨越的若干码的平均值”，' ...
             '非逐码真值。\n      要逐码 DNL 请增大 Npts 或用第 7 节的局部放大扫描。\n']);
end
fprintf('DNL: max=%+.4f / min=%+.4f LSB\n', DNLmax, DNLmin);
fprintf('INL: max=%+.4f / min=%+.4f LSB  (≈ %.4g ppm of FS)\n', INLmax, INLmin, inl_ppm);

%% 6. 绘图（沿用 Static_test.m 风格：DNL 在上、INL 在下，粗线加粗 Arial）
xcode_INL = Dout_u;            % INL 横轴：输出码
xcode_DNL = Dout_u(1:end-1);   % DNL 横轴：比 INL 少一个点

figure;
% ---------- DNL（上）----------
subplot(2,1,1);
Q_DNL = plot(xcode_DNL, DNL, 'k');
set(gca,'linewidth',2);
set(gca,'FontWeight','bold','fontsize',15,'fontname','Arial');
set(Q_DNL,'linewidth',2);
title('DNL','FontWeight','bold','fontsize',20,'fontname','Arial');
xlabel('Digital Code [LSB]');  ylabel('DNL [LSB]');
grid on; box on;  xlim([0 2^N]);
lo = min(-1, floor(min(DNL)));  hi = max(1, ceil(max(DNL)));  ylim([lo hi]);
text(0.02,0.5,sprintf('DNLmax = %3.2f LSB\n\n\n\n\nDNLmin = %3.2f LSB',DNLmax,DNLmin),...
    'sc','FontWeight','bold','fontsize',15,'fontname','Arial');

% ---------- INL（下）----------
subplot(2,1,2);
Q_INL = plot(xcode_INL, INL, 'k');
set(gca,'linewidth',2);
set(gca,'FontWeight','bold','fontsize',15,'fontname','Arial');
set(Q_INL,'linewidth',2);
title('INL','FontWeight','bold','fontsize',20,'fontname','Arial');
xlabel('Digital Code [LSB]');  ylabel('INL [LSB]');
grid on; box on;  xlim([0 2^N]);
lo = floor(min(INL));  hi = ceil(max(INL));  if lo==hi, lo=lo-1; hi=hi+1; end
ylim([lo hi]);  set(gca,'xgrid','off');
set(gcf,'unit','centimeters','position',[10 5 18 14]);
text(0.02,0.5,sprintf('INLmax = %3.2f LSB\n\n\n\n\nINLmin = %3.2f LSB\n\n(INL %.3g ppm)',...
    INLmax,INLmin,inl_ppm),...
    'sc','FontWeight','bold','fontsize',15,'fontname','Arial');

%% 7.（可选）局部放大：在某个码附近做亚-LSB 精扫，得到逐码真值 DNL
% 24-bit 全范围逐码不现实；如需真逐码 DNL，锁定一小段（例如某个大进位
% 附近）用亚-LSB 步长精扫。同样是驱动模型，不重复方程。
do_zoom = false;
if do_zoom
    code_center = 2^16;          % 关注的码（默认对准第一级一个大进位边界）
    span_codes  = 512;           % 放大窗口宽度（码）
    sub_lsb     = 8;             % 每个码取几个亚-LSB采样点(>=2)
    LSB_v = FS_diff / 2^N;                       % 1 LSB 对应的差分电压
    Vc = (code_center/2^N - 0.5) * 2 * FS_diff;  % 该码对应的大致差分电压
    Vz = Vc + linspace(-span_codes/2, span_codes/2, span_codes*sub_lsb).'*LSB_v;
    Vpz = Vcm + Vz/2;  Vnz = Vcm - Vz/2;
    fprintf('局部放大：驱动模型精扫 %d 点 @code≈%d ...\n', numel(Vz), code_center);
    Dz  = run_sweep(mdl, Vpz, Vnz);
    pz  = polyfit(Vz, Dz, 1);
    cps = pz(1)*(Vz(2)-Vz(1));
    DNLz = diff(Dz)/cps - 1;
    INLz = Dz - polyval(pz, Vz);
    figure('unit','centimeters','position',[8 4 20 16]);
    subplot(2,1,1); plot(Dz(1:end-1), DNLz,'k','linewidth',1.5); grid on; box on;
    title(sprintf('DNL 局部放大（逐码真值）@code≈%d',code_center),'fontweight','bold');
    xlabel('code'); ylabel('DNL [LSB]');
    subplot(2,1,2); plot(Dz, INLz,'k','linewidth',1.5); grid on; box on;
    title('INL 局部放大','fontweight','bold');
    xlabel('code'); ylabel('INL [LSB]');
    fprintf('局部放大: DNL max=%+.4f/min=%+.4f LSB (每步 %.2f 码)\n',...
        max(DNLz),min(DNLz),cps);
end

% ===================================================================
%                            本地函数
% ===================================================================
function Dout = run_sweep(mdl, Vp, Vn)
% 用给定的正/负端直流序列作为输入，驱动模型仿真一次，返回输出码 sim_Dout。
% From Workspace 块从 base workspace 读取 Vinp_ts/Vinn_ts；StopTime 为
% 符号表达式 (num-1)/fs，故设好 num 即自动匹配点数。
fs = evalin('base','fs');
Np = numel(Vp);
tt = (0:Np-1).'/fs;
assignin('base','Vinp_ts', timeseries(Vp, tt));
assignin('base','Vinn_ts', timeseries(Vn, tt));
assignin('base','num',     Np);
out  = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
Dout = squeeze(out.get('sim_Dout'));   % 兼容 2-D / 3-D 记录格式
Dout = Dout(:);
end
