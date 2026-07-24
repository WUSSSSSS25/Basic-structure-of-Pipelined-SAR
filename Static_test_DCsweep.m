%% Static_test_DCsweep.m
% ===================================================================
%  DC 扫描法静态测试 (DNL / INL)，适用于高分辨率(24-bit)流水线 SAR
% ===================================================================
% 与直方图法(Static_test.m)不同：本脚本主动逐点扫描直流输入，直接测出
% 传输曲线 code = f(Vin)，再由此算 INL / DNL。无需海量样本，天然适配
% 24-bit。行为方程与 build_PipeSAR24_model.m 生成的模型块完全一致
% （若你改了模型里的 SAR/RA 公式，请同步本文件末尾的本地函数）。
%
% 前提：静态测试要求确定性——务必关掉所有随机噪声(默认已满足)：
%   T=0（kT/C 噪声关）、Comp_noise*=0、Amp_noise*=0。
%
% 用法：
%   1) 在 PipeSAR_params.m 里注入你要考察的非理想（如 sigmaCu1=1e-3、
%      Av=80），或直接改本脚本“注入”小节；
%   2) 运行 Static_test_DCsweep。
% -------------------------------------------------------------------

clear; clc; close all;

%% 1. 载入参数
PipeSAR_params;

% —— 静态测试强制关噪声（即使 params 里被打开也在此覆盖，保证确定性）——
Comp_noise1 = 0; Comp_noise2 = 0; Comp_noise3 = 0;
Amp_noise1  = 0; Amp_noise2  = 0;
Vn_sig_p1=0; Vn_sig_n1=0; Vn_sig_p2=0; Vn_sig_n2=0; Vn_sig_p3=0; Vn_sig_n3=0;

%% 2.（可选）在此注入要考察的非理想因素，制造校准目标
% 说明：改电容失配/增益后必须重算 C_act 与 Cs/Ch。最省事的做法是直接在
% PipeSAR_params.m 里改 sigmaCu1 / Av 再运行本脚本；这里留一个示例开关。
inject_demo = false;
if inject_demo
    Av = 80;                                   % 故意压低级间增益 -> 增益误差
    sigmaCu1 = 1e-3; C_mismatch1 = sigmaCu1*Cu1;
    rng(mismatch_seed);
    C_dev_p1 = C_mismatch1*sqrt(C_arr_p1).*randn(1,N1+1);
    C_dev_n1 = C_mismatch1*sqrt(C_arr_n1).*randn(1,N1+1);
    C_act_p1 = C_arr_p1*Cu1 + C_dev_p1;  C_act_n1 = C_arr_n1*Cu1 + C_dev_n1;
    C_tot_p1 = sum(C_act_p1)+Cp_p1;      C_tot_n1 = sum(C_act_n1)+Cp_n1;
end

%% 3. 扫描设置
Npts = 2^16;              % 全范围扫描点数（越大 INL/DNL 分辨越细；2^16 起步）
over = 1.02;              % 过驱动系数（略超满量程，确保首末码都被覆盖）
FS_diff = Vref1;          % 差分满量程（差分输入 Vd 的量程 ≈ ±Vref1）

Vd = linspace(-over*FS_diff, over*FS_diff, Npts).';   % 差分输入扫描 (列向量)
Vp = Vcm + Vd/2;                                      % 正端
Vn = Vcm - Vd/2;                                      % 负端
dV = Vd(2) - Vd(1);                                   % 均匀步长

%% 4. 把参数打包，逐点跑行为级流水线（向量化，一次算完所有扫描点）
P.N1=N1; P.N2=N2; P.N3=N3;
P.Vref1=Vref1; P.Vref2=Vref2; P.Vref3=Vref3; P.Vcm=Vcm; P.fs=fs; P.Av=Av; P.GBW=GBW;
P.C_act_p1=C_act_p1; P.C_act_n1=C_act_n1; P.C_tot_p1=C_tot_p1; P.C_tot_n1=C_tot_n1;
P.C_act_p2=C_act_p2; P.C_act_n2=C_act_n2; P.C_tot_p2=C_tot_p2; P.C_tot_n2=C_tot_n2;
P.C_act_p3=C_act_p3; P.C_act_n3=C_act_n3; P.C_tot_p3=C_tot_p3; P.C_tot_n3=C_tot_n3;
P.Cs1=Cs1; P.Ch1=Ch1; P.Cs2=Cs2; P.Ch2=Ch2;
P.Coff1=Comp_offset1; P.Coff2=Comp_offset2; P.Coff3=Comp_offset3;
P.Aoff1=Amp_offset1; P.Aoff2=Amp_offset2;
P.W1=W1; P.W2=W2; P.W3=W3; P.OFS=OFS;

Dout = pipe_adc(Vp, Vn, P);        % [Npts x 1] 输出码

%% 5. 由传输曲线计算 INL / DNL
% 5.1 剔除过驱动导致的饱和平台（首末码卡在两端轨的点）
dmin = min(Dout); dmax = max(Dout);
usable = (Dout > dmin) & (Dout < dmax);   % 只保留未饱和的中间段
% 找到连续可用区间的首尾，避免边缘个别抖动
idx = find(usable);
i1 = idx(1); i2 = idx(end);
Vd_u   = Vd(i1:i2);
Dout_u = Dout(i1:i2);

% 5.2 最小二乘拟合理想直线（自动扣除增益+失调误差）
p_fit  = polyfit(Vd_u, Dout_u, 1);   % p_fit(1)=斜率(码/伏), p_fit(2)=截距
Dline  = polyval(p_fit, Vd_u);       % 每点的理想码

% 5.3 INL = 实际码 - 理想直线（单位 LSB）
INL = Dout_u - Dline;

% 5.4 DNL = 局部斜率 / 理想斜率 - 1
%     理想情况下每步应前进 codes_per_step 个码
codes_per_step = p_fit(1) * dV;      % 每个输入步长对应的理想码增量
dDout = diff(Dout_u);                % 实际每步前进的码数
DNL   = dDout / codes_per_step - 1;  % [length(Dout_u)-1 x 1]

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

%% 6. 绘图
codeAxis = Dout_u;   % 横轴用输出码
figure('unit','centimeters','position',[8 4 20 16]);
subplot(2,1,1);
plot(codeAxis, INL, 'k', 'linewidth', 1.5); grid on; box on;
title('INL (DC-sweep)','fontweight','bold','fontsize',16);
xlabel('Output code [LSB]'); ylabel('INL [LSB]');
set(gca,'fontsize',12,'fontweight','bold'); xlim([dmin dmax]);
text(0.02,0.85,sprintf('INLmax=%.3f LSB\nINLmin=%.3f LSB\n(%.3g ppm)',INLmax,INLmin,inl_ppm),...
    'sc','fontweight','bold','fontsize',11,'BackgroundColor','w','EdgeColor','k','Margin',3);

subplot(2,1,2);
plot(codeAxis(1:end-1), DNL, 'k', 'linewidth', 1.5); grid on; box on;
title('DNL (DC-sweep)','fontweight','bold','fontsize',16);
xlabel('Output code [LSB]'); ylabel('DNL [LSB]');
set(gca,'fontsize',12,'fontweight','bold'); xlim([dmin dmax]);
text(0.02,0.85,sprintf('DNLmax=%.3f LSB\nDNLmin=%.3f LSB',DNLmax,DNLmin),...
    'sc','fontweight','bold','fontsize',11,'BackgroundColor','w','EdgeColor','k','Margin',3);

%% 7.（可选）局部放大：在某个码附近做亚-LSB 精扫，得到逐码真值 DNL
% 24-bit 全范围逐码不现实；如需真逐码 DNL，锁定一小段（例如某个大进位
% 附近）用亚-LSB 步长精扫。把 do_zoom 设 true 并指定中心/跨度。
do_zoom = false;
if do_zoom
    code_center = 2^(N-1);        % 关注的码（默认中点大进位处）
    span_codes  = 4096;           % 放大窗口宽度（码）
    sub_lsb     = 4;              % 每个码取几个亚-LSB采样点(>=2)
    LSB_v = FS_diff / 2^N;                       % 1 LSB 对应的差分电压
    Vc = (code_center/2^N - 0.5) * 2 * FS_diff;  % 该码对应的大致差分电压
    Vz = (Vc + linspace(-span_codes/2, span_codes/2, span_codes*sub_lsb).'*LSB_v);
    Vpz = Vcm + Vz/2; Vnz = Vcm - Vz/2;
    Dz  = pipe_adc(Vpz, Vnz, P);
    pz  = polyfit(Vz, Dz, 1);
    cps = pz(1)*(Vz(2)-Vz(1));
    DNLz = diff(Dz)/cps - 1;
    INLz = Dz - polyval(pz, Vz);
    figure('unit','centimeters','position',[8 4 20 16]);
    subplot(2,1,1); plot(Dz, INLz,'k','linewidth',1.5); grid on; box on;
    title(sprintf('INL 局部放大 @code≈%d',code_center),'fontweight','bold');
    xlabel('code'); ylabel('INL [LSB]');
    subplot(2,1,2); plot(Dz(1:end-1), DNLz,'k','linewidth',1.5); grid on; box on;
    title('DNL 局部放大（逐码真值）','fontweight','bold');
    xlabel('code'); ylabel('DNL [LSB]');
    fprintf('局部放大: DNL max=%+.4f/min=%+.4f LSB (每步 %.2f 码)\n',...
        max(DNLz),min(DNLz),cps);
end

% ===================================================================
%                        本地函数（行为级流水线）
%     与 build_PipeSAR24_model.m 的 sarScriptR/sarScriptLast/raScript
%     完全对应，但向量化以一次处理所有扫描点（噪声项已置 0）。
% ===================================================================
function Dout = pipe_adc(Vp, Vn, P)
% 完整三级流水线：Stage1 -> RA1 -> Stage2 -> RA2 -> Stage3 -> 数字重组
[D1, Vrp1, Vrn1] = sar_redundant(Vp, Vn, P.Vref1, P.C_act_p1, P.C_act_n1, ...
                                 P.C_tot_p1, P.C_tot_n1, P.Coff1, P.N1);
[Vip2, Vin2]     = res_amp(Vrp1, Vrn1, P.Av, P.GBW, P.Cs1, P.Ch1, P.Vcm, P.Aoff1, P.fs);
[D2, Vrp2, Vrn2] = sar_redundant(Vip2, Vin2, P.Vref2, P.C_act_p2, P.C_act_n2, ...
                                 P.C_tot_p2, P.C_tot_n2, P.Coff2, P.N2);
[Vip3, Vin3]     = res_amp(Vrp2, Vrn2, P.Av, P.GBW, P.Cs2, P.Ch2, P.Vcm, P.Aoff2, P.fs);
D3               = sar_last(Vip3, Vin3, P.Vref3, P.C_act_p3, P.C_act_n3, ...
                            P.C_tot_p3, P.C_tot_n3, P.Coff3, P.N3);
% 数字重组（默认理想权重；若做权重校准，把 P.W* 换成估计值即可）
Dout = D1*P.W1(:) + D2*P.W2(:) + D3*P.W3(:) - P.OFS;
end

function [D, Vrp, Vrn] = sar_redundant(Vp, Vn, Vref, Cap_p, Cap_n, Ctot_p, Ctot_n, Coff, Nb)
% 带冗余 SAR（对应 SAR_FUN_R）：Nb 位全部开关，向量化处理全部扫描点
Np = numel(Vp);
D  = zeros(Np, Nb);
for i = 1:Nb
    dec = (Vp - Vn) > Coff;                 % 判决：Vp-Vn>失调 => 码=1
    D(:,i) = dec;
    sp = Vref/2 * Cap_p(i) / Ctot_p;        % 该位 DAC 步进(正端)
    sn = Vref/2 * Cap_n(i) / Ctot_n;        % 该位 DAC 步进(负端)
    s  = 1 - 2*dec;                          % 码0=>+1(反馈+), 码1=>-1(反馈-)
    Vp = Vp + s.*sp;
    Vn = Vn - s.*sn;
end
Vrp = Vp;  Vrn = Vn;                          % 余差电压
end

function D = sar_last(Vp, Vn, Vref, Cap_p, Cap_n, Ctot_p, Ctot_n, Coff, Nb)
% 末级 SAR（对应 SAR_FUN）：前 Nb-1 位开关，最后一位仅比较不开关
Np = numel(Vp);
D  = zeros(Np, Nb);
for i = 1:Nb-1
    dec = (Vp - Vn) > Coff;
    D(:,i) = dec;
    sp = Vref/2 * Cap_p(i) / Ctot_p;
    sn = Vref/2 * Cap_n(i) / Ctot_n;
    s  = 1 - 2*dec;
    Vp = Vp + s.*sp;
    Vn = Vn - s.*sn;
end
D(:,Nb) = (Vp - Vn) > Coff;                  % 末位只比较
end

function [Vop, Von] = res_amp(Vp, Vn, Av, GBW, Cs, Ch, Vcm, Aoff, fs)
% 级间余差放大器（对应 RA_FUN）：有限增益 + 有限带宽建立误差 + 失调
Av_abs = 10^(Av/20);                         % 开环增益绝对值
beta   = Cs / Ch;                            % 反馈系数，理想闭环增益 1/beta
tamp   = 0.4 / fs;                           % 放大建立时间
toi    = 1 / (2*pi*beta*GBW);                % 闭环时间常数
g = (Av_abs/(1+beta*Av_abs)) * (1 - exp(-tamp/toi));  % 实际(含误差)增益
Vop = Vcm + g*(Vp - Vcm + Aoff);             % 失调只加正端(与 RA_FUN 一致)
Von = Vcm + g*(Vn - Vcm);
end
