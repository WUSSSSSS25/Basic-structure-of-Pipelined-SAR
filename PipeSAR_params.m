%% PipeSAR_params.m
% 24-bit 三级 Pipelined-SAR ADC 参数初始化脚本（Simulink 模型用）
% 位分配: N1 + N2 + N3 - 2 = 8 + 8 + 10 - 2 = 24 bit（级间各 1-bit 冗余）
% 所有参数加载到 base workspace，Simulink 模型中的 MATLAB Function 块
% 以 "Parameter" 作用域直接引用同名变量。
%
% 模型结构与原 MATLAB 脚本 PipeSAR_test_3stage.m 完全一致：
%   Stage1(SAR_FUN_R) -> RA1(RA_FUN) -> Stage2(SAR_FUN_R) -> RA2 -> Stage3(SAR_FUN)
%   -> Digital_Backend(数字重组/校准接口)

% 注意：本脚本会被 run_PipeSAR24_sim / build_PipeSAR24_model 调用，
% 不要在此处 clear，否则会清掉调用方的变量。

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% ADC 基本参数 %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N1 = 8;              % 第一级分辨率 (bit)
N2 = 8;              % 第二级分辨率 (bit)
N3 = 10;             % 第三级分辨率 (bit)
N  = N1+N2+N3-2;     % 总分辨率 = 24 bit（两处 1-bit 冗余重叠）
fs  = 500e3;         % 采样率 (Hz)
Vcm = 0.9;           % 共模电压 (V)

%%%%%%%%%%%%%%%%% 第一级 ADC 参数 %%%%%%%%%%%%%%%%%%
Cu1 = 50e-15;                                   % 单位电容 (F)
sigmaCu1 = 0.0;  C_mismatch1 = sigmaCu1*Cu1;    % 电容失配系数
C_arr_p1 = [2.^[(N1-1):-1:0],1];                % CDAC_P 权重
C_arr_n1 = [2.^[(N1-1):-1:0],1];                % CDAC_N 权重
C_dev_p1 = C_mismatch1*sqrt(C_arr_p1).*randn(1,N1+1);
C_dev_n1 = C_mismatch1*sqrt(C_arr_n1).*randn(1,N1+1);
C_act_p1 = C_arr_p1*Cu1 + C_dev_p1;             % 含失配的实际电容
C_act_n1 = C_arr_n1*Cu1 + C_dev_n1;
Cp_p1 = 0e-15;  Cp_n1 = 0e-15;                  % 寄生电容 (F)
C_tot_p1 = sum(C_act_p1)+Cp_p1;
C_tot_n1 = sum(C_act_n1)+Cp_n1;
Vref1 = 1.8;                                    % 参考电压 (V)

%%%%%%%%%%%%%%%%% 第二级 ADC 参数 %%%%%%%%%%%%%%%%%%
Cu2 = 2e-15;
sigmaCu2 = 0.0;  C_mismatch2 = sigmaCu2*Cu2;
C_arr_p2 = [2.^[(N2-1):-1:0],1];
C_arr_n2 = [2.^[(N2-1):-1:0],1];
C_dev_p2 = C_mismatch2*sqrt(C_arr_p2).*randn(1,N2+1);
C_dev_n2 = C_mismatch2*sqrt(C_arr_n2).*randn(1,N2+1);
C_act_p2 = C_arr_p2*Cu2 + C_dev_p2;
C_act_n2 = C_arr_n2*Cu2 + C_dev_n2;
Cp_p2 = 0e-15;  Cp_n2 = 0e-15;
C_tot_p2 = sum(C_act_p2)+Cp_p2;
C_tot_n2 = sum(C_act_n2)+Cp_n2;
Vref2 = 1.8;

%%%%%%%%%%%%%%%%% 第三级 ADC 参数 %%%%%%%%%%%%%%%%%%
% 注意：第三级沿用 SAR_FUN 的电容阵列结构 [2^(N3-2)...1, 1]，最后一位
% 仅比较不开关。
Cu3 = 1e-15;
sigmaCu3 = 0.0;  C_mismatch3 = sigmaCu3*Cu3;
C_arr_p3 = [2.^[(N3-2):-1:0],1];
C_arr_n3 = [2.^[(N3-2):-1:0],1];
C_dev_p3 = C_mismatch3*sqrt(C_arr_p3).*randn(1,N3);
C_dev_n3 = C_mismatch3*sqrt(C_arr_n3).*randn(1,N3);
C_act_p3 = C_arr_p3*Cu3 + C_dev_p3;
C_act_n3 = C_arr_n3*Cu3 + C_dev_n3;
Cp_p3 = 0e-15;  Cp_n3 = 0e-15;
C_tot_p3 = sum(C_act_p3)+Cp_p3;
C_tot_n3 = sum(C_act_n3)+Cp_n3;
Vref3 = 1.8;

%%%%%%%%%%%%%%%%%%%% 环境参数 %%%%%%%%%%%%%%%%%%%%%%
k = 1.38e-23;        % 玻尔兹曼常数
Jitter = 0e-15;      % 时钟抖动 (s)
T = 0;               % 温度 (K)，0 即关闭 kT/C 噪声

% 预计算各级采样 kT/C 噪声标准差（传入 Simulink 的参数）
Vn_sig_p1 = sqrt(k*T/C_tot_p1);  Vn_sig_n1 = sqrt(k*T/C_tot_n1);
Vn_sig_p2 = sqrt(k*T/C_tot_p2);  Vn_sig_n2 = sqrt(k*T/C_tot_n2);
Vn_sig_p3 = sqrt(k*T/C_tot_p3);  Vn_sig_n3 = sqrt(k*T/C_tot_n3);

%%%%%%%%%%%%%%%%%%%% 放大器参数 %%%%%%%%%%%%%%%%%%%%
% 提示：24-bit 下级间增益 G1=2^(N1-1)=128，反馈系数 beta=2/2^N1，
% 闭环增益误差约 1/(beta*Av)。Av=120dB 时误差约 1.3e-4，足以把
% ENOB 限制在 ~13 bit —— 这正是留给数字校准算法去修正的非理想因素。
Av  = 120;           % 开环增益 (dB)
GBW = 1e12;          % 增益带宽积 (Hz)
Amp_offset = 0e-3;   % 失调电压 (V)
Amp_noise  = 0e-3;   % 输入参考噪声 RMS (V)

% 级间增益设置（与原 RA_FUN 调用一致）
Cs1 = 2*Cu1*Vref1/Vref2;   Ch1 = 2^N1*Cu1 + Cp_n1;   % G1 ≈ Ch1/Cs1 = 2^(N1-1)
Cs2 = 2*Cu2*Vref2/Vref3;   Ch2 = 2^N2*Cu2 + Cp_p2;   % G2 ≈ 2^(N2-1)

%%%%%%%%%%%%%%%%%%%% 比较器参数 %%%%%%%%%%%%%%%%%%%%
Comp_offset1 = 0e-3;
Comp_offset2 = 0e-3;
Comp_offset3 = 0e-3;
Comp_noise   = 0e-3;

%%%%%%%%%%%%%%%%%%%% 输入信号 %%%%%%%%%%%%%%%%%%%%%%
num = 2^15;                  % 采样点数（FFT 点数）
Vfs = 1.8;                   % 差分输入摆幅 (V)
fin = 3877/num*fs;           % 相干采样频率（3877 为素数 bin）
t_jit = (0:num-1)'/fs + Jitter*randn(num,1);   % 含抖动的采样时刻
Vin_p = Vcm + (Vfs/2)*sin(2*pi*fin*t_jit);
Vin_n = Vcm - (Vfs/2)*sin(2*pi*fin*t_jit);

tt = (0:num-1)'/fs;          % Simulink 时间轴（均匀）
Vinp_ts = timeseries(Vin_p, tt);
Vinn_ts = timeseries(Vin_n, tt);

%%%%%%%%%%%%%%% 数字后端：理想重组权重 %%%%%%%%%%%%%
% ===== 数字校准接口 =====
% 默认权重为理想二进制权重（对应原脚本的 DOUT1/DOUT2/DOUT3 重组公式）。
% 最简单的校准接入方式：用你的算法估计出实际权重后改写 W1/W2/W3/OFS，
% 重新运行仿真或离线重组即可。
W1  = 2.^[(N-1):-1:(N-N1)];          % 第一级位权: 2^23 ... 2^16
W2  = 2.^[(N-N1):-1:(N-N1-N2+1)];    % 第二级位权: 2^16 ... 2^9 （与W1重叠1bit冗余）
W3  = 2.^[(N-N1-N2+1):-1:0];         % 第三级位权: 2^9  ... 2^0 （与W2重叠1bit冗余）
OFS = 2^(N-N1-1) + 2^(N-N1-N2);      % 失调修正项（与原脚本一致）
Kv  = Vref1/2^N;                     % 数字码 -> 电压换算系数

disp('PipeSAR_params: 24-bit 三级 Pipelined-SAR 参数已加载到 base workspace。');
