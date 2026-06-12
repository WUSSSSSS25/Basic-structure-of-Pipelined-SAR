function[THD,SFDR,SNR,SNDR,ENOB]=Dynamic_test(Vin,fs,Nsample,En_plot,wid)

Dout=Vin;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% 是否加窗 %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if  wid == 0                           
    Dout = Dout - mean(Dout);
elseif  wid==1
    Dout = Dout - mean(Dout);
    Dout = Dout'.*window(@hamming,Nsample);
    Dout = Dout - mean(Dout); % 加汉明窗，并滤去直流
elseif  wid==2
    Dout = Dout - mean(Dout);
    Dout = Dout'.*window(@hann,Nsample);
    Dout = Dout - mean(Dout); % 加汉宁窗，并滤去直流
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 动态特性测试 %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Amp_spectrum    = abs(fft(Dout',Nsample)); % 幅度谱
Power_spectrum  = Amp_spectrum.^2; % 功率谱
dB_spectrum     = 10*log10(Amp_spectrum.^2/(Nsample/2)); % dB谱
max_dBc         = max(dB_spectrum); % 输入信号功率 (dB)


[~, bin]        = max(dB_spectrum(1:floor(Nsample/2))); % 找到输入信号位置（MATLAB下标）
signal_bin      = 1; % 将信号附近的旁瓣视作信号功率
harmonic_bin    = 1; % 将谐波附近的旁瓣视作谐波功率
start_power     = 5; % 忽略DC直流
signal_power    = sum(Power_spectrum(bin-signal_bin:bin+signal_bin)); % 输入信号功率，包括旁瓣
total_power     = sum(Power_spectrum(start_power:floor(Nsample/2))); % 总功率

%%%%%%%%%%%%%%%%%%%% 找到谐波位置 %%%%%%%%%%%%%%%%%%%
% 信号位于 MATLAB 下标 bin，对应 0 基频率格点 f0 = bin-1；
% h 次谐波的频率格点为 h*f0：先对 Nsample 取模，超过 Nsample/2 时镜像
% 折回第一奈奎斯特区，最后 +1 转回 MATLAB 下标，直接索引全谱
% Power_spectrum（修正了旧版按 h*bin 定位偏 h-1 个 bin、又用全谱坐标
% 索引被 start_power 截短的半谱再偏 4 个 bin 的问题）。
f0 = bin - 1;
harmonic_order = 2:20; % 考虑到20次谐波
harmonic_power_single = zeros(1, length(harmonic_order));
for i = 1:length(harmonic_order)
    kh = mod(harmonic_order(i)*f0, Nsample);   % 折叠到 0 ~ Nsample-1
    if kh > Nsample/2
        kh = Nsample - kh;                     % 镜像折回第一奈奎斯特区
    end
    idx = kh + 1;                              % 0 基频率格点 -> MATLAB 下标
    lo = max(idx - harmonic_bin, start_power); % 防越界，且不计入直流附近
    hi = min(idx + harmonic_bin, floor(Nsample/2));
    if lo <= hi
        harmonic_power_single(i) = sum(Power_spectrum(lo:hi));
    end
end

harmonic_power  = sum(harmonic_power_single); % 计算总的谐波功率

%%%%%%%%%%%%%%%%%%%% 计算各项指标 %%%%%%%%%%%%%%%%%%%
THD             = 10*log10(harmonic_power/signal_power);
SNDR            = 10*log10(signal_power/(total_power-signal_power));
SNR             = 10*log10(signal_power/(total_power-signal_power-harmonic_power));
ENOB            = (SNDR-1.76)/6.02;
Dout_SFDR       = abs(dB_spectrum - dB_spectrum(bin));
Dout_SFDR       = Dout_SFDR(1:Nsample/2);
Dout_SFDR_1     = min(Dout_SFDR(start_power:(bin-signal_bin-1)));
Dout_SFDR_2     = min(Dout_SFDR((bin+signal_bin+1):Nsample/2));
SFDR            = min(Dout_SFDR_1,Dout_SFDR_2);

%%%%%%%%%%%%%%%%%%%% 绘出指标图形 %%%%%%%%%%%%%%%%%%%

if(En_plot==1) % 判断是否绘图
    fs_=fs/1e6; % 将X轴设置为MHz单位
    fin=(bin-1)*fs/(1e6*Nsample); % 输入信号实际频率（bin 为 MATLAB 下标，频率格点为 bin-1）
    ydB = dB_spectrum(2:Nsample/2+1) - max_dBc;    % 归一化谱 (dBc)
    % Y 轴下限随实际噪底自适应：高分辨率下单点噪底约为
    % -(SNDR + 10*log10(Nsample/2)) dBc，固定 -140 会把整条噪底裁出图外
    mindB = min(ydB(isfinite(ydB)));               % 忽略幅度为 0 的 -Inf 频点
    if isempty(mindB) || ~isfinite(mindB), mindB = -200; end
    floor_dB = max(10*floor((mindB-10)/10), -250);
    ydB(~isfinite(ydB)) = floor_dB;                % -Inf 频点钳位到下限，保证连线可绘
    figure; % 定义图片
    % hold on
    Figure=plot([0:Nsample/2-1].*fs_/Nsample, ydB,'k'); % 绘图
    grid on;
    zoom;
    set(gca,'linewidth',3);
    set(gca,'fontsize',20,'FontWeight','bold','fontname','Arial');
    set(Figure,'linewidth',2.5);
    title(sprintf('fin = %3.3f MHz,  fs = %.3g MHz',fin,fs_),'FontWeight','bold','fontsize',20,'fontname','Arial');
    xlabel('Frequency (MHz)','FontWeight','bold','fontsize',20,'fontname','Arial');
    ylabel('Amplitude (dB)','FontWeight','bold','fontsize',20,'fontname','Arial');
    xlim([0 fs_/2]);ylim([floor_dB 0]);

    % 添加动态参数文本显示
    text(fs_/5,-30, sprintf(' THD = %3.2f dB\n SFDR = %3.2f dB\n SNR = %3.2f dB\n SNDR = %3.2f dB\n ENOB = %3.2f bit',...
        THD,SFDR,SNR,SNDR,ENOB),...
        'LineWidth',2,'fontsize',20,'Margin',5,'FontWeight','bold','fontname','Arial');
    set(gcf, 'unit', 'centimeters', 'position', [10 5 18 14]);
    hold off;
end