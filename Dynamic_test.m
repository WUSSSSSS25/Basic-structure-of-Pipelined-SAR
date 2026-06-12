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


[~, bin]        = max(dB_spectrum(1:floor(Nsample/2))); % 找到输入信号位置
signal_frequency= bin; 
signal_bin      = 1; % 将信号附近的旁瓣视作信号功率
harmonic_bin    = 1; % 将谐波附近的旁瓣视作谐波功率
start_power     = 5; % 忽略DC直流
signal_power    = sum(Power_spectrum(bin-signal_bin:bin+signal_bin)); % 输入信号功率，包括旁瓣
total_power     = sum(Power_spectrum(start_power:floor(Nsample/2))); % 总功率
Power_spectrum_half = Power_spectrum(start_power:floor(Nsample/2)); % 半边谱功率，奈奎斯特区间总功率


harmonic_indices = signal_frequency * (2:20); % 考虑到20次谐波

%%%%%%%%%%%%%%%%%%%% 找到谐波位置 %%%%%%%%%%%%%%%%%%%
for i = 1:length(harmonic_indices)
    if harmonic_indices(i) > Nsample / 2
        division_result = harmonic_indices(i) / (Nsample / 2);
        remainder_fs_2 = mod(harmonic_indices(i), Nsample / 2);

        if rem(division_result, 2) == 1 % 如果除以N/2是奇数，说明落在偶数区间
            if remainder_fs_2 == 0
                harmonic_indices(i) = Nsample / 2;
            else
                harmonic_indices(i) = Nsample / 2 - abs(remainder_fs_2);
            end
        else % 如果除以N/2是偶数，说明落在奇数区间
            harmonic_indices(i) = remainder_fs_2;
        end
    end
    harmonic_indices(i)=harmonic_indices(i);
    harmonic_power_single(i)  = sum(Power_spectrum_half(harmonic_indices(i)-harmonic_bin:harmonic_indices(i)+harmonic_bin));
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
    fin=bin*fs/(1e6*Nsample); % 输入信号实际频率
    figure; % 定义图片
    % hold on
    Figure=plot([0:Nsample/2-1].*fs_/Nsample,dB_spectrum(2:Nsample/2+1)-max_dBc,'k'); % 绘图
    mindB=min(dB_spectrum(2:Nsample/2+1)-max_dBc); % 标准化Y轴
    grid on;
    zoom;
    set(gca,'linewidth',3);
    set(gca,'fontsize',20,'FontWeight','bold','fontname','Arial');
    set(Figure,'linewidth',2.5);
    title(sprintf('fin = %3.2f MHz,  fs = %d MHz',fin,fs_),'FontWeight','bold','fontsize',20,'fontname','Arial');
    xlabel('Frequency (MHz)','FontWeight','bold','fontsize',20,'fontname','Arial');
    ylabel('Amplitude (dB)','FontWeight','bold','fontsize',20,'fontname','Arial');
    xlim([0 fs_/2]);ylim([-140 0]);

    % 添加动态参数文本显示
    text(fs_/5,-30, sprintf(' THD = %3.2f dB\n SFDR = %3.2f dB\n SNR = %3.2f dB\n SNDR = %3.2f dB\n ENOB = %3.2f bit',...
        THD,SFDR,SNR,SNDR,ENOB),...
        'LineWidth',2,'fontsize',20,'Margin',5,'FontWeight','bold','fontname','Arial');
    set(gcf, 'unit', 'centimeters', 'position', [10 5 18 14]);
    hold off;
end