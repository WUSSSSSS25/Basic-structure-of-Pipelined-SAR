function[DNLmax,DNLmin,INLmax,INLmin]=Static_test(Vin,N)

Dout=Vin;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% 静态特性测试 %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 计算DNL/INL
min_bin=min(Dout);
max_bin=max(Dout);
code_num=max_bin-min_bin;
h=hist(Dout, min_bin:max_bin);            % 直方图
ch = cumsum(h);                           % 累积直方图
Tlevels = -cos(pi*ch/sum(h));             % 升余弦拟合
hlin = Tlevels(2:end) - Tlevels(1:end-1); % 相邻码箱之间的差异
hlin = hlin(3:end-2);                     % 丢弃边缘码箱
lsb = sum(hlin) / (length(hlin));         % 查找箱之间的平均差异,以消除增益误差
dnl = [0 hlin/lsb-1];                     % 删除增益误差,中心处DNL为0
inl= cumsum(dnl);                         % INL是DNL的积分（求和）
DNLmax=max(dnl);
DNLmin=min(dnl); 
INLmax=max(inl);
INLmin=min(inl);
  
% 绘出DNL/INL绘
figure;
subplot(2,1,1) %绘出DNL
Q_DNL=plot(linspace(min_bin+2, max_bin-2, length(dnl)), dnl,'k');
set(gca,'linewidth',2);
set(gca,'FontWeight','bold','fontsize',15,'fontname','Arial');
set(Q_DNL,'linewidth',2);
title(sprintf('DNL'),'FontWeight','bold','fontsize',20,'fontname','Arial');
xlabel('Digital Code [LSB]');
ylabel('DNL [LSB]');
grid on;
box on;
xlim([0 2^N]);
ylim([-1 ceil(max(dnl))]);
text(0.02,0.5,sprintf('DNLmax = %3.2fLSB\n\n\n\n\nDNLmin = %3.2fLSB',DNLmax,DNLmin),...
    'sc','FontWeight','bold','fontsize',15,'fontname','Arial');
subplot(2,1,2) %绘出INL
Q_INL=plot(linspace(min_bin+2, max_bin-2, length(dnl)), inl,'k');
set(gca,'linewidth',2);
set(gca,'FontWeight','bold','fontsize',15,'fontname','Arial');
set(Q_INL,'linewidth',2);
title(sprintf('INL'),'FontWeight','bold','fontsize',20,'fontname','Arial');
xlabel('Digital Code [LSB]');
ylabel('INL [LSB]');
grid on;
box on;
xlim([0 2^N]);
ylim([floor(min(inl)) ceil(max(inl))]);
set(gca,'xgrid','off');
set(gcf, 'unit', 'centimeters', 'position', [10 5 18 14]);
text(0.02,0.5,sprintf('INLmax = %3.2fLSB\n\n\n\n\nINLmin = %3.2fLSB',INLmax,INLmin),...
    'sc','FontWeight','bold','fontsize',15,'fontname','Arial');
  
  
