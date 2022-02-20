%{
通过中断，读取串口数据，显示传感器压力数据速度变化曲线和某传感器块滤波前后的压力数据曲线
%}

function myserial_Arduino_interupt


%% 如果matlab已经打开了串口，关闭它，防止占用
    if ~isempty(instrfind)
         fprintf('关闭已打开串口: \r\n');
         fclose(instrfind);
         delete(instrfind);
    end
    
    
    %% 中断设置
    global s h1_2 h1_1 tt h2_1 h2_2 h2_3 h2_4 h3_1 h3_2 base_t pre_angle pre_angle2
    maxSize = 300;
    subplot 221
    title('角度数据')
    h1_1  = animatedline('MaximumNumPoints',maxSize,'color','b');
    h1_2  = animatedline('MaximumNumPoints',maxSize,'color','r');
    legend('acc1','acc2')
    subplot 222
    title('角速度数据')
    h2_1 = animatedline('MaximumNumPoints',maxSize,'color','b');
    h2_2 = animatedline('MaximumNumPoints',maxSize,'color','r');
    h2_3 = animatedline('MaximumNumPoints',maxSize,'color','g');
    h2_4 = animatedline('MaximumNumPoints',maxSize,'color','y');
    legend('acc1','acc2')
    subplot 223
    title('助力电流')
    h3_1 = animatedline('MaximumNumPoints',maxSize,'color','b');
    h3_2 = animatedline('MaximumNumPoints',maxSize,'color','r');
    legend('acc1','acc2')

    s = serial('COM34');         %定义串口对象
    set(s,'BaudRate',115200);    %设置波特率,缺省9600bit/s 
    s.parity='none';            %设置校验位无奇偶校验，默认none
    s.stopbits=1;               %设置停止位，默认1
    s.Terminator='LF';          %设置输入缓冲区为256b，缺省值 
    
    % 设置中断响应模式（有“byte”和“Terminator”两种模式可选，“byte”是达到一定字节数产生中断，“Terminator”可用作键盘某个按键事件来产生中断）
    s.BytesAvailableFcnMode = 'terminator';

    % 打开串口，数据声明和初始化
    fopen(s);                  %打开串口对象s
    s.BytesAvailableFcn = @ReceiveCallback;
    fprintf('打开串口: %s\r\n',s.Port);
    
    %傅里叶变换，间隔tt
    tt = -1;
    base_t = 0;0
    
    
    pre_angle = 0;
    pre_angle2 = 0;
end


%% 中断函数
function ReceiveCallback( ~,~)
    global s h1_2 h1_1 tt h2_1 h2_2 h2_3 h2_4 h3_1 h3_2 base_t pre_angle pre_angle2
    
    b = fgetl(s);
%     b
    b = str2num(b);     %用函数fget(s)从缓冲区读取串口数据，当出现终止符（换行符）停止。
                        %所以在Arduino程序里要使用Serial.println()
                        %如果不是数字而是汉字，比如“这是6”，将返回[]

    if( size(b,2)>5 )        %检验是否是[]，因为每次scan前，都会输出汉字，参考arduino中程序
        if(tt == -1)
            base_t = b(1);
        end
        tt = b(1) - base_t;
        addpoints(h1_1,tt,b(2));
        drawnow limitrate;
        addpoints(h2_1,tt,b(3));
        drawnow limitrate;
        addpoints(h3_1,tt,b(6));
        drawnow limitrate;
        
        
        
        addpoints(h1_2,tt,b(4));
        drawnow limitrate;
        addpoints(h2_2,tt,b(5));
        drawnow limitrate;
        addpoints(h3_2,tt,b(7));
        drawnow limitrate;
        
%         addpoints(h2_3,tt,(b(2)-pre_angle)/0.01);
%         addpoints(h2_4,tt,(b(4)-pre_angle2)/0.01);
%         pre_angle = b(2);
%         pre_angle2 = b(4);
       % tt = tt+1;
    end
end

