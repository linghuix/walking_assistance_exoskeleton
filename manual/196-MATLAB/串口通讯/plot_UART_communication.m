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
    global s n linedata_last line
    maxSize = 300;
    n = 0;
    linedata_last = zeros(1,50);
    
    title('串口数据曲线，最大数量为10')
    h1_1  = animatedline('MaximumNumPoints',maxSize,'color','b');
    h1_2  = animatedline('MaximumNumPoints',maxSize,'color','r');
    h1_3  = animatedline('MaximumNumPoints',maxSize,'color','y');
    h1_4  = animatedline('MaximumNumPoints',maxSize,'color','k');
    h1_5  = animatedline('MaximumNumPoints',maxSize,'color','c');
    h1_6  = animatedline('MaximumNumPoints',maxSize,'color','g');
    h1_7  = animatedline('MaximumNumPoints',maxSize,'color','b','LineStyle','--');
    h1_8  = animatedline('MaximumNumPoints',maxSize,'color','r','LineStyle','--');
    h1_9  = animatedline('MaximumNumPoints',maxSize,'color','y','LineStyle','--');
    h1_10  = animatedline('MaximumNumPoints',maxSize,'color','k','LineStyle','--');
    line = [h1_1 h1_2 h1_3 h1_4 h1_5 h1_6 h1_7 h1_8 h1_9 h1_10]
    legend('show')

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
    
end


%% 中断函数
function ReceiveCallback( ~,~)
    global s n linedata_last line
    
    b = fgetl(s);
    b = str2num(b);     %用函数fget(s)从缓冲区读取串口数据，当出现终止符（换行符）停止。
                        %所以在Arduino程序里要使用Serial.println()
                        %如果不是数字而是汉字，比如“这是6”，将返回[]
    
    if( length(b) > 0 )   %检验是否是[]，因为每次scan前，都会输出汉字，参考arduino中程序
        for Num = 1:length(b)
            n = n + 1;
            b(Num) = 0.8*linedata_last(Num)+0.2*b(Num);
            linedata_last(Num) = b(Num);
            addpoints(line(Num),n,b(Num));
        end
    end
end

