function [signal] = SignalBuilder(SignalName, num, mean, variance)

    switch(SignalName)
        case 'G'
            signal = mean+randn(1,num)*variance;

        otherwise
            error('ÊäÈëĞÅºÅÃû´íÎó£¡£¡');
    end
    
end