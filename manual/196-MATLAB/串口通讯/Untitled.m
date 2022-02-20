
    % reference model
    Wm = 15; yita = 1.0; 
    ym = 0; ym_dot = 0;

    % Command input constants
    Sampling_period = 1/2000;                      % sampling period
    Simulation_total_time = 2*1.2;                 % simualtion time span
    Square_period= 1.2;                            % half period of the command input
    % Command input Initialization
    uc=0;                                    % initial values of the command input
    
    
    for i = 1 : Simulation_total_time/Sampling_period
        TIME(i)=(i-1)*Sampling_period;
        
        % calculate the control input Um(i) based on plant outputs and uc
        ym_ddot = Wm*Wm*(uc-ym) - 2*yita*Wm*ym_dot;
        ym_dot = Sampling_period*ym_ddot+ym_dot;
        ym = Sampling_period*ym_dot+ym;

        save_reference_command_input{i}=[ym uc]';
        
        % simulations with the step command case
        uc = 1*(mod(TIME(i), Square_period) < Square_period/2); 

    end
    
    figure(1)
    reference_command = cell2mat(save_reference_command_input)';

    plot(TIME, reference_command, '-'); hold on
    
    xlabel('time (sec)')
    ylabel('Reference transform')
    legend( 'ym');
    


