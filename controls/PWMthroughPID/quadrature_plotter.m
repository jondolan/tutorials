function quadrature_plotter()

    % force the closeArduino function to be called, even on Control-C
    thecleanerupper = onCleanup(@closeArduino);
    
    % Start the Arduino serial port
    Arduino = serial('COM7', 'BaudRate', 115200);
    
    % go until control-c
    while 1
        % wait for the user to say go, so they can upload a new sketch and
        % we're not blocking
        tmp = input('Press enter to take a sample');
        
        % open the serial com line
        fopen(Arduino);
        
        % first 3 prints are "Kp: ?"...", Ki: ?"...and ", Kd: ?"
        % create that string
        thetitle = strcat(strtrim(fgetl(Arduino)), strtrim(fgetl(Arduino)), strtrim(fgetl(Arduino)));
        
        % open a figure with 2 subplots, label it, etc.
        figure('Name',thetitle);
        subplot(2,1,1);
        title(thetitle);
        xlabel("t (milliseconds)");
        ylabel("position");
        hold on;
        
        % get the next 3 lines, target position, total experiment time, and
        % time step (period of PID loop)
        target = str2num(fgetl(Arduino));
        time = str2num(fgetl(Arduino));
        period = str2num(fgetl(Arduino));
        
        % show the setpoint changing by displaying some time before t=0
        t_neg = -time/2:period:time;
        target_f = target .* (t_neg > 0); % logical masking to show the setpoint
        plot(t_neg, target_f); % plot the setpoint
        
        % reset t for the data collection, only from t=0 onwards matters
        t = 0:period:time;
        
        % create zero vectors for the position and output plots
        position = zeros(1, length(t));
        output = zeros(1, length(t));
        
        % indexer for position and output
        x = 1;
        line = fgetl(Arduino); % get the next line
        while isequal('end', strtrim(line)) == 0 % Arduino prints "end" when it's done
            
            % split the line by the comma in "output,position"
            line = split(strtrim(line), ',');
            
            line
            
            % place the values in the vector
            output(1,x) = str2num(line{1,1});
            position(1,x) = str2num(line{2,1});

            % get the next line, increment the indexer
            line = fgetl(Arduino);
            x = x + 1;
            
        end
        % be sure to clean up after ourselves
        fclose(Arduino);
        
        % plot the position and setup the legend
        plot(t, position);
        legend('setpoint', 'position', 'Location', 'west');
        
        % activate the second subplot for the PID output plot (PWM, 0-255)
        subplot(2,1,2);
        xlabel("t (milliseconds)");
        ylabel("PID output (PWM value)");
        plot(t_neg, [zeros(1, round((time/2)/period)) output]); % plot the output
        legend('PWM output', 'Location', 'west');
        
    end
    
    % cleans up the Arduino serial connection when the script is Ctrl-C'd
    function closeArduino()
        disp 'Cleaning up Arduino COM port connection...'
        fclose(Arduino);
    end
end