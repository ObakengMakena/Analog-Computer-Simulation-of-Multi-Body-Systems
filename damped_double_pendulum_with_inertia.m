tic;
    % Parameters with inertia
    m1 = 2.0; m2 = 1.0; % masses
    l1 = 1.0; l2 = 1.0; % lengths
    g = 9.81; % gravity
    b1 = 6; 
    b2 = 6; % damping coefficients
    I1 = 1/3 * m1 * l1^2; % moment of inertia for the first pendulum
    I2 = 1/3 * m2 * l2^2; % moment of inertia for the second pendulum

    % Initial conditions
    init_conditions = [pi/9, 0, pi/(180/21), 0];

    % Time span
    tspan = 0:0.001:30;

    % ODE solver
    options = odeset('AbsTol',1e-9,'RelTol',1e-9);
    [t, y] = ode45(@(t, y) equations_of_motion(t, y, m1, m2, l1, l2, g, b1, b2, I1, I2), tspan, init_conditions, options);
     
    
    theta_degrees = rad2deg(y(:, 1));

    theta_degrees2 = rad2deg(y(:, 3));

% Stop the timer and display the elapsed time
matlabSimTime = toc;
fprintf('Computation Time for MATLAB Code Simulation: %f seconds\n', matlabSimTime);

    % Plotting
    figure;
    plot(t, theta_degrees, 'r-', 'LineWidth', 2);
    hold on;
    plot(t, theta_degrees2, 'b-', 'LineWidth', 2);
    % plot(t, simout, 'r-', 'LineWidth', 2);
    % plot(t, simout1, 'y-', 'LineWidth', 2);
    plot(t, simout3, 'g- ','LineWidth', 3);
    plot(t, simout4, 'k-', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Angle (degrees)');
    title('Damped Double Pendulum Angles ');
    legend('\theta_1', ' \theta_2', 'real \theta1', 'real \theta2');
   
    
    grid on;
    hold off;


function dy = equations_of_motion(t, y, m1, m2, l1, l2, g, b1, b2, I1, I2)
    theta1 = y(1);
    dot_theta1 = y(2);
    theta2 = y(3);
    dot_theta2 = y(4);

    % Modified equations with inertia
    ddot_theta1 = (-g * (2*m1 + m2) * sin(theta1) - m2 * g * sin(theta1 - 2*theta2) - 2*sin(theta1 - theta2) * m2 * (dot_theta2^2 * l2 + dot_theta1^2 * l1 * cos(theta1 - theta2)) - b1 * dot_theta1) / (l1 * (2*m1 + m2 - m2 * cos(2*theta1 - 2*theta2)) );
    ddot_theta2 = (2 * sin(theta1 - theta2) * (dot_theta1^2 * l1 * (m1 + m2) + g * (m1 + m2) * cos(theta1) + dot_theta2^2 * l2 * m2 * cos(theta1 - theta2) - b2 * dot_theta2)) / (l2 * (2*m1 + m2 - m2 * cos(2*theta1 - 2*theta2))) ;
    % ddot_theta1 = (-g * l1 * (1/2*m1 + m2) * sin(theta1)  - sin(theta1 - theta2) * 1/2* m2 *l1*l2* dot_theta2^2  - ddot_theta2  * m2*l1*l2*1/2 * cos(theta1 - theta2) - b1 * dot_theta1) /(1/3 * m1* l1^2 +m2*l1^2) ;
    % ddot_theta2 = ( sin(theta1 - theta2) *dot_theta1^2 * 1/2* m2 *l1*l2 -  ddot_theta1*  1/2* m2 *l1*l2* cos(theta1 - theta2) -g * l2 * (1/2*m2) * sin(theta2) - b2 * dot_theta2) / (1/3 *m2*l2^2);

    dy = [dot_theta1; ddot_theta1 ; dot_theta2; ddot_theta2];
end


% 
% open_system("real_double_pendulum");
% 
% 
% slewrate = [0.001e6 0.002e6 0.005e6 0.007e6 0.009e6 0.01e6 0.02e6 0.05e6 0.07e6 0.09e6 0.1e6 0.2e6 0.5e6 0.7e6 0.9e6 1e6 2e6 5e6 7e6 9e6 10e6 20e6];
% % Preallocate storage for results
% Abs_error = cell(1, length(slewrate));
% mae_error = cell(1, length(slewrate));
% Mean_abs = cell(1, length(slewrate));
% corr2 = cell(1, length(slewrate));
% MAPE = cell(1, length(slewrate));  % Add storage for MAPE
% Max_PHASE = cell(1, length(slewrate));
% Min_PHASE = cell(1, length(slewrate));
% Avg_PHASE = cell(1, length(slewrate)); 
% 
% 
% Abs_error1 = cell(1, length(slewrate));
% mae_error1 = cell(1, length(slewrate));
% Mean_abs1 = cell(1, length(slewrate));
% corr21 = cell(1, length(slewrate));
% MAPE1 = cell(1, length(slewrate));  % Add storage for MAPE
% Max_PHASE1 = cell(1, length(slewrate));
% Min_PHASE1 = cell(1, length(slewrate));
% Avg_PHASE1 = cell(1, length(slewrate)); 
% 
% 
%     for k = 1:length(slewrate)
% 
% 
%         set_param([bdroot, '/Band-Limited Op-Amp1'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp2'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp3'], 'Vdotmax', num2str(slewrate(k)));        
%         set_param([bdroot, '/Band-Limited Op-Amp4'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp5'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp7'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp6'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp8'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp9'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp10'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp11'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp12'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp13'], 'Vdotmax', num2str(slewrate(k)));
% 
% 
%         simOut{k} = sim(gcs);
%         disp(['Completed ', num2str(k), ' of ', num2str(length(slewrate)), ' simulations']);
%         signalOut{k} = simout3;
%         signalOut1{k} = simout4;
% 
%   % Error and correlation calculations
%         squared_error = (theta_degrees - simout3).^2;
%         rmse = sqrt(mean(squared_error));
%         absolute_error = abs(theta_degrees - simout3);
%         mae = max(absolute_error);
%         correlation_coefficient = corr(theta_degrees, simout3);
% 
%         squared_error1 = (theta_degrees2 - simout4).^2;
%         rmse1 = sqrt(mean(squared_error1));
%         absolute_error1 = abs(theta_degrees2 - simout4);
%         mae1 = max(absolute_error1);
%         correlation_coefficient1 = corr(theta_degrees2, simout4);
% 
% 
%         % Mean Absolute Percentage Error calculation
%         mape = mean(abs((theta_degrees - simout3)./theta_degrees))*100;
%         mape1 = mean(abs((theta_degrees2 - simout4)./theta_degrees2))*100;
% 
%         % Store performance metrics
%         Abs_error{k} = rmse;
%         mae_error{k} = mae;
%         Mean_abs{k} = mean(absolute_error);
%         MAPE{k} = mape;  % Store MAPE
%         corr2{k} = correlation_coefficient;
% 
%         Abs_error1{k} = rmse1;
%         mae_error1{k} = mae1;
%         Mean_abs1{k} = mean(absolute_error1);
%         MAPE1{k} = mape1;  % Store MAPE
%         corr21{k} = correlation_coefficient1;
% % 
% %         % Extract execution time ignoring compilation
% % simInfo = out.getSimulationMetadata.TimingInfo;
% % simulinkExecutionTime = simInfo.ExecutionElapsedWallTime;
% % fprintf('Computation Time for Simulink Model Simulation (ignoring compilation): %f seconds\n', simulinkExecutionTime);
% 
% 
%  ideal = theta_degrees;% Ideal signal (replace with your actual data)
%  ideal1 = theta_degrees2;
% 
% real = simout3;  % Real signal with phase shift (replace with your actual data)
% real1 = simout4;
% 
% % Apply Hilbert transform to obtain the analytic signals
% analytic_ideal = hilbert(ideal);
% analytic_real = hilbert(real);
% analytic_ideal1 = hilbert(ideal1);
% analytic_real1 = hilbert(real1);
% 
% 
% % Calculate instantaneous phases
% phase_ideal = angle(analytic_ideal);
% phase_real = angle(analytic_real);
% 
% % Calculate phase difference in radians
% phase_difference_radians = phase_ideal - phase_real;
% 
% % Convert phase difference to degrees
% phase_difference_degrees = rad2deg(phase_difference_radians);
% 
% % Calculate maximum, minimum, and average phase difference
% max_phase_diff = max(phase_difference_degrees);
% min_phase_diff = min(phase_difference_degrees);
% avg_phase_diff = mean(phase_difference_degrees);
% 
% Max_PHASE{k} = max_phase_diff;
% Min_PHASE{k} = min_phase_diff;
% Avg_PHASE{k} = avg_phase_diff;
% 
% %Calculate instantaneous phases
% phase_ideal1 = angle(analytic_ideal1);
% phase_real1 = angle(analytic_real1);
% 
% % Calculate phase difference in radians
% phase_difference_radians1 = phase_ideal1 - phase_real1;
% 
% % Convert phase difference to degrees
% phase_difference_degrees1 = rad2deg(phase_difference_radians1);
% 
% % Calculate maximum, minimum, and average phase difference
% max_phase_diff1 = max(phase_difference_degrees1);
% min_phase_diff1 = min(phase_difference_degrees1);
% avg_phase_diff1 = mean(phase_difference_degrees1);
% 
% Max_PHASE1{k} = max_phase_diff1;
% Min_PHASE1{k} = min_phase_diff1;
% Avg_PHASE1{k} = avg_phase_diff1;
% 
%     end 
% 
% 
% figure;
% for k = 1:length(slewrate)
% 
%     logsout = simOut{k};% Get the simulation output data
%     signal = signalOut{k};
%     plot(ScopeData.time,signalOut{k});
%     hold on
%     plot(ScopeData.time,signalOut1{k});
% 
% end
% 
% 
%  grid on;
%  xlabel('seconds');
%  ylabel('Output');
%  legend('S= 1.7', 'S= 1.8', 'S= 1.9', 'S= 2', 'S= 2.1', 'S= 2.2', 'S= 2.3', 'S= 2.4', 'S= 2.5', 'S= 2.6', 'S= 2.7', 'S= 2.8',Location='northeast');
%  title('Response');
%  hold off




