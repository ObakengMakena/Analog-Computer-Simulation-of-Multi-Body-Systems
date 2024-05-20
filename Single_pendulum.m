tic;
% Pendulum parameters
L = 2; % length of the pendulum
g = 9.81; % acceleration due to gravity
b = 1; % damping coefficient
m = 1; % mass of pendulum
I = 0.8; % moment of inertia of pendulum


%inital test conditions l=2 b=1 m=5 I=0.8
% resistor= 0.0815494


% Initial conditions
theta0 = pi/9; % initial angle (in radians)
omega0 = 0; % initial angular velocity


% Combine initial conditions into a vector
y0 = [theta0; omega0];

% Time span for integration
tspan = 0:0.001:10; % from t=0 to t=10

% Solve the ODE
options = odeset('AbsTol',1e-9,'RelTol',1e-9);
[t, y] = ode45(@(t, y) dampedPendulumODE(t, y, L, g, b, m, I), tspan, y0, options);

% Extract solutions
theta_solution = y(:, 1); % pendulum angle

% Convert theta to degrees for plotting
theta_degrees = rad2deg(theta_solution);

% Define the second-order ODE for a damped pendulum
function dydt = dampedPendulumODE(t, y, L, g, b, m, I)
    % y(1) = theta, y(2) = dtheta/dt
    dydt = [y(2); -(m*g*L/I) * sin(y(1)) - (b/I) * y(2) ];
end

% Stop the timer and display the elapsed time
matlabSimTime = toc;
fprintf('Computation Time for MATLAB Code Simulation: %f seconds\n', matlabSimTime);




function overshoot = calculateOvershoot(output)
    final_value = output(end);
    if final_value ~= 0
        overshoot = (max(output) - final_value) / final_value * 100;
    else
        overshoot = NaN;
    end
end


% Plot the pendulum motion
plot(t, theta_degrees, 'g-');
hold on
plot(ScopeData.time, simout, 'r-');
plot(ScopeData.time, simout2, 'b-');
xlabel('Time (s)');
ylabel('Pendulum Angle (degrees)');
title('Damped Pendulum Motion');

%Adding Legend
legend('Code Result', 'ideal result', 'Real result');


hold off


% Simulink Model Simulation Computation Time
% Replace 'single_pendulum_model' with the actual name of your Simulink model

% 
% open_system("real_pendulum");
% 
% 
% slewrate = [ 0.001 0.002 0.005 0.007 0.009 0.01 0.02 0.05 0.07 0.09 0.1 0.2 0.5 0.7 0.9 1 2 5 7 9 10 20];
% % Preallocate storage for results
% Abs_error = cell(1, length(slewrate));
% mae_error = cell(1, length(slewrate));
% Mean_abs = cell(1, length(slewrate));
% corr2 = cell(1, length(slewrate));
% MAPE = cell(1, length(slewrate));  % Add storage for MAPE
% Max_PHASE = cell(1, length(slewrate));
% Min_PHASE = cell(1, length(slewrate));
% Avg_PHASE = cell(1, length(slewrate));
% simInfo = cell(1, length(slewrate));
%     for k = 1:length(slewrate)
% 
% 
%         set_param([bdroot, '/Resistor9'], 'R', num2str(slewrate(k)));
%         set_param([bdroot, '/Resistor3'], 'R', num2str(slewrate(k)));
%         % set_param([bdroot, '/Band-Limited Op-Amp7'], 'Rin', num2str(slewrate(k)));
%         % set_param([bdroot, '/Band-Limited Op-Amp6'], 'Rin', num2str(slewrate(k)));
% 
%         simOut{k} = sim(gcs);
%         disp(['Completed ', num2str(k), ' of ', num2str(length(slewrate)), ' simulations']);
%         signalOut{k} = simout2;

%         % Extract execution time ignoring compilation
% simInfo{k} = out.getSimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
% simulinkExecutionTime = simInfo{k};
% fprintf('Computation Time for Simulink Model Simulation (ignoring compilation): %f seconds\n', simulinkExecutionTime);
% 

% %  % Error and correlation calculations
% 
%         squared_error = (simout2- theta_degrees).^2;
%         rmse = sqrt(mean(squared_error));
%         absolute_error = abs(simout2- theta_degrees);
%         mae = max(absolute_error);
%         correlation_coefficient = corr(theta_degrees, simout2);
% 
%         % Mean Absolute Percentage Error calculation
%         mape = mean(abs((simout2-theta_degrees)./simout2))*100;
% 
%         % Store performance metrics
%         Abs_error{k} = rmse;
%         mae_error{k} = mae;
%         Mean_abs{k} = mean(absolute_error);
%         MAPE{k} = mape;  % Store MAPE
%         corr2{k} = correlation_coefficient;
% 
% 
% 
% 
%         ideal = theta_degrees;  % Ideal signal (replace with your actual data)
%         real = simout2;  % Real signal with phase shift (replace with your actual data)
% 
% % Apply Hilbert transform to obtain the analytic signals
% analytic_ideal = hilbert(ideal);
% analytic_real = hilbert(real);
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

% Display the max, min, and average phase differences
% disp(['Maximum Phase Difference: ', num2str(max_phase_diff), ' degrees']);
% disp(['Minimum Phase Difference: ', num2str(min_phase_diff), ' degrees']);
% disp(['Average Phase Difference: ', num2str(avg_phase_diff), ' degrees']);

% Plot the phase difference in degrees
% figure;
% plot(t, phase_difference_degrees);
% hold on
% plot(t, test, 'g-');
% 
% plot(ScopeData.time, test1, 'r-');
% plot(ScopeData.time, simout2, 'b-');
% 
% xlabel('Time [s]');
% ylabel('Phase Difference [degrees]');
% title('Phase Difference Over Time');
% legend('Phase Difference');
% grid on;  % Adds a grid to the plot for better visibility
% % Plot the pendulum motion
%     end 


figure;
for k = 1:length(slewrate)

    logsout = simOut{k};% Get the simulation output data
    signal = signalOut{k};
    plot(ScopeData.time,signalOut{k});



end


 grid on;
 xlabel('seconds');
 ylabel('Output');
 legend('S= 1.7', 'S= 1.8', 'S= 1.9', 'S= 2', 'S= 2.1', 'S= 2.2', 'S= 2.3', 'S= 2.4', 'S= 2.5', 'S= 2.6', 'S= 2.7', 'S= 2.8',Location='northeast');
 title('Response');
 hold off




% % Initialize the required parameters and vectors
% slewrate = [0.1e6 0.2e6 0.3e6 0.4e6 0.5e6 0.7e6 0.8e6 0.9e6 1e6	1.5e6	2e6 2.5e6 3e6 3.5e6 4e6 4.5e6 5e6 6e6 7e6 8e6 9e6 10e6 20e6 30e6 40e6 45e6 50e6];  % Define your slew rate values
%   % Set your Simulink model name
% 
% % Preallocate storage for results
% Abs_error = cell(1, length(slewrate));
% mae_error = cell(1, length(slewrate));
% Mean_abs = cell(1, length(slewrate));
% corr2 = cell(1, length(slewrate));
% MAPE = cell(1, length(slewrate));  % Add storage for MAPE
% 
% % Main simulation loops
% for i = 1:10
%     for k = 1:length(slewrate)
% 
%         % Set the slew rate for the op-amps
%         set_param([bdroot, '/Band-Limited Op-Amp4'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp5'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp6'], 'Vdotmax', num2str(slewrate(k)));
%         set_param([bdroot, '/Band-Limited Op-Amp7'], 'Vdotmax', num2str(slewrate(k)));
% 
%         % Run the simulation
%         simOut{k} = sim(gcs);
%         disp(['Completed ', num2str(k), ' of ', num2str(length(slewrate)), ' simulations']);
%         signalOut{k} = simOut{k}.signals.values;
% 
%         % Error and correlation calculations
%         squared_error = (theta_degrees - signalOut{k}).^2;
%         rmse = sqrt(mean(squared_error));
%         absolute_error = abs(theta_degrees - signalOut{k});
%         mae = max(absolute_error);
%         correlation_coefficient = corr(theta_degrees, signalOut{k});
% 
%         % Mean Absolute Percentage Error calculation
%         mape = mean(abs((theta_degrees - signalOut{k})./theta_degrees))*100;
% 
%         % Store performance metrics
%         Abs_error{k} = rmse;
%         mae_error{k} = mae;
%         Mean_abs{k} = mean(absolute_error);
%         MAPE{k} = mape;  % Store MAPE
%         corr2{k} = correlation_coefficient;
%     end
% 
%     % Display completion of the current cycle
%     disp(['Completed ', num2str(i), ' of 10', ' cycles']);
% end
% 
% % Post-simulation analysis
% % Frequency response analysis (if sys is defined)
% if exist('sys', 'var')
%     [H, W] = freqresp(sys);
%     figure;
%     subplot(2,1,1);
%     semilogx(W, 20*log10(abs(H)));  % Magnitude in dB
%     title('Bode Plot - Magnitude');
%     subplot(2,1,2);
%     semilogx(W, rad2deg(angle(H)));  % Phase in degrees
%     title('Bode Plot - Phase');
% end
% 
% % Harmonic distortion analysis (if thd function is available)
% if exist('thd', 'file')
%     THD = thd(signalOut{k});
%     disp(['Total Harmonic Distortion: ', num2str(THD), ' %']);
% end
% 
% % Step response analysis (if sys is defined)
% if exist('sys', 'var')
%     step(sys);
%     title('Step Response');
% end
% 
% % Plotting all signals
% figure;
% hold on;
% for k = 1:length(slewrate)
%     plot(signalOut{k});
% end
% title('Simulation Outputs Across Slew Rates');
% legend(arrayfun(@(x) ['Slew Rate ', num2str(x)], slewrate, 'UniformOutput', false));
% hold off;