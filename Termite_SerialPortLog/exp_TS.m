% load('/home/cuichi/ownCloud/basinstab_experimental_data/[datafile].mat')
clear all; close all;
%% if SamplePeriod etc need to be provided manually:
SamplePeriodInSeconds = 10.7E-6;                                           % time-discretization of microcontroller         
bufferLength          = 51200;                                             % length of buffer from microcontroller
TcoupOnInSeconds      = 5E-3;                                              % time in seconds when coupling is turned on

numb_measurements     = 21;                                                % number of measurements to be evaluated
averThreeP            = round(3E-3/SamplePeriodInSeconds);                 % averaging of order parameter over a time of 3ms (3 x free running period)
averOneP              = round(1E-3/SamplePeriodInSeconds);                 % averaging of global frequency over a time of 1ms (3 x free running period)

fid = fopen('results.txt', 'wt');
fprintf(fid, 'phi1, phi2, phi3, lastR, meanR_3periodTw, StD_3periodTw, meanOmeg1_Tw, meanOmeg2_Tw, meanOmeg3_Tw\n');
s   = dir;                                                                 % load a structure of all the data
for i=1:numb_measurements
    filename   = sprintf('data%02i',i);                                    % change to %03i if more than 100 files and to %04i if more than 1000
    foldername = sprintf('results_data%02i',i);
    startrow   = 4;
    endrow     = 51200;

    Channel1 = dlmread(filename,'\t',[startrow,0,endrow,0]);
    Channel2 = dlmread(filename,'\t',[startrow,1,endrow,1]);
    Channel3 = dlmread(filename,'\t',[startrow,2,endrow,2]);

    initial_skip_time = 20E-6;                                             % initial time that is skipped before edge detection in seconds
    correction_enable = 1;
    tol               = (0.01*1E-3)/SamplePeriodInSeconds;

    %% generate time-vector and prepare raw data from experiments for plotting
    time = 0:SamplePeriodInSeconds:SamplePeriodInSeconds*(length(Channel1)-1); 
    time = time';
    fprintf('\nmeasurement time Tges = %0.2f seconds\n\n', time(length(time)));

    % calculate phases from measured signals, assume linearly growing phase between two flanks
    [t1, phase1, edges1] = fphase(Channel1, time, SamplePeriodInSeconds, initial_skip_time);
    [t2, phase2, edges2] = fphase(Channel2, time, SamplePeriodInSeconds, initial_skip_time);
    [t3, phase3, edges3] = fphase(Channel3, time, SamplePeriodInSeconds, initial_skip_time);

    fprintf('after construction phase length(t1): %0.d\n', length(t1))
    fprintf('after construction phase length(t2): %0.d\n', length(t2))
    fprintf('after construction phase length(t3): %0.d\n', length(t3))

    % check the edge detection - MAYBE, combine egdes and signals to make it easier to fit a sinusoidal function
    yvec_supp_edges1     = 0.5+zeros( length(edges1), 1);
    figure(999); hold on;
    plot(edges1, yvec_supp_edges1, '*'); plot(time, Channel1,'d-'); ylim([-0.05 1.05]);
    xlabel('time in [s]'); ylabel('signal and edges'); legend('edges','signal');

    % identify the vector of the data that starts at the latest time
    fprintf('start-times: {%0.6f,%0.6f,%0.6f} -- end-times: {%0.6f,%0.6f,%0.6f}', t1(1), t2(1), t3(1), t1(end), t2(end), t3(end))
    ts_max = max( [t1(1) t2(1) t3(1)] );
    ts_min = min( [t1(end) t2(end) t3(end)] );
    t1sPos = find( t1>ts_max ,1); t1ePos = find( abs(t1-ts_min)<1E-15 ); ophase1 = phase1((t1sPos-1):t1ePos);
    t2sPos = find( t2>ts_max ,1); t2ePos = find( abs(t2-ts_min)<1E-15 ); ophase2 = phase2((t2sPos-1):t2ePos);
    t3sPos = find( t3>ts_max ,1); t3ePos = find( abs(t3-ts_min)<1E-15 ); ophase3 = phase3((t3sPos-1):t3ePos);

    % correct for small discrepencies given a tolerance - i.e., oscillations in with periods of Milliseconds, hence we allow for 0.01 of a Millisecond
    if correction_enable == 1
        fprintf('\nlength(t1) before correction: %0.d\n', length(t1))
        fprintf('length(t2) before correction: %0.d\n', length(t2))
        fprintf('length(t3) before correction: %0.d\n', length(t3))
        if ((length(ophase1) - length(ophase2)) < tol ) || ((length(ophase2) - length(ophase3)) <tol)
            fprintf('\nCORRECTION of times performed!\n')
            minlengthphases = min( [length(ophase1) length(ophase2) length(ophase3)] );
            t1 = t1(1:minlengthphases); ophase1 = ophase1(1:minlengthphases); phase1 = phase1(1:minlengthphases);
            t2 = t2(1:minlengthphases); ophase2 = ophase2(1:minlengthphases); phase2 = phase2(1:minlengthphases);
            t3 = t3(1:minlengthphases); ophase3 = ophase3(1:minlengthphases); phase3 = phase3(1:minlengthphases);
        end
        time_order = t1;
    else
        time_order = t1(t1sPos-1:t1ePos);
    end

    % calculate order parameter over time
    orderparam = 1/3 * ( exp(1i*ophase1') + exp(1i*ophase2') + exp(1i*ophase3') );

    init_phase1 = mod(phase1(find(t1>=0.01,1)),2*pi);
    init_phase2 = mod(phase2(find(t2>=0.01,1)),2*pi);
    init_phase3 = mod(phase3(find(t3>=0.01,1)),2*pi);

    % output the initial phases
    fprintf('initial phase: %0.8f, %0.8f, %0.8f\n', init_phase1, init_phase2, init_phase3)
    fprintf('after correction length(t1): %0.d\n', length(t1))
    fprintf('after correction length(t2): %0.d\n', length(t2))
    fprintf('after correction length(t3): %0.d\n', length(t3))
    fprintf('length(time_order): %0.d\n', length(time_order))
    % NOTE: the first entries of t1, t2, t3 starts with the time of the first rising edge, not at zero 
    % HOWEVER: if the time-series of phases that have been constructed are not
    % equally long, there is the option to approximate that in certain bounds (tolerance)

    mkdir(foldername)
    cd(foldername)
    
    % plot evaluation
    % time-series
    figure(1); hold on;
    plot(time, Channel1, time, Channel2, time, Channel3);
    plot(TcoupOnInSeconds,0.5,'r*','MarkerSize',8);
    % xlim([0 0.01*time(end)]); 
    ylim([-0.05 1.05])
    xlabel('t in [s]');
    ylabel('x[t]');
    % print('-r300', '-depsc2', 'signals.eps');
    saveas(figure(1), 'signals.fig');

    % plot phases complete
    figure(2); hold on; 
    plot(t1, mod(phase1', 2*pi), t2, mod(phase2', 2*pi), t3, mod(phase3', 2*pi));
    plot(TcoupOnInSeconds,0.05,'r*','MarkerSize',8);
    plot(TcoupOnInSeconds,init_phase1,'k.','MarkerSize',8);
    plot(TcoupOnInSeconds,init_phase2,'k.','MarkerSize',8);
    plot(TcoupOnInSeconds,init_phase3,'k.','MarkerSize',8);
    xlabel('t in [s]');
    ylabel('\theta(t)');
    % print('-r300', '-depsc2', 'phases_complete.eps');
    % saveas(figure(2), 'phases_complete.fig');

    % phase differences
    figure(3); hold on;
    plot(time_order, mod((ophase1-ophase2+pi), 2*pi)-pi, time_order, mod((ophase2-ophase3+pi), 2*pi)-pi);
    plot([TcoupOnInSeconds; TcoupOnInSeconds], [-5*pi; 5*pi], 'k--', 'LineWidth', 1.5);
    % plot(time_order, mod(ophase1-ophase2, 2*pi), '*', time_order, mod(ophase2-ophase3, 2*pi), '.');
    % plot(time_order, mod(ophase1-ophase2, 2*pi), time_order, mod(ophase2-ophase3, 2*pi));
    % xlim([0 0.01*time(end)]); 
    ylim([-1.1*pi 1.1*pi])
    xlabel('t in [s]');
    ylabel('\theta_1-\theta_2, \theta_2-\theta_3');
    % print('-r300', '-depsc2', 'phase_diffs.eps');
    saveas(figure(3), 'phase_diffs.fig'); saveas(figure(3), 'phase_diffs.png');

    % plot phases from the smallest common time defined by the first raising flank until the latest common time
    figure(4); hold on; 
    plot(time_order, mod(ophase1', 2*pi), time_order, mod(ophase2', 2*pi), time_order, mod(ophase3', 2*pi));
    xlabel('t in [s]');
    ylabel('\theta(t)');
    % print('-r300', '-depsc2', 'phases.eps');
    saveas(figure(4), 'phases.fig'); 

    % plot order parameter
    figure(5); hold on; 
    plot(time_order, abs(orderparam)); plot(TcoupOnInSeconds,0.5,'r*','MarkerSize',8);
    xlabel('t in [s]');
    ylabel('R(t)');
    % print('-r300', '-depsc2', 'order_parameter.eps');
    saveas(figure(5), 'order_parameter.fig'); saveas(figure(5), 'order_parameter.png');

    % plot instantaneous frequencies
    figure(6); hold on; 
    ff1 = smooth(diff(unwrap(ophase1))/(2*pi*SamplePeriodInSeconds),1000);
    ff2 = smooth(diff(unwrap(ophase2))/(2*pi*SamplePeriodInSeconds),1000);
    ff3 = smooth(diff(unwrap(ophase3))/(2*pi*SamplePeriodInSeconds),1000);
    plot(time_order(2:length(time_order)), ff1,'c');
    plot(time_order(2:length(time_order)), ff2,'b');
    plot(time_order(2:length(time_order)), ff3,'k');
    xlabel('t in [s]');
    ylabel('f(t) in [Hz]');
    % print('-r300', '-depsc2', 'order_parameter.eps');
    saveas(figure(6), 'instant_freq.fig'); saveas(figure(6), 'instant_freq.png');
    
    cd ..
    % save results to one common file
    fprintf(fid, '%0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f\n', ...
        init_phase1, init_phase2, init_phase3, abs(orderparam(end)), mean(abs(orderparam(end-averThreeP:end))), std(abs(orderparam(end-averThreeP:end))), ...
        mean(ff1(end-averOneP:end)), mean(ff2(end-averOneP:end)), mean(ff3(end-averOneP:end)) );
    pause(1);
    close all;
end
fclose(fid);                                                               % close results file
%% define function to find edges in digital signal
% for test purposes %%%%%%%%%%%%%%%%
% dt = SamplePeriodInSeconds;
% t  = time; 
sig= Channel1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t,y, edgetimes] = fphase(sig, t, dt, ist)
    if ( ~isvector(sig) || length(sig) ~= length(t) )
        error('Input must be a vector with the time-series of the signal or signal and time container have different lengths!')
    end
    % for the propper initial condition, find the first raising edge of the signal!
    skip_steps = floor(ist/dt); sig = sig(skip_steps:end); t = t(skip_steps:end);
    edgetimes  = [];
    if find( sig == 0, 1 ) == 1                                            % if the digital signal is zero initially, 
        first_raising_edgePos = find( sig ~= 0, 1 );                       % the first 1 found corresponds to the raising edge 
        edgetimes = [edgetimes t(first_raising_edgePos)];
    else                                                                   % if digital signal initially at one, then find the first zero 
        first_falling_edgePos = find( sig ~= 1, 1 );                       % and then from there the first rising edge
        first_raising_edgePos = (first_falling_edgePos-1)+find( sig(first_falling_edgePos:end) ~= 0, 1 );
        edgetimes = [edgetimes t(first_raising_edgePos)];
    end    
    fprintf('Found first raising edge in entry %i, i.e., at time %0.4f\n', first_raising_edgePos, edgetimes(1))
    % extract the times of all edges from the digital signal and store in edgetimes
    for i = first_raising_edgePos:(length(sig)-1)
        if sig(i) ~= sig(i+1)                                              % CAREFUL WITH UINT8: 0-1=0 if the numbers are of type uint
            edgetimes = [edgetimes t(i)];
        end
    end  
    % create vector, supplementing the edges vector, that grow for each entry by pi
    % also generate a corresponding vector of the times, such that the data can be aligned properly when plotting
    y = [0]; t = [edgetimes(1)];
    for i = (1:length(edgetimes)-1)
        slope_per_timestep       = (pi*dt)/(edgetimes(i+1)-edgetimes(i));
        last_phase_value         = y(length(y));
        time_steps_between_edges = round((edgetimes(i+1)-edgetimes(i))/dt);
        temp_time = 1:time_steps_between_edges; temp_time=temp_time*dt; 
%         t = [t edgetimes(i):dt:(edgetimes(i+1)-dt)]; 
        t = [t (edgetimes(i)+temp_time)];
%         fprintf('\nAdding %i time-steps.', length(edgetimes(i):dt:(edgetimes(i+1)-dt)))
        y = [y (last_phase_value+(1:time_steps_between_edges)*slope_per_timestep)];
%         fprintf('\nAdding %i phase-points.', length((last_phase_value+(1:(time_steps_between_edges))*slope_per_timestep)))
%         if( length((last_phase_value+(1:(time_steps_between_edges))*slope_per_timestep)) ~= length(edgetimes(i):dt:(edgetimes(i+1)-dt)) )
%             fprintf('\nDifference added phase-points and times found: %i', length((last_phase_value+(1:(time_steps_between_edges))*slope_per_timestep)) - length(edgetimes(i):dt:(edgetimes(i+1)-dt)))
%         else
%             fprintf('\nNo difference!')
%         end
    end
    
    
    
%     y = [0]; t = [];
%     for i = (1:length(edgetimes)-1)
%         slope_per_timestep       = (pi*dt)/(edgetimes(i+1)-edgetimes(i));
%         last_phase_value         = y(length(y));
%         time_steps_between_edges = round((edgetimes(i+1)-edgetimes(i))/dt);
%         t = [t edgetimes(i):dt:(edgetimes(i+1)-dt)];
%         y = [y (last_phase_value+(1:(time_steps_between_edges))*slope_per_timestep)];
%     end
%     t = [t t(end)+dt];

    % from the edges vector, that contains the times of the edges and the corresponding phase vector we construct the phase vector by 
end
% Ansatz fitting %%%%%%%%%%%%%%%%%%%%%%
function [fitresult, gof] = createFit(time, Channel1)                      % directly fit the digital signal with a sine-curve
%CREATEFIT(TIME,CHANNEL1)
%  Create a fit.
%
%  Data for 'fit with sinusoidal signal' fit:
%      X Input : time
%      Y Output: Channel1
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 17-Apr-2018 13:25:04
%% Fit: 'fit with sinusoidal signal'.
[xData, yData] = prepareCurveData( time, Channel1 );

% Set up fittype and options.
ft = fittype( '0.5+0.5*sin(a*x+b)', 'independent', 'x', 'dependent', 'y' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.Lower = [3500 0];
opts.StartPoint = [4000 0.01];
opts.Upper = [10000 7];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, xData, yData );
legend( h, 'Channel1 vs. time', 'untitled fit 1', 'Location', 'NorthEast' );
% Label axes
xlabel time
ylabel Channel1
grid on
end
