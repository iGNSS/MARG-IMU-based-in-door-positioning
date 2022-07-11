% Allan Variance analyzer: This script takes in computed Allan Deviation,
% averaging time (T), along with desired slopes and averaging times of
% interest to extract noise coefficients of interest.
% [sigmasOut,f,h] = AnalyzeAVAR(AD,dt,T,Slopes,Ts,plots,method)
% 
% Outputs:
% sigmasOut: Vector of noise coefficients of interest
% f: figure handles to Allan Deviation and Slope plots 
% h: axis handles to plotted elements
%
% Inputs:
% AD: An Allan deviation vector
% T: The corresponding averaging time vector for Allan deviation
% Slopes: A vector containing Allan deviation slopes of interest
% Ts: The corresponding vector containing averaging times of interest for
% extrapolation. Use NaN for slopes with no averaging time of interest such
% as bias instability.
% plots: 0 for no plots, 1 for plots
% method: 1 for left-to-right slope search, 2 for min distance search
% 
% Author: Juan Jurado, Air Force Institute of Technology, 2017

function [sigmasOut,Tbias,f,h] = AnalyzeAVAR(AD,T,Slopes,Ts,plots,method,col)

% First ensure each Slope has a corresponding Ts
if length(Slopes)~=length(Ts)
    error('Each slope must have a corresponding Ts. Enter NaN if no Ts')
end

% Next, compute Log-Log slope of input Allan deviation signal. Log-Log
% slope is: (Log10(Y(t+1))-Log10(Y(t)))/((Log10(X(t+1))-Log10(X(t)))
slope = logdiff(AD)./logdiff(T);

% Create basic figure shells if plots were requsted
if plots
    h = zeros(2,length(Slopes)+1);
    f(1) = figure;
    h(1,1) = loglog(T,AD,'LineWidth',2); hold on; grid on;
    if col>0 && col<4%是加速度
    title('Allan Deviation (deg/h)')
    else%是角速度
        title('Allan Deviation (m/s^2/h)')
    end
    xlabel('Averaging Time (h)');
    ylabel('Allan Deviation');
    f(2) = figure;
    h(2,1) = semilogx(T(2:end),slope); hold on; grid on;
    title('Slope of Allan Deviation');
    xlabel('Averaging Time');
    ylabel('Log-Log Slope');
    
    % Color string for marking up each slope of interest in figures
%     c = {'r';'b';'k';'g';'y'};
 c = {'r';'y';'g';'b';'k'};%MXF
else
    h = 0;
    f = 0;
end

% Creaate output vector shell
sigmasOut = zeros(length(Slopes),1);
Tbias = [];

% Now, step through each slope of interest, find the slope in the Allan
% deviation signal, extrapolate to corresponding Ts of interest and record
% the coefficient found at that location.
for ii = 1:length(Slopes)
        curr_slope = Slopes(ii);
        curr_t = Ts(ii);
        
        % Method 1: Use knowledge of Allan deviation shape, starting from
        % the left (small T), look for first occurence of desired slope.
        % Method 2: Use min distance method to locate point with closest
        % matching slope to the one desired.
        if method==1
            if sign(slope(1)) == -1
                idx = find(slope>curr_slope,1,'first');
            else
                idx = find(slope<curr_slope,1,'first');
            end
        elseif method==2
            dist = (slope-curr_slope).^2;
           [~,idx] = min(dist);
        end

        % If the desired slope is found within the signal, create anonymus
        % function for the line to extrapolate to coresponding Ts.
        if ~isempty(idx)
            if isnan(curr_t)
                sigmasOut(ii) = AD(idx);
                Tbias=T(idx);
                linefun = @(t) AD(idx) + 0*t;
            else
                linefun = @(t) 10.^(curr_slope*(log10(t)-log10(T(idx)))...
                    +log10(AD(idx)));
                sigmasOut(ii) = linefun(curr_t);
            end

            if plots%% && (ii==2 || ii==3)%MXF
                figure(f(1)); h(1,ii+1) = plot(T(idx),AD(idx)...
                    ,'s','Color',c{ii},'MarkerFaceColor',c{ii});...
                    plot(T,linefun(T),'--','LineWidth',1.2,'Color',c{ii});
                figure(f(2)); h(2,ii+1) = plot(T(idx+1),...
                    slope(idx),'s','Color',c{ii},'MarkerFaceColor',c{ii});
            end
        end
end

% Compute Log10(X(t+1))-Log10(X(t)) for use in Log-Log slope
function logdiff = logdiff(X)
N = size(X,1);
logdiff = zeros(N-1,1);
for ii = 1:N-1
    logdiff(ii) = log10(X(ii+1))-log10(X(ii));
end
    