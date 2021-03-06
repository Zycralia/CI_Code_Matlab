% populate the invariant model and parameters here
% =================================================================================================================
% invariant Model : ArduCopter
fprintf('!!!New Start!!!\n')
global A
global B
global C
global D
A = [0.8809 -0.2312 -0.0061; 0.0023 0.9997 -0.0000; 0.0000 0.0025 1.0000];
B = [0.0023; 0.0000; 0.0000];
C = [67.9605; 91.1797; 2.5415];
D = 0;
% monitoring parameters
global window
global threshold
window = 1.4355e+03; %121;   % why do they calculate? window size x 40 (121.0000 * 40)
threshold = 3.0677; %3.0189;  % why do they calculate? threshold x 10000 (3.0189 * 10000)    
% =================================================================================================================
% invariant state variables
    
pathname = 'valuation/val1011';     % log directory
Ts = 0.1;               % sample time in log
Ts2 = 0.0025;           % main loop rate
files = dir(strcat(pathname, '/*.csv'));
N = length(files);
data = cell(N);
for i = 1:N
    global inv_y
    global inv_x
    inv_y = 0.0;
    inv_x = [0.0; 0.0; 0.0];

    % accumulated errors
    global err_sum
    err_sum = 0;

    % local variables
    global w_index
    global max_mse
    global w_count;  % counter to know which window we are in at the moment of attackdetection
    w_index = 0;    % current index in window (here also start with 1 because matlab?)
    max_mse = 0;    % max error (for logs)
    w_count = 0;

    % attack states
    global attack_detected
    attack_detected = false;%not usefull because never used?

    %--------------- invariant start
    
    file = fullfile(pathname, files(i).name);
    disp(files(i).name)
    csv = csvread(file, 3, 0);
    L = length(csv);
    for j = 1:L
        tr = csv(j,3); %desired roll
        mr = csv(j,4); %measured roll
        copter_invariants_check(tr, mr) % checking desired yaw and measured yaw ans target and measured
    end
end

function copter_invariants_check(target, measured)
    global A
    global B
    global C
    global D
    global window
    global threshold
    global inv_y
    global inv_x
    global err_sum
    global w_index
    global w_count;
    global max_mse
    global attack_detected
        err_sum = 0;
        % y = Cx[i] + Du[i]
        inv_y = (C(1)*inv_x(1) + C(2)*inv_x(2) + C(3)*inv_x(3)) + D*target;
 
        % x' = Ax[i] + Bu[i]
        x0 = (A(1,1)*inv_x(1) + A(1,2)*inv_x(2) + A(1,3)*inv_x(3)) + B(1)*target;
        x1 = (A(2,1)*inv_x(1) + A(2,2)*inv_x(2) + A(2,3)*inv_x(3)) + B(2)*target;
        x2 = (A(3,1)*inv_x(1) + A(3,2)*inv_x(2) + A(3,3)*inv_x(3)) + B(3)*target;
        inv_x(1) = x0;
        inv_x(2) = x1;
        inv_x(3) = x2;   


        % update invariants (for logs)
        %invariant = inv_y; %not usefull because update of library for c++ not matlab?
        %error calculation
        error = inv_y - measured;           % error at each time point
        err_sum = err_sum + (error.*error);  % accumulated squared-error
        mse = err_sum / (w_index + 1);    % mse in the current window
        %ierror = mse; %not usefull because update of library c++ not matlab?
        
        if (mse > max_mse)      % maximum mse (for analysis)
            max_mse = mse;
        end
 
        w_index = w_index + 1;
        if(w_index >= window)    % new window
            w_index = 0;
            err_sum = 0;
            w_count = w_count + 1;
        end
    
        % check error
        if(mse > threshold) %&& invariant_enabled) invariant_enabled seems to be always true for our porposes
            % raise an alram
            attack_detected = true; %not usefull because never used?
            fprintf('!!!ATTACK DETECTED!!! in window')
            disp(w_count)
        end

end
