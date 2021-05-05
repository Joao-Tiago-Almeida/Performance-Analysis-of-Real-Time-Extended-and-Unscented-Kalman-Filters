clear;
close all;
clearvars my_timer;
% FIM ou IEKF-eq - qUASI nEWTON (QN-EKF) ou IEKF ou Levenberg-Marquardy
% Iterated EKF


%% EKF 

%% Simulation path
% Car path
x_start =0;y_start=0;
[x_teo,y_teo,theta_teo,phi_teo] = testingrobot(x_start,y_start);

% Test Both paths

% figure(30);
% plot(x_teo,y_teo)
% hold on;
% plot(x_teo1,y_teo1)


IMU_data = [awgn(x_teo',15/0.0001,'measured','linear'),awgn(y_teo',15/0.0001,'measured','linear'),awgn(theta_teo',15/0.00001,'measured','linear')];
x_new = zeros(size(IMU_data,1),1); x_new(1) = x_teo(1);
y_new = zeros(size(IMU_data,1),1); y_new(1) = y_teo(1);
theta_new = zeros(size(IMU_data,1),1); theta_new(1) = theta_teo(1);
xunc = .01; % 
yunc = .01; % 

Q = eye(3).*0.01;
R = eye(3).*0.01;
P = [xunc^2 0 0 ; 0 xunc^2 0 ;0 0 (xunc*0.01)^2];
for i = 2:size(x_teo,2)

    %% Prediction Phase

    %% Process State
    Norma = norm([x_teo(i) y_teo(i)]-[x_teo(i-1) y_teo(i-1)]);
    x_new(i) = x_new(i-1) + Norma*cos(theta_new(i-1));
    y_new(i) = y_new(i-1) + Norma*sin(theta_new(i-1));
    theta_new(i) = IMU_data(i,3);


    F = [1 0 -Norma*sin(theta_new(i-1));...
         0 1 Norma*cos(theta_new(i-1)); ...
         0 0 1];

    %% Process Covariance

    P = F*P*F';

    %% Update Phase
    Norma =  norm([x_new(i) y_new(i)]);
    H = [((x_new(i))/Norma) ((y_new(i))/Norma) 0; ...
          ((-y_new(i))/(x_new(i)^2 + y_new(i)^2)) ((x_new(i))/(x_new(i)^2 + y_new(i)^2)) 0];

    Norma = norm([x_teo(i) y_teo(i)]);
    y_hat = [norm([x_new(i) y_new(i)]);atan(y_new(i)/x_new(i))];
    y_theory = [norm([x_teo(i) y_teo(i)]);atan(y_teo(i)/x_teo(i))];
    y = y_theory - y_hat;
    K = P*H'/(H*P*H' + H*Q*H');
    aux =  [x_new(i);y_new(i);theta_new(i)] + K*y;
    P = (eye(size(K,1))-K*H)*P;

    x_new(i) = aux(1);
    y_new(i) = aux(2);
    theta_new(i) = aux(3);

end

% figure();
% plot(x_teo,y_teo);
% hold on;
% plot(IMU_data(:,1),IMU_data(:,2));
% hold on;
% plot(x_new,y_new);
% legend('True Course','Odometry','EKF Based');
% legend show;

%% UEKF
x_new_uEKF = zeros(size(IMU_data,1),1); x_new_uEKF(1) = x_teo(1);
y_new_uEKF = zeros(size(IMU_data,1),1); y_new_uEKF(1) = y_teo(1);
theta_new_uEKF = zeros(size(IMU_data,1),1); theta_new_uEKF(1) = theta_teo(1);
xunc = .01; %  

Q = eye(3).*0.0001;
R = eye(3).*0.01;

P = [xunc^2 0 0 ; 0 xunc^2 0 ;0 0 (xunc*0.01)^2];
for i = 2:size(x_teo,2)
    
    %%%%%%%%%%%%% Initialising measurement and process noise %%%%%%%%%%%%%
    Norma = norm([x_teo(i) y_teo(i)]-[x_teo(i-1) y_teo(i-1)]);

    x_new_uEKF(i) = x_new_uEKF(i-1) + Norma*cos(theta_new_uEKF(i-1));
    y_new_uEKF(i) = y_new_uEKF(i-1) + Norma*sin(theta_new_uEKF(i-1));
    theta_new_uEKF(i) = IMU_data(i,3);

    F = [1 0 -Norma*sin(theta_new_uEKF(i-1));...
         0 1 Norma*cos(theta_new_uEKF(i-1)); ...
         0 0 1];
%     %%%%%%%%%%%%% Predict Stage %%%%%%%%%%%%%
% 
%     %%%%%%%%%%%%% State Process %%%%%%%%%%%%%
    
%     %%%%%%%%%%%%% Defining the Terms of the Measurement Jacobian %%%%%%%%%%%%%
%    
    Norma =  norm([x_new_uEKF(i) y_new_uEKF(i)]);
    H = [((x_new_uEKF(i))/Norma) ((y_new_uEKF(i))/Norma) 0; ...
          ((-y_new_uEKF(i))/(x_new_uEKF(i)^2 + y_new_uEKF(i)^2)) ((x_new_uEKF(i))/(x_new_uEKF(i)^2 + y_new_uEKF(i)^2)) 0];

    y_theory = [norm([x_teo(i) y_teo(i)]);atan(y_teo(i)/x_teo(i))];



    L=3;                                 %numer of states
    m=2;                                 %numer of measurements
    alpha=1e-3;                                 %default, tunable
    ki=0;                                       %default, tunable
    beta=2;                                     %default, tunable
    lambda=1-L;                    %scaling factor
    c=L+lambda;                                 %scaling factor
    Wm=[lambda/c 0.5/c+zeros(1,2*L)];           %weights for means
    Wc=Wm;
    Wc(1)=Wc(1)+(1-alpha^2+beta);    
    c=sqrt(c);
    
    % Acho que só leio 2*L e não 2*L + 1
    x_pos_uEKF = [x_new_uEKF(i-1);y_new_uEKF(i-1);theta_new_uEKF(i-1)];
    xsigma_post=sigmas(x_pos_uEKF,P,c);
    sum_group = zeros(3,1);
    pos_group = zeros(3,2*L+1);
    for j = 1:2*L+1
        Norma = norm([x_teo(i) y_teo(i)]-[x_teo(i-1) y_teo(i-1)]);

        x_new1 = xsigma_post(1,j) + Norma*cos(xsigma_post(3,1));
        y_new1 = xsigma_post(2,j) + Norma*sin(xsigma_post(3,1));
        theta_new1 = theta_teo(i);
        pos_group(:,j) = [x_new1 y_new1 theta_new1];
        sum_group = sum_group + Wm(j)*pos_group(:,j);      
        
    end
    
    Deviations = pos_group - sum_group(:,ones(1,2*L+1));
    P_pos = Deviations*diag(Wc)*Deviations' + F*R*F';
    
    %% Measurements
    sum_group_mea = zeros(2,1);
    pos_group_mea = zeros(2,2*L+1);    
    for j = 1:2*L+1

        y_hat_test = [norm([xsigma_post(1,j) xsigma_post(2,j)]);atan(xsigma_post(2,j)/xsigma_post(1,j))];
        pos_group_mea(:,j) = y_hat_test;
        sum_group_mea = sum_group_mea + Wm(j)*pos_group_mea(:,j);      
        
    end
    
    Deviations_mea = pos_group_mea - sum_group_mea(:,ones(1,2*L+1));
    P_mea = Deviations_mea*diag(Wc)*Deviations_mea' + H*Q*H';

    P12 = Deviations*diag(Wc)*Deviations_mea';
    K = P12*inv(P_mea);

    aux = sum_group + K*(y_theory-sum_group_mea); 
    P = P_pos - K*P12';
   
   x_new_uEKF(i) = aux(1);
    y_new_uEKF(i) = aux(2);
    theta_new_uEKF(i) = aux(3);


end
figure();
plot(x_teo,y_teo);
hold on;
plot(IMU_data(:,1),IMU_data(:,2));
hold on;
plot(x_new,y_new);
hold on;
plot(x_new_uEKF,y_new_uEKF);

legend('True Course','Odometry','EKF Based','UEKF Based');
legend show;

function X=sigmas(x,P,c)
%Sigma points around reference point
%Inputs:
%       x: reference point
%       P: covariance
%       c: coefficient
%Output:
%       X: Sigma points
P = [P(1,1) 0 0;0 P(2,2) 0; 0 0 P(3,3)];
A = c*chol(P)';
Y = x(:,ones(1,numel(x)));
X = [x Y+A Y-A]; 
end

function [x_vec,y_vec,theta_vec,phi_vec] = testingrobot(x_start,y_start)


start_v = 0;
my_timer = timer('Name', 'my_timer', 'ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                    'StartFcn', @(x,y)disp('started...'), ...
                    'StopFcn', @(x,y)disp('stopped ...'), ...
                    'TimerFcn', @my_start_fcn);
x = x_start;
y = y_start;
t = 0;
theta = pi/4;
x_old = x;
y_old = y;
v = 0.5;
phi = 0;
w_phi = 0.01;
dx = cos(theta)*0.02;
dy = sin(theta)*0.02;
figure
plot([x,x+dx],[y,y+dy],'b');
hold on;
plot(x,y,'O');
start(my_timer);
while t < 50
    if start_v == 1
        x_old = x;
        y_old = y;
        [x,y,theta,phi] = robot_simulation(x, y, theta, v, phi, w_phi);
        dx = cos(theta)*0.002;
        dy = sin(theta)*0.002;
        plot([x,x+dx],[y,y+dy],'b');
        plot(x,y,'O');
        x_vec(t+1) = x;
        y_vec(t+1) = y;
        theta_vec(t+1) = theta;
        phi_vec(t+1) = phi;
        t = t + 1;
        
        start_v = 0;
    end
end



function my_start_fcn(obj, event)
    start_v = 1;
end




end
