clear;
close all;
clearvars my_timer;


%% User choice for the path
switch menu('Do you want to draw a path?','Yes, sure!','No, use the pre-defined.')
    %% Drawing a path
    case 1 
        path = drawfreehand('Closed',false,'FaceAlpha',0,'InteractionsAllowed','none');
        x_teo = path.Position(:,1)';
        y_teo = path.Position(:,2)';
        theta_teo = atan2(diff(y_teo) , diff(x_teo));   % compute direction [rad]
        theta_teo = [theta_teo, theta_teo(end)];    % the last direction does not vary
        phi_teo = diff(theta_teo);    % compute angular velocity [rad/s]
        phi_teo = [phi_teo, phi_teo(end)];  % the last velocity does not vary
        delete(path);
        close;
    %% Simulation path
	case 2
        x_start =0;y_start=0; theta_start=pi/4;
        [x_teo,y_teo,theta_teo,phi_teo] = testingrobot(x_start,y_start,theta_start);
end

%% EKF 

% Test Both paths

% figure(30);
% plot(x_teo,y_teo)
% hold on;
% plot(x_teo1,y_teo1)


IMU_data = [awgn(x_teo',15/0.0001,'measured','linear'),awgn(y_teo',15/0.0001,'measured','linear'),awgn(theta_teo',15/0.000001,'measured','linear')];
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
          ((-(y_new(i)-y_new(i-1)))/((x_new(i)-x_new(i-1))^2 + (y_new(i)-y_new(i-1))^2)) ((x_new(i)-x_new(i-1))/((x_new(i)-x_new(i-1))^2 + (y_new(i)-y_new(i-1))^2)) 0];

    Norma = norm([x_teo(i) y_teo(i)]);
    y_hat = [norm([x_new(i) y_new(i)]);atan2((y_new(i)-y_new(i-1)),(x_new(i)-x_new(i-1)))];

    y_theory = [norm([x_teo(i) y_teo(i)]);atan2((y_teo(i)-y_teo(i-1)),(x_teo(i)-x_teo(i-1)))];

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

%% UKF
x_new_uEKF = zeros(size(IMU_data,1),1); x_new_uEKF(1) = x_teo(1);
y_new_uEKF = zeros(size(IMU_data,1),1); y_new_uEKF(1) = y_teo(1);
theta_new_uEKF = zeros(size(IMU_data,1),1); theta_new_uEKF(1) = theta_teo(1);
xunc = .01; %  

Q = eye(3).*0.001;
R = eye(3).*0.00001;

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
          ((-(y_new_uEKF(i)-y_new_uEKF(i-1)))/((x_new_uEKF(i)-x_new_uEKF(i-1))^2 + (y_new_uEKF(i)-y_new_uEKF(i-1))^2)) ((x_new_uEKF(i)-x_new_uEKF(i-1))/((x_new_uEKF(i)-x_new_uEKF(i-1))^2 + (y_new_uEKF(i)-y_new_uEKF(i-1))^2)) 0];


    y_theory = [norm([x_teo(i) y_teo(i)]);atan2((y_teo(i)-y_teo(i-1)),(x_teo(i)-x_teo(i-1)))];

    L=3;                                 %numer of states
    m=2;                                 %numer of measurements
    alpha=1e-3;                                 %default, tunable
    ki=0;                                       %default, tunable
    beta=2;                                     %default, tunable
    lambda=3-L;                    %scaling factor
    c=L+lambda;                                 %scaling factor
    Wm=[lambda/c 0.5/c+zeros(1,2*L)];           %weights for means
    Wc=Wm;
    Wc(1)=Wc(1)+(1-alpha^2+beta);    
    c=sqrt(c);
    
    % Acho que s� leio 2*L e n�o 2*L + 1
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

        y_hat_test = [norm([pos_group(1,j) pos_group(2,j)]);atan2(pos_group(2,j) - y_new_uEKF(i-1),pos_group(1,j)-x_new_uEKF(i-1))];
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


%% Matlab Functions
% EKF
x_newEKF_Matlab = zeros(size(IMU_data,1),1); x_newEKF_Matlab(1) = x_teo(1);
y_newEKF_Matlab = zeros(size(IMU_data,1),1); y_newEKF_Matlab(1) = y_teo(1);
theta_newEKF_Matlab = zeros(size(IMU_data,1),1); theta_newEKF_Matlab(1) = theta_teo(1);
xunc = .01; % 
yunc = .01; % 

QEKF_Matlab = eye(2).*0.01;
REKF_Matlab = eye(3).*0.0001;
PEKF_Matlab = [xunc^2 0 0 ; 0 xunc^2 0 ;0 0 (xunc*0.01)^2].*0.001;

x_pred = [x_newEKF_Matlab(1);y_newEKF_Matlab(1);theta_newEKF_Matlab(1)];
    filter = trackingEKF('State', x_pred, ...
        'StateCovariance', PEKF_Matlab, ...
        'MeasurementNoise', QEKF_Matlab, ...
        'StateTransitionFcn', @State_Model, ...
        'MeasurementFcn', @measurement_Model, ...
        'StateTransitionJacobianFcn', @state_jacobian, ...
        'MeasurementJacobianFcn', @mea_jacobian,...
        'ProcessNoise', REKF_Matlab);
% UKF
x_newUKF_Matlab = zeros(size(IMU_data,1),1); x_newUKF_Matlab(1) = x_teo(1);
y_newUKF_Matlab = zeros(size(IMU_data,1),1); y_newUKF_Matlab(1) = y_teo(1);
theta_newUKF_Matlab = zeros(size(IMU_data,1),1); theta_newUKF_Matlab(1) = theta_teo(1);
xunc = .01; % 
yunc = .01; % 

QUKF_Matlab = eye(2).*0.001;
RUKF_Matlab = eye(3).*0.00001;
PUKF_Matlab = [xunc^2 0 0 ; 0 xunc^2 0 ;0 0 (xunc*0.01)^2].*0.001;


x_predUKF = [x_newUKF_Matlab(1);y_newUKF_Matlab(1);theta_newUKF_Matlab(1)];
    filterUKF = trackingUKF('State', x_predUKF, ...
        'StateCovariance', PUKF_Matlab, ...
        'MeasurementNoise', QUKF_Matlab, ...
        'StateTransitionFcn', @State_Model, ...
        'MeasurementFcn', @measurement_Model, ...
        'Alpha',1e-3,...
        'Beta',2,...
        'Kappa',3,...
        'ProcessNoise', RUKF_Matlab);



for i = 2:size(x_teo,2)

    %% Prediction Phase
    %% Process State
    [xpred, Ppred] = predict(filter,[x_teo(i);y_teo(i);theta_teo(i)],[x_teo(i-1);y_teo(i-1);theta_teo(i-1)],IMU_data(i,3));  
    x_newEKF_Matlab(i) = xpred(1);
    y_newEKF_Matlab(i) = xpred(2);
    theta_newEKF_Matlab(i) = xpred(3);
   [xpred, Ppred] = predict(filterUKF,[x_teo(i);y_teo(i);theta_teo(i)],[x_teo(i-1);y_teo(i-1);theta_teo(i-1)],IMU_data(i,3));  
    x_newUKF_Matlab(i) = xpred(1);
    y_newUKF_Matlab(i) = xpred(2);
    theta_newUKF_Matlab(i) = xpred(3);
    Norma = norm([x_teo(i) y_teo(i)]);

    y_theory = [norm([x_teo(i) y_teo(i)]);atan2((y_teo(i)-y_teo(i-1)),(x_teo(i)-x_teo(i-1)))];

    [xpred, Ppred] = correct(filter,y_theory,[x_newEKF_Matlab(i-1);y_newEKF_Matlab(i-1);theta_newEKF_Matlab(i-1)]);
    x_newEKF_Matlab(i) = xpred(1);
    y_newEKF_Matlab(i) = xpred(2);
    theta_newEKF_Matlab(i) = xpred(3);
    
    [xpred, Ppred] = correct(filterUKF,y_theory,[x_newUKF_Matlab(i-1);y_newUKF_Matlab(i-1);theta_newUKF_Matlab(i-1)]);
    x_newUKF_Matlab(i) = xpred(1);
    y_newUKF_Matlab(i) = xpred(2);
    theta_newUKF_Matlab(i) = xpred(3);
end


%% Plots


figure()
hold on;
plot(x_teo,y_teo);
hold on;
plot(IMU_data(:,1),IMU_data(:,2));
hold on;
plot(x_new,y_new);
hold on;
plot(x_new_uEKF,y_new_uEKF);
hold on;
plot(x_newEKF_Matlab,y_newEKF_Matlab);
hold on;
plot(x_newUKF_Matlab,y_newUKF_Matlab);

legend('True Course','Odometry','EKF Based','UKF Based','EKF From Matlab Based','UKF From Matlab Based');
legend show;


%% Auxiliar function

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


% Rafael, arranja isto que nso se percebe nada!
function [x_vec,y_vec,theta_vec,phi_vec] = testingrobot(x,y,theta)

t = 0;

v = 0.5;
phi = 0;
w_phi = 0.01;

dx = cos(theta)*0.02;
dy = sin(theta)*0.02;

figure
plot([x,x+dx],[y,y+dy],'b');
hold on;
plot(x,y,'O');

while t < 50
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
       
end


end

