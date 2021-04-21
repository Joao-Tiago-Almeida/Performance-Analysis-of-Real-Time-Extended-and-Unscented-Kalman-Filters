close all;
clear

%% Colado a Cuspo
% ERRO - è necessário ter a aceleração ou orientação teórica
% Está mal porque o documento não tem aceleração
% Measurements - Distância à origem(landmark), Odometria

[t,xa] = ode45(@(t,x) odefcn(t,x),[0 1],[0; 0.5; 0.1; 0; 0.5; 0.1]');

odomx = awgn(xa(:,1),15/0.001,'measured','linear');
odomy = awgn(xa(:,4),15/0.001,'measured','linear');
% Initial Conditions 

Estimated_x = zeros(size(odomx,1),1); Estimated_x(1) = xa(1,1);
Estimated_y = zeros(size(odomy,1),1); Estimated_y(1) = xa(1,4);
Estimated_vx = zeros(size(odomx,1),1); Estimated_vx(1) =  xa(1,2); 
Estimated_vy = zeros(size(odomy,1),1); Estimated_vy(1) = xa(1,5);
true_ax = zeros(size(odomx,1),1); true_ax(1) = xa(1,3);
true_ay = zeros(size(odomy,1),1);true_ay(1) = xa(1,6);
% 
% [true_ax,Estimated_vx] = Get_prop(x_true);
% [true_ay,Estimated_vy] = Get_prop(y_true);


% Satae x.xdot.xdotdot,...
% Depois meter a formula do process noise como está lá
xunc = .01; % 
yunc = .01; % 

Q = eye(6).*0.01;
P = [xunc^2 0 0 0 0 0; 0 xunc^2 0 0 0 0;0 0 xunc^2 0 0 0;0 0 0 yunc^2 0 0;0 0 0 0 yunc^2 0;0 0 0 0 0 yunc^2];

% Measurement- distancia, odomx e odomy


for i = 2:(size(odomx,1)-3)
    T = t(i);

    % Jacobian motion model
    F = [[1 T 0.5*T^2 0 0 0]
         [0 1 T 0 0 0]
         [0 0 1 0 0 0]
         [0 0 0 1 T 0.5*T^2]
         [0 0 0 0 1 T ]
         [0 0 0 0 0 1 ]];

    x_pos = F*[Estimated_x(i-1);Estimated_vx(i-1);true_ax(i-1);Estimated_y(i-1);Estimated_vy(i-1);true_ay(i-1)];

    Estimated_x(i) = x_pos(1);
    Estimated_y(i) = x_pos(4);
    Estimated_vx(i) = x_pos(2); 
    Estimated_vy(i) = x_pos(5);
%     true_ax(i) = x_pos(3);
%     true_ay(i) = x_pos(6);
    
    Norma = norm([Estimated_x(i) Estimated_y(i)]);
    
    H = [((Estimated_x(i))/Norma) 0 0 ((Estimated_y(i))/Norma) 0 0; ...
         1 0 0 0 0 0;
         0 0 0 1 0 0];

   y_hat = [norm([Estimated_x(i) Estimated_y(i)]); ...
        Estimated_x(i);
        Estimated_y(i)];
   y1 = [ norm([xa(i,1) xa(i,4) ]); xa(i,1); ...
         xa(i,4)];
    y = (y1 - y_hat);
   P = F*P*F';
   %% Update Stage
   K = P*H'/(H*P*H' + H*Q*H');
   aux =  x_pos + K*y;
   P = (eye(size(K,1))-K*H)*P;
   
    Estimated_x(i) = aux(1);
    Estimated_y(i) = aux(4);
    Estimated_vx(i) = aux(2); 
    Estimated_vy(i) = aux(5);
%     true_ax(i) = aux(3);
%     true_ay(i) = aux(6);

end







% 
figure();
plot(xa(:,1),xa(:,4));
hold on;
plot(odomx,odomy);
hold on;
plot(Estimated_x,Estimated_y);
legend('True Course','Odometry','EKF Based');
legend show;

function F = odefcn(t,x)

F = [x(1)+t*x(2) + 0.5*t^2*x(3);
     x(2) + t*x(3);
     x(3);
     x(4)+t*x(5) + 0.5*t^2*x(6);
     x(5) + t*x(6);
     x(6)];
 
end
%% Teste



% 
% function [acc,v] = Get_prop(position)
%     acc = zeros(size(position,1),1);
%     v = zeros(size(position,1),1);
%     for i = 2:size(position,1)
%         v(i-1) = (position(i) -position(i-1))/0.1;
%         if i > 2
%            acc(i-1) =(v(i) -v(i-1))/0.1;
%         end
%         
%         
%     end
% 
% 
% 
% end