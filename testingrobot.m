global start_v
start_v = 0;
my_timer = timer('Name', 'my_timer', 'ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                    'StartFcn', @(x,y)disp('started...'), ...
                    'StopFcn', @(x,y)disp('stopped ...'), ...
                    'TimerFcn', @my_start_fcn);
x = 0;
y = 0;
t = 0;
theta = pi/4;
x_old = 0;
y_old = 0;
v = 0.5;
phi = 0;
w_phi = 0.01;
dx = cos(theta)*0.002;
dy = sin(theta)*0.002;
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
        t = t + 1;
        start_v = 0;
    end
end

function my_start_fcn(obj, event)
    global start_v
    start_v = 1;
end