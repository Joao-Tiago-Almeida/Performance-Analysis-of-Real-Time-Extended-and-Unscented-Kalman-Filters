function [x,y,theta, phi] = robot_simulation(x_k, y_k, theta_k, v, phi_k, w_phi)
    L = 2.2;
    dx = v*cos(theta_k)*0.1;
    dy = v*sin(theta_k)*0.1;
    x = x_k+dx;
    y = y_k+dy;
    dphi = w_phi;
    phi = phi_k+w_phi;
    dtheta = (v/L)*tan(abs(phi_k));
    theta = theta_k+dtheta;
end