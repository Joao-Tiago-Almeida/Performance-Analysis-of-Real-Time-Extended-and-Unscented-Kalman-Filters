function F = state_jacobian(x,x_teoretical,x_teoretical_old,IMU_theta)
    x_teo = x_teoretical(1); y_teo=x_teoretical(2);
    x_teo_old = x_teoretical_old(1); y_teo_old=x_teoretical_old(2); theta_new_old = x_teoretical_old(3);
    

    Norma = norm([x_teo y_teo]-[x_teo_old y_teo_old]);

    F = [1 0 -Norma*sin(x(3));...
         0 1 Norma*cos(x(3)); ...
         0 0 1];

    
end