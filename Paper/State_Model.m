function [xd] = State_Model(x,x_teoretical,x_teoretical_old,IMU_theta)

    x_teo = x_teoretical(1);y_teo = x_teoretical(2);
    x_teo_old = x_teoretical_old(1);y_teo_old = x_teoretical_old(2);
    %%%%%%%%%%%%% State Process %%%%%%%%%%%%%
    
    Norma = norm([x_teo y_teo]-[x_teo_old y_teo_old]);
    xd(1) = x(1) + Norma*cos(x(3));
    xd(2) = x(2) + Norma*sin(x(3));
    xd(3) = IMU_theta;
    
    xd = xd';
    
end