function H = mea_jacobian(x,x_old)
    x_new = x(1);y_new=x(2);
    x_new_old = x_old(1);y_new_old=x_old(2);

    Norma =  norm([x_new y_new]);

    H = [((x_new )/Norma) ((y_new )/Norma) 0; ...
          ((-(y_new -y_new_old))/((x_new -x_new_old)^2 + (y_new -y_new_old)^2)) ((x_new -x_new_old)/((x_new -x_new_old)^2 + (y_new -y_new_old)^2)) 0];

    
end

