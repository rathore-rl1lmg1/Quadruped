
close all;
clear all;
a1 = 10;
a2 = 10;
l((250),(12)) = zeros;
q((1),(12)) = zeros;


m=0;
 q_lb = [pi/3; pi/3];
 q_lf = [pi/3; pi/3];
 q_rb = [pi/3; pi/3];
 q_rf = [pi/3; pi/3];

% ___________initializing left back_______________
x_lb = -2;
y_lb = -16;
% ___________initializing left front_______________
x_lf = +2;
y_lf = -16;
% ___________initializing right back_______________
x_rb = -2;
y_rb = -16;
% ___________initializing right front_______________
x_rf = +2;
y_rf = -16;

%_________________initializing mu_a________________
mu_a_lb = [x_lb; y_lb];
mu_a_lf = [x_lf; y_lf];
mu_a_rb = [x_rb; y_rb];
mu_a_rf = [x_rf; y_rf];
%__________________initializing t_________________
t=0;

for i = 1:250 
    if i <= 50                  %__________left back_____________
         x_lb = (i*pi/50)/2 - 2;
         y_lb = sin(2*(x_lb+2)) - 16;
         
    elseif i >= 50 && i <= 100 %___________left front_____________
         x_lf = ((i-50)*pi/50)/2+2;
         y_lf = sin(2*(x_lf-2))-16;
         
    elseif i >= 100 && i <= 150 %__________right front______________
         x_rb = ((i-100)*pi/50)/2 - 2;
         y_rb = sin(2*(x_rb+2)) - 16;
         
    elseif i >= 150 && i <= 200 %____________right back_____________
         x_rf = ((i-150)*pi/50)/2 + 2;
         y_rf = sin(2*(x_rf-2)) - 16; 

    elseif i >= 200
         x_lb = 2.5*pi - 2 - i*pi/100;
         x_lf = 2.5*pi + 2 - i*pi/100;
         x_rb = 2.5*pi - 2 - i*pi/100;
         x_rf = 2.5*pi + 2 - i*pi/100;
         y_lb = -16;
         y_lf = -16;
         y_rb = -16;
         y_rf = -16;

    end     
    mu_a_lb = [x_lb; y_lb];
    mu_a_lf = [x_lf; y_lf];
    mu_a_rb = [x_rb; y_rb];
    mu_a_rf = [x_rf; y_rf];
    for j=1:10 %__________________inverse kinemati equation _______________
       
        th1_lb = q_lb(1);
        th2_lb = q_lb(2);

        th1_lf = q_lf(1);
        th2_lf = q_lf(2);

        th1_rb = q_rb(1);
        th2_rb = q_rb(2);

        th1_rf = q_rf(1);
        th2_rf = q_rf(2);

        q_lb = [th1_lb; th2_lb];
        q_lf = [th1_lf; th2_lf];
        q_rb = [th1_rb; th2_rb];
        q_rf = [th1_rf; th2_rf];

        %_______________initializing mu_e_________________________________
        mu_e_lb = [a1*cos(th1_lb) + a2*cos(th1_lb + th2_lb) - 2 ; a1*sin(th1_lb) + a2*sin(th1_lb + th2_lb) ];
        mu_e_lf = [a1*cos(th1_lf) + a2*cos(th1_lf + th2_lf) + 2 ; a1*sin(th1_lf) + a2*sin(th1_lf + th2_lf) ];
        mu_e_rb = [a1*cos(th1_rb) + a2*cos(th1_rb + th2_rb) - 2 ; a1*sin(th1_rb) + a2*sin(th1_rb + th2_rb) ];
        mu_e_rf = [a1*cos(th1_rf) + a2*cos(th1_rf + th2_rf) + 2 ; a1*sin(th1_rf) + a2*sin(th1_rf + th2_rf) ];
        %_______________initializing error______________________________
        delta_mu_lb = mu_a_lb - mu_e_lb;
        delta_mu_lf = mu_a_lf - mu_e_lf;
        delta_mu_rb = mu_a_rb - mu_e_rb;
        delta_mu_rf = mu_a_rf - mu_e_rf;
        delta_mu = [delta_mu_lb; delta_mu_lf; delta_mu_rb; delta_mu_rf];

        if abs(delta_mu) < 1e-5
            break;
       
        end    

        J_lb = [-a2*sin(th2_lb+th1_lb)-a1*sin(th1_lb) -a2*sin(th1_lb+th2_lb) ; a2*cos(th1_lb+th2_lb)+a1*cos(th1_lb) a2*cos(th1_lb+th2_lb) ];
        J_lf = [-a2*sin(th2_lf+th1_lf)-a1*sin(th1_lf) -a2*sin(th1_lf+th2_lf) ; a2*cos(th1_lf+th2_lf)+a1*cos(th1_lf) a2*cos(th1_lf+th2_lf) ];
        J_rb = [-a2*sin(th2_rb+th1_rb)-a1*sin(th1_rb) -a2*sin(th1_rb+th2_rb) ; a2*cos(th1_rb+th2_rb)+a1*cos(th1_rb) a2*cos(th1_rb+th2_rb) ];
        J_rf = [-a2*sin(th2_rf+th1_rf)-a1*sin(th1_rf) -a2*sin(th1_rf+th2_rf) ; a2*cos(th1_rf+th2_rf)+a1*cos(th1_rf) a2*cos(th1_rf+th2_rf) ];
        q_lb = q_lb + inv(J_lb)*delta_mu_lb;
        q_lf = q_lf + inv(J_lf)*delta_mu_lf;
        q_rb = q_rb + inv(J_rb)*delta_mu_rb;
        q_rf = q_rf + inv(J_rf)*delta_mu_rf;
        pause(0.001)
        q_lb = wrapToPi(q_lb);
        q_lf = wrapToPi(q_lf);
        q_rb = wrapToPi(q_rb);
        q_rf = wrapToPi(q_rf);
    

    end
    
    l(i,  1) = 0;
    l(i,  2) = q_lf(1);
    l(i,  3) = q_lf(2);
    l(i,  4) = 0;
    l(i,  5) = q_rb(1);
    l(i,  6) = q_rb(2);
    l(i,  7) = 0;
    l(i,  8) = q_rf(1);
    l(i,  9) = q_rf(2);
    l(i, 10) = 0;
    l(i, 11) = q_lb(1);
    l(i, 12) = q_lb(2);
    plot([-2 a1*cos(th1_lb)-2  mu_e_lb(1)] , [0 a1*sin(th1_lb) mu_e_lb(2)],'b--o')
    hold on
    plot([2  a1*cos(th1_lf)+2  mu_e_lf(1)] , [0 a1*sin(th1_lf) mu_e_lf(2)],'r--o')
    hold on
    plot([-2 a1*cos(th1_rb)-2  mu_e_rb(1)] , [0 a1*sin(th1_rb) mu_e_rb(2)],'b-o')
    hold on
    plot([2  a1*cos(th1_rf)+2  mu_e_rf(1)] , [0 a1*sin(th1_rf) mu_e_rf(2)],'r-o')
    hold on
    plot([-2 2],[0 0],'-')
    axis([-(a1+a2) (a1+a2) -(a1+a2) (a1+a2)])
    grid on
    hold off
    pause(0.01)
end

l = clip(l , -pi , pi); 
l = l*180/pi;
disp(l)