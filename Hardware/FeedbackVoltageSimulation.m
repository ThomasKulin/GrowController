clc;
clear;  

%         Vout
%         R2
%         |
% ARD R1--+--- FB_PIN
%         |
%         R3
%         GND

v_ctl = 0.8; % voltage maintained on the feedback pin of LM5117 by control loop
R1 = 17000:500:17500;
R2 = 178000;
R3 = 10000;
v_fb = 0:0.5:5;

% v_out = (v_ctl*(R2*R3 + R1*R3 + R1*R2) - v_fb*R2*R3)/(R1*R3);
vout = zeros(length(R1),length(v_fb));
for i = 1:1:length(R1)
    for j = 1:1:length(v_fb)
        v_out(i,j) = (v_ctl*(R2*R3 + R1(i)*R3 + R1(i)*R2) - v_fb(j)*R2*R3)/(R1(i)*R3);
    end
end

surf(v_fb, R1, v_out);