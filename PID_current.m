clear()
% 
%  s = 2630         1          1
%     ------   = ------ = -----------------  => R =4758/2630, L= 1/2630
%     s + 4758   sL + R   s/2630 + 4758/2630

%%% System Parameters %%%
R =  4758/2630; % Resistance in Ohms
L = 1/2630; % Inductance in Henries
Ts = 1/8000; % Sample period
wc = pi/10; % Crossover frequency, in Radians per sample
%%%
s = tf('s');
sys = 1/(L*s + R); % Continuous time transfer function
z = tf('z', Ts);
sys_d = c2d(sys, Ts); % Zero order hold equivalent
ki = 1-exp(-R*Ts/L) % Calculate Ki
k = R*((wc)/(1-exp(-R*Ts/L))) % Calculate loop gain
controller = k*(1 + ki/((z-1))); % PI controller transfer function
fp = series(controller, sys_d); % Forward Path
cl = feedback(fp, 1); % Unity feedback
%%% Plot open-loop, return ratio, and controller bode plots %%%
figure;
hold all
bode(controller);
bode(sys_d);
margin(fp);
legend('Controller', 'Plant', 'Return Ratio')
%%% Plot closed-loop step response %%%
figure;step(cl);

figure;bode(controller);margin(controller);
figure;bode(sys_d);margin(sys_d);
figure;bode(fp);margin(fp);

% ki = 0.4483, k = 1.2678
% Kii  = 3,586.4
% Kpp = 1.2678
% controller2 = Kpp*(1 + Kii*Ts/((z-1)));
% fp2 = series(controller2, sys_d);
% closed_loop_system = feedback(fp2, 1);
% figure;step(closed_loop_system);