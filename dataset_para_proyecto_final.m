% Parámetros
m = 0.25;
l = 0.2;
g = 9.81;

Kp = 2;
Ki = 0.3;
Kd = 0.2;

% Tiempo
dt = 0.01;
T = 35;
t = 0:dt:T;

% Estado inicial
theta = 3.14;
omega = 0.0;

% PID interno
int_error = 0;
prev_error = 0;

% Dataset
data = [];

for i = 1:length(t)
    % Error
    % error = 0 - theta;  % setpoint = 0
    error = atan2(sin(0 - theta), cos(0 - theta));
    int_error = int_error + error * dt;
    der_error = (error - prev_error) / dt;
    
    % PID
    tau = Kp*error + Ki*int_error + Kd*der_error;
    

    % PERTURBACIÓN EXTERNA (elige una)
    tau_disturb = 0;
    if t(i) > 14 && t(i) < 14.1
        tau_disturb = -0.5;
    end
    if t(i) > 16 && t(i) < 16.1
        tau_disturb = -0.55;
    end
    if t(i) > 18 && t(i) < 18.1
        tau_disturb = -0.58;
    end
    if t(i) > 22 && t(i) < 22.1
        tau_disturb = 0.5;
    end
    if t(i) > 24 && t(i) < 24.1
        tau_disturb = 0.55;
    end
    if t(i) > 26 && t(i) < 26.1
        tau_disturb = 0.58;
    end
    if t(i) > 28 && t(i) < 28.1
        tau_disturb = 0.7;
    end
    if t(i) > 30 && t(i) < 30.1
        tau_disturb = -0.6;
    end
    % if t(i) > 8 && t(i) < 8.1
    %     tau_disturb = 1.5;
    % end
    % if t(i) > 10 && t(i) < 10.1
    %     tau_disturb = -1.5;
    % end
    % if t(i) > 12 && t(i) < 12.1
    %     tau_disturb = 2;
    % end
    % if t(i) > 14 && t(i) < 14.1
    %     tau_disturb = -2;
    % end

    tau = max(min(tau, 0.3), -0.3);  % limitar a ±5 N·m

    % Torque total
    tau_total = tau + tau_disturb;

    % Dinámica del sistema
    dtheta = omega;
    domega = (g/l)*sin(theta) + (1/(m*(l^2)))*tau_total;
    
    % Integrar (Euler)
    theta = theta + dtheta * dt;
    theta = atan2(sin(theta), cos(theta));
    omega = omega + domega * dt;
    
    %Utilizar el camino corto en caso de pasar el angulo 2pi
    % theta = atan2(sin(theta), cos(theta));


    % Guardar datos
    data(end+1, :) = [theta, omega, tau];
    
    % Actualizar error
    prev_error = error;
end

% Guardar dataset
writematrix(data, 'dataset.csv')
