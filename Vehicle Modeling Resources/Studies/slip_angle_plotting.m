clear; clc; close all;

carParams = Cars.FE13();

mmd = MMD.MMD(carParams);

n = 10;
v = linspace(10, 20, n);
mesh.SA_CG  = -10:0.5:1;
mesh.dSteer = -30:30;

body_slip = zeros(1, n);
steer_angle = zeros(1, n);
ay = zeros(1, n);
Omega = zeros(1, n);

for i=1:n
    mesh.V = v(i);
    results = mmd.evaluate(mesh, "free_rolling");
    SS = MMD.analysis.getSteadyStateCAy(results);
    body_slip(i) = SS.SA;
    steer_angle(i) = SS.dSteer;
    ay(i) = SS.CAy;
    Omega(i) = SS.Omega;
end

%%
coord_AllW = calcWheelCoords(carParams);
Steer_AllW_rad = MMD.models.steeringModel_default(carParams, deg2rad(steer_angle'));
VxCurr = v;
VyCurr = v .* tand(body_slip);

for p=1:4
SA_Wheel(:, p) = rad2deg(atan2( (VyCurr + Omega .* coord_AllW(1,p)) ...
                                , (VxCurr - Omega .* coord_AllW(2,p)) ) - Steer_AllW_rad(p));
end

figure;
hold on
grid on
plot(v, body_slip)
plot(v, steer_angle)
plot(v, SA_Wheel(:, 1))
plot(v, SA_Wheel(:, 2))
plot(v, SA_Wheel(:, 3))
plot(v, SA_Wheel(:, 4))

xlabel("velocity (m/s)")
ylabel("slip angle (deg)")
legend(["body slip", "steer angle", "FR", "FL", "RR", "RL"])

figure;
hold on
grid on
plot(ay, body_slip)
plot(ay, steer_angle)
plot(ay, SA_Wheel(:, 1))
plot(ay, SA_Wheel(:, 2))
plot(ay, SA_Wheel(:, 3))
plot(ay, SA_Wheel(:, 4))

xlabel("ay (g)")
ylabel("slip angle (deg)")
legend(["body slip", "steer angle", "FR", "FL", "RR", "RL"])