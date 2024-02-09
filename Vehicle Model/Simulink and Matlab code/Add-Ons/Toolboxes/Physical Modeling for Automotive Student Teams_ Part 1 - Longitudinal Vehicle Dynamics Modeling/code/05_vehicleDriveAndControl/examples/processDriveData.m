%% Process Data
% Set 0 for values < 0
FrontBrakePedal(FrontBrakePedal < 0) = 0;
RearBrakePedal(RearBrakePedal < 0) = 0;

%% Plot data
subplot(3,1,1)
plot(PedalTime,ThrottlePedal)
xlabel('Time(s)')
ylabel('Throttle')
subplot(3,1,2)
plot(PedalTime,FrontBrakePedal)
xlabel('Time(s)')
ylabel('Front Brake')
subplot(3,1,3)
plot(PedalTime,RearBrakePedal)
xlabel('Time(s)')
ylabel('Rear Brake')
%Copyright 2015 The MathWorks, Inc.