function [] = wheelrate_cal(kf,kr,m_total,front_weight_distribute)

fw = 0.01*front_weight_distribute;
rw = 0.01*abs(100-front_weight_distribute);

wheelrate_front = sqrt(kf*1000*9.8/(m_total*0.5*fw));
wheelrate_rear = sqrt(kr*1000*9.8/(m_total*0.5*rw));
rt = wheelrate_front/wheelrate_rear;

x = ['The front wheel rate is ', num2str(wheelrate_front), 'rad/sec'];
y = ['The rear wheel rate is ', num2str(wheelrate_rear), 'rad/sec'];
z = ['The wheel rate different is ', num2str(rt)];
z1= ['(note: greater than 1 means front rate is higher)' ];
disp(x)
disp(y)
disp(z)
disp(z1)



