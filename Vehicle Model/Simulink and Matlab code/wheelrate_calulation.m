function [] = wheelrate_calculation()


prompt = 'Please enter in the front spring rate (kg/mm)\n';
kf = input(prompt);
prompt = 'Please enter in the rear spring rate (kg/mm)\n';
kr =  input(prompt);
prompt = 'Please enter in the total weight of the car (kg)\n';
m_total = input(prompt);
prompt = 'Please enter in the front weight distribution (ex: for front 58%, type 58)\n';
front_weight_distribute = input(prompt);
wheelrate_cal(kf,kr,m_total,front_weight_distribute)