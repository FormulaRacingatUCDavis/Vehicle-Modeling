function [] = springrate_calculation()

prompt = 'Please enter in the front wheel rate (rad/sec)\n';
wkf = input(prompt);
prompt = 'Please enter in the wheel rate different \n (ex: for front > rear by 10%, type 10 ; fpr front < rear by 10%, type -10)\n';
diff =  input(prompt);
prompt = 'Please enter in the total weight of the car (kg)\n';
m_total = input(prompt);
prompt = 'Please enter in the front weight distribution (ex: for front 58%, type 58)\n';
front_weight_distribute = input(prompt);
springrate_cal(wkf,diff,m_total,front_weight_distribute)