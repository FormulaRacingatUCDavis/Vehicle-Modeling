function [] = springrate_cal(wkf,diff,m_total,front_weight_distribute)

fw = 0.01*front_weight_distribute;
rw = 0.01*abs(100-front_weight_distribute);
kf = (wkf^2)*(0.5*m_total*fw/9.8/1000);
diff_percent = 0.01*(100+diff);
wkr = wkf/diff_percent;
kr = (wkr^2)*(0.5*m_total*rw/9.8/1000);

x = ['The front spring rate is ', num2str(kf), 'kg/mm'];
y = ['The rear spring rate is ', num2str(kr), 'kg/mm'];

disp(x)
disp(y)
