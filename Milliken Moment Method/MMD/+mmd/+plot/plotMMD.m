function plotMMD(result, SSAy)
    SA_CG  = result.grid.SA_CG;
    dSteer = result.grid.dSteer;
    CAyVel = result.CAyVel;
    MzBody = result.MzBody;

    figure
    hold on
    grid on
    for i = 1:length(dSteer)
        steer = plot(CAyVel(i,:),MzBody(i,:), "Color", "blue", 'LineStyle','--');
        if dSteer(i) == 0
            plot_zeroSteer = plot(CAyVel(i,:),MzBody(i,:), "Color", "blue", 'LineStyle','-');
        end
    end
    
    for i = 1:length(SA_CG)
        slip = plot(CAyVel(:,i),MzBody(:,i), "Color", "red", 'LineStyle', '--');
        if SA_CG(i) == 0
            plot_zeroSA = plot(CAyVel(:,i),MzBody(:,i), "Color", "red", "LineStyle", '-');
        end
    end
    
    xlabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
    ylabel("Normalized Yaw Moment $(C_{Mz})$",'Interpreter','latex')
    xlim([-2,2])
    ylim([-1,1])
    
    title({'Free Rolling MMD: Constant Velocity' ...
           ['Velocity = ', num2str(V),' m/s'] ...
           ['Minimum Radius = ' num2str(V.^2./(SSFy.CAy.*9.81)) ' m']...
          },'Interpreter','latex')
    
    
    %%% Maximum Ay Plotting Stuff
    if nargin == 2
        AyMaxSS = plot(SSAy.CAy, 0, "Marker", ".", "MarkerSize", 20, "Color","g");
        label = sprintf(['C0   = %.4g\n',...
                         'SA      = %.3g°\n',...
                         'Steer  = %.3g°'],...
                        SSAy.CAy, SSAy.SA, SSAy.dSteer);
        
        text(SSAy.CAy - 0.07, 0.20, label, "FontSize", 8, 'FontWeight','bold', 'Interpreter', 'latex');
        
        legend([steer, slip, AyMaxSS], ...
                {"Constant Steer", ...
                 "Constant Slip", ...
                 "$C_{Ay_{Max SS}}$"},...
                 "Location","northeast",'Interpreter','latex')
    end
end
