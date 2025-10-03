function plot3DMMD(result)
    dSteer = result.grid.dSteer;
    SA_CG  = result.grid.SA_CG;
    MzBody = result.MzBody;
    CAxVel = result.CAxVel;
    CAyVel = result.CAyVel;


    figure
    hold on
    grid on
    
    for i = 1:length(dSteer)
    
        steer = plot3(MzBody(i,:),CAyVel(i,:),CAxVel(i,:), "Color", "blue", 'LineStyle','--');
        % if mod(i,7) == 0
        %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg steer'];  % Dynamic label for clarity
        %     % text(saveCAyVel(i,end), saveMzBody(i,end), labelText)
        % end
    
    end
    
    for i = 1:length(SA_CG)
        slip = plot3(MzBody(:,i),CAyVel(:,i),CAxVel(:,i), "Color", "red", 'LineStyle','--');
        % if mod(i,7) == 0
        %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg SA'];  % Dynamic label for clarity
        %     % text(saveCAyVel(1,i), saveMzBody(1,i), labelText)
        % end
    end
    
    zlim([-2, 2])
    
    view(3)
    ylabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
    xlabel("Normalized Yaw Moment $(C_{Mz})$",'Interpreter','latex')
    zlabel("Normalized Longitudinal Accleration $(C_{Ax})$",'Interpreter','latex')
end

