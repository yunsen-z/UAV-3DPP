function plotFigure(startPos,goalPos,X,Y,Z,path)
    % 绘制起点和终点
    scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y')
    hold on
    scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y')
    
    % 绘制地形
    surf(X,Y,Z)
    shading flat
    
    % 绘制路径
    plot3(path(:,1), path(:,2), path(:,3), 'r','LineWidth',2);
    
    hold off
    grid on
end