%% 初始化
clc
clear
close all
tic

%% 随机地图的随机数种子设置
rng(2025)

%% 三维地图模型
startPos = [1, 1, 1]; % 起点
goalPos = [200, 180, 150]; % 终点
mapRange = [0,200;0,200;0,200;]; % 山峰地图的大小

% 地图实例化
[X,Y,Z] = defMap(mapRange);

%% 构造三维空间用于路径规划的切片结构体
sliceNum = 19; % 中间路程点的栅格划分,将起始点放在单独的栅格中
perSliceH = round((goalPos(3)-startPos(3)+1)/sliceNum);
slice = struct;
slice.allowedPos = []; % 每一层切片的允许访问栅格
slice = repmat(slice,sliceNum,1);

intrp_num =100;

% 获得每一个切片允许访问的栅格
for i = 1:sliceNum+1
    if i == 1
        slice(i).allowedPos = startPos;
    elseif i == sliceNum+1
        slice(i).allowedPos = goalPos;
    else
        h = startPos(2)+(i-1)*perSliceH; % 切片高度
        for x = X(1,1):perSliceH:X(1,end)
            for y = Y(1,1):perSliceH:Y(end,1)
                if h > Z(x,y)
                    slice(i).allowedPos(end+1,:) = [x,y,h];
                end
            end
        end
    end
end

%% A*算法实现
% 定义启发式函数（三次样条插值的点距离和）
heuristic = @(pos1, pos2) sqrt(sum((pos1 - pos2).^2));
% calFitness

% 初始化开放列表和关闭列表
openList = [];
closedList = [];

% 初始化起点
startNode.pos = startPos;
startNode.g = 0;
startNode.h = heuristic(startPos, goalPos);
startNode.f = startNode.g + startNode.h;
startNode.parent = [];
openList = [openList; startNode];

% A*算法主循环
while ~isempty(openList)
    % 从开放列表中找到f值最小的节点
    [~, idx] = min([openList.f]);
    currentNode = openList(idx);
    path_show = reconstructPath(currentNode);
    if size(path_show,1)>=4
        [~,~,path_show] = calH(X,Y,Z, path_show);
        plotFigure(startPos,goalPos,X,Y,Z,path_show);
        pause(0.001);
    end
    % 如果当前节点是目标节点，则路径找到
    if isequal(currentNode.pos, goalPos)
        path = reconstructPath(currentNode);
        break;
    end
    
    % 将当前节点从开放列表移到关闭列表
    openList(idx) = [];
    closedList = [closedList; currentNode];
    
    % 获取当前节点的所有邻居节点
    neighbors = getNeighbors(currentNode, slice, sliceNum,goalPos);
    disp(['迭代到点' num2str(neighbors(1).pos)]);
    
    for i = 1:length(neighbors)
        neighbor = neighbors(i);
        
        % 如果邻居节点在关闭列表中，则跳过
        if any(ismember(vertcat(closedList.pos), neighbor.pos, 'rows'))
            continue;
        end

        % 计算邻居节点的g值
        [flag,neighbor_g,path]=calH(X,Y,Z, [reconstructPath(currentNode); neighbor.pos]);
        % tentative_g = currentNode.g + heuristic(currentNode.pos, neighbor.pos);
        if flag==0
            tentative_g = neighbor_g;
        else
            tentative_g = 1000*neighbor_g;
        end
        % 如果邻居节点不在开放列表中，或者新的g值更小
        if isempty(vertcat(openList.pos)) || ~any(ismember(vertcat(openList.pos), neighbor.pos, 'rows')) || tentative_g < neighbor.g
            neighbor.g = tentative_g;
            neighbor.h = heuristic(neighbor.pos, goalPos);
            neighbor.f = neighbor.g + neighbor.h;
            neighbor.parent = currentNode;
            
            % 如果邻居节点不在开放列表中，则加入开放列表
            if isempty(vertcat(openList.pos)) || ~any(ismember(vertcat(openList.pos), neighbor.pos, 'rows'))
                openList = [openList; neighbor];
            end
        end
    end
end

%% 结果展示
% 画适应度迭代图
[flag,fitness,path] = calFitness(X,Y,Z, path,intrp_num);
disp(['得到的Fitness' num2str(fitness)]);
plotFigure(startPos,goalPos,X,Y,Z,path);
toc