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
[X,Y,Z] = Astar.defMap(mapRange);

%% 构造三维空间用于路径规划的切片结构体
sliceNum = 7; % 中间路程点的栅格划分,将起始点放在单独的栅格中
perSliceH = round((goalPos(3)-startPos(3)+1)/sliceNum);
slice = struct;
slice.allowedPos = [];                % 每一层切片的允许访问栅格
slice.par = [];                       % 每一层切片连接下一层切片的参数：信息素等
slice = repmat(slice,sliceNum,1);

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

% 初始化信息素和启发值
for i = 1:sliceNum-1
    for j = 1:size(slice(i).allowedPos,1)
        pathNum = size(slice(i+1).allowedPos,1);
        slice(i).par(j).tau = ones(pathNum,1);
        % deltaX = slice(i+1).allowedPos(:,1) - slice(i).allowedPos(j,1);
        % deltaY = slice(i+1).allowedPos(:,2) - slice(i).allowedPos(j,2);
        % deltaZ = slice(i+1).allowedPos(:,3) - slice(i).allowedPos(j,3);
        deltaX = slice(i+1).allowedPos(:,1) - goalPos(1);
        deltaY = slice(i+1).allowedPos(:,2) - goalPos(2);
        deltaZ = slice(i+1).allowedPos(:,3) - goalPos(3);
        slice(i).par(j).eta = 20./sqrt(deltaX.^2 + deltaY.^2 + deltaZ.^2);
    end
end

%% 定义蚁群结构体
% 蚁群相关定义
m = 10;                             % 蚂蚁数量,每个蚂蚁走自己的路
alpha = 50;                          % 信息素重要程度因子
beta = 1;                            % 启发函数重要程度因子
rho = 0.2;                           % 信息素挥发因子
Q = 1;                               % 常数
intrp_num = 120;     % 插值拟合后的路线碰撞判断点数量。
iter_max = 10;                      % 最大迭代次数

% 定义蚁群结构体
antColony = struct;
antColony.pos= [];
antColony.path = [];
antColony.fitness = [];
antColony.Best.pos = [];
antColony.Best.path = [];
antColony.Best.fitness = inf;
antColony = repmat(antColony,m,1);

% 创建最优路线记录
GlobalBest = struct;
GlobalBest.fitness=inf;
GlobalBest.pos = [];
GlobalBest.path = [];

NowBest = struct;
NowBest.fitness=inf;
NowBest.pos = [];
NowBest.path = [];


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
    path_show = Astar.reconstructPath(currentNode);
    % if size(path_show,1)>=4
    %     [~,~,path_show] = Astar.calH(X,Y,Z, path_show);
    %     Astar.plotFigure(startPos,goalPos,X,Y,Z,path_show);
    %     pause(0.001);
    % end
    % 如果当前节点是目标节点，则路径找到
    if isequal(currentNode.pos, goalPos)
        path = Astar.reconstructPath(currentNode);
        break;
    end

    % 将当前节点从开放列表移到关闭列表
    openList(idx) = [];
    closedList = [closedList; currentNode];

    % 获取当前节点的所有邻居节点
    neighbors = Astar.getNeighbors(currentNode, slice, sliceNum,goalPos);
    disp(['迭代到点' num2str(neighbors(1).pos)]);

    for i = 1:length(neighbors)
        neighbor = neighbors(i);

        % 如果邻居节点在关闭列表中，则跳过
        if any(ismember(vertcat(closedList.pos), neighbor.pos, 'rows'))
            continue;
        end

        % 计算邻居节点的g值
        [flag,neighbor_g,path]=Astar.calH(X,Y,Z, [Astar.reconstructPath(currentNode); neighbor.pos]);
        % tentative_g = currentNode.g + heuristic(currentNode.pos, neighbor.pos);
        if flag==0
            tentative_g = neighbor_g;
        else
            tentative_g = 10*neighbor_g;
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
[flag,fitness,path_show] = Astar.calFitness(X,Y,Z, path,intrp_num);
disp(['得到的Fitness' num2str(fitness)]);
Astar.plotFigure(startPos,goalPos,X,Y,Z,path_show);

% GlobalBest.pos=path;
% GlobalBest.path=path_show;
% GlobalBest.fitness=fitness;

%% 更新由A*确定的路径下的信息素
for i =1:size(path,1)-1
    for j =1:size(slice(i).allowedPos,1)
        if path(i)==slice(i).allowedPos(j)
            for k = 1:size(slice(i+1).allowedPos,1)
                if path(i+1)==slice(i+1).allowedPos(k)
                    slice(i).par(j).tau(k) = Q*1000/fitness;
                end
            end
        end
    end
end

%% 迭代寻找最佳路径
for  iter = 1:iter_max

    % 初始化蚁群路径变化信息素结构体
    deltaTau = struct;
    deltaTau.start = [0,0,0];
    deltaTau.target = [0,0,0];
    deltaTau.delat_tau = (0);
    
    % 逐个蚂蚁路径选择
    for i = 1:m
        
        % 将起始位置和目标位置存放在蚁群结构体中
        antColony(i).pos(1,:) = startPos;
        antColony(i).pos(sliceNum,:) = goalPos;
        nowPos = startPos;
        idx = 1;
        
        % 逐个切片的栅格选择
        for j = 1:sliceNum-2            
            %计算下一个节点的访问概率
            P = slice(j).par(idx).tau .^alpha .* slice(j).par(idx).eta .^ beta;
            P = P/sum(P);
            
            % 轮盘赌法选择下一个访问节点
            Pc = cumsum(P);
            Pc = [0; Pc];
            randnum = rand;
            for k = 1:length(Pc)-1
                if randnum > Pc(k) && randnum < Pc(k+1)
                    targetPos = slice(j+1).allowedPos(k,:);
                    break
                end
            end
            
            % 更新蚂蚁的当前位置和索引
            antColony(i).pos(j+1,:) = targetPos;
            nowPos = targetPos;
            idx = k; 
        end
        
        % 根据每一个切片的栅格点，利用插值拟合得到三维路径
        [flag,fitness,path] = ACA.calFitness(X,Y,Z, antColony(i).pos,intrp_num);
        antColony(i).path = path;
        
        % 判断路径可行性
        if flag == 0
            % 若不碰撞
            antColony(i).fitness = fitness;

            % 更新单只蚂蚁的最优
            if antColony(i).fitness < antColony(i).Best.fitness
                antColony(i).Best.pos = antColony(i).pos ;
                antColony(i).Best.path = antColony(i).path ;
                antColony(i).Best.fitness = antColony(i).fitness ;
                NowBest = antColony(i).Best;
            end
            
            % 更新全局最优
            if antColony(i).Best.fitness < GlobalBest.fitness
                GlobalBest = antColony(i).Best;
            end
            
            % 统计、更新经过某条路径路径的信息素增量
            for j = 1:sliceNum-1
                % 先判断该路径是否已经存在于deltaTau中
                [~,idx1] = ismember(antColony(i).pos(j,:), deltaTau.start, 'rows');
                [~,idx2] =  ismember(antColony(i).pos(j+1,:), deltaTau.target, 'rows');
                if idx1 == idx2 && idx1 ~= 0
                    deltaTau.delat_tau(idx1)  = deltaTau.delat_tau(idx1) + Q / fitness;
                else
                    deltaTau.start(end+1,:) = antColony(i).pos(j,:);
                    deltaTau.target(end+1,:)  = antColony(i).pos(j+1,:);
                    deltaTau.delat_tau(end+1,1)  = Q / fitness;
                end
            end    
            
        else
             % 若碰撞
             antColony(i).fitness = 1000* fitness;
        end
    end
    
    % 考虑挥发因子，更新信息素
    for i = 1:sliceNum-1
        for j = 1:size(slice(i).allowedPos,1)
            start = slice(i).allowedPos(j,:);
            for k = 1:size(slice(i).par(j).tau,1)
                target = slice(i+1).allowedPos(k,:);
                [~,idx1] = ismember(start, deltaTau.start, 'rows');
                [~,idx2] = ismember(target, deltaTau.target, 'rows');
                if idx1 == idx2 && idx1~=0
                    slice(i).par(j).tau(k,1) = (1-rho)* slice(i).par(j).tau(k,1) +...
                        deltaTau.delat_tau(idx1,1);
                else
                    slice(i).par(j).tau(k,1) = (1-rho)* slice(i).par(j).tau(k,1);
                end
            end
        end
    end
    
    % 把每一代的最优蚂蚁赋值给fitness_beat_iters
    fitness_beat_iters(iter) = GlobalBest.fitness;
    
    % 在命令行窗口显示每一代的信息
    disp(['第' num2str(iter) '代:' '最优适应度 = ' num2str(fitness_beat_iters(iter))]);
    
    % 画图
    ACA.plotFigure(startPos,goalPos,X,Y,Z,GlobalBest,NowBest);
    pause(0.001);
end

savefig("exp/AACA_final_fig") 
save("exp/AACA_GlobalBest.mat","GlobalBest")
save("exp/AACA_fitness_beat_iters.mat","fitness_beat_iters")
%% 结果展示
% 画适应度迭代图
plot(fitness_beat_iters*2200,'LineWidth',2);
legend('AACA算法');
xlabel('迭代次数');
ylabel('最优适应度');
toc
savefig("exp/AACA_iter_fig") 
