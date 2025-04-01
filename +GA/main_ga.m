clc
clear
close all
%% 随机地图的随机数种子设置
rng(2025)

%% 三维地图模型
startPos = [1, 1, 20]; % 起点
goalPos = [200, 160, 90];% 终点
mapRange = [0,200;0,200;0,200;]; % 山峰地图的大小

% 地图实例化
[X,Y,Z] = defMap(mapRange);

%% 这里说明一下
% 利用遗传算法的时候，染色体长度被认为是路线控制点的点数，那么染色体上的控制点就形成了路线
% 种群就是pop，其中存储了

%% 设置超参数
chromLength = 20;     % 染色体长度，代表路线的控制点数，未加首末两点
p_select = 0.5;      % 选择概率,决定选择个体的数量
p_crs = 0.6;         % 交叉概率
p_mut = 0.8;         % 变异概率
popNum = 100;         % 种群规模
intrp_num = 120;     % 插值拟合后的路线碰撞判断点数量。
iterMax = 200;       % 最大迭代数

fitness_beat_iters=[0,iterMax;];

%% 种群初始化
% 产生初始种群   
pop = initPop(popNum,chromLength,mapRange);

% 计算种群适应度
pop = calFitness(startPos, goalPos, X,Y,Z,pop,intrp_num);

% 更新种群最优
GlobalBest.fitness = inf; % 初始化每一代的最优粒子
[pop,GlobalBest,~] = calBest(pop,GlobalBest); % 对最优路线进行更新

%% 主程序
for i = 1:iterMax    
    % 根据轮盘概率随机选择父母
    parentPop = select(pop, p_select);

    % 交叉操作
    childPop = crossover(parentPop,p_crs);
    
    % 变异操作
    childPop = mutation(childPop,p_mut,mapRange);
    
    % 将父代和子代组合得到新的种群
    pop = [parentPop, childPop];
    
    % 计算种群适应度
    pop = calFitness(startPos, goalPos, X,Y,Z,pop,intrp_num);

    % 更新最优记录
    [pop,GlobalBest,NowBest] = calBest(pop,GlobalBest);
    
    % 把每一代的最优粒子赋值给fitness_beat_iters
    fitness_beat_iters(i) = GlobalBest.fitness;
    
    % 在命令行窗口显示每一代的信息
    disp(['第' num2str(i) '代:' '最优适应度 = ' num2str(fitness_beat_iters(i))]);
    
    % 画图
    plotFigure(startPos,goalPos,X,Y,Z,GlobalBest,NowBest);
    pause(0.001);
end

%% 结果展示

% 画适应度迭代图
figure
plot(fitness_beat_iters*2200,'LineWidth',2);
hold on;
% 画适应度迭代图
legend('GA算法');
xlabel('迭代次数');
ylabel('最优适应度');
toc

