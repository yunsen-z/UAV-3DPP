%%
clc
clear
close all
tic
%% �����ͼ���������������
rng(2025)

%% ��ά��ͼģ��
startPos = [1, 1, 1]; % ���
goalPos = [200, 180, 150];% �յ�
mapRange = [0,200;0,200;0,200;]; % ɽ���ͼ�Ĵ�С

% ��ͼʵ����
[X,Y,Z] = defMap(mapRange);

%% ������ά�ռ�����·���滮����Ƭ�ṹ��,��������Ѱ��·��
% ����ά�ռ����ܹ��������ϵĵ�,����������������Ϣ
% �ܹ�ǰ����λ��;��Ӧλ�õ���Ϣ�غ�����ֵ

sliceNum = 9;                        % �м�·�̵��դ�񻮷�,����ʼ����ڵ�����դ����
perSliceH = (goalPos(3)-startPos(3)+1)/sliceNum;
slice = struct;
slice.allowedPos = [];                % ÿһ����Ƭ���������դ��
slice.par = [];                       % ÿһ����Ƭ������һ����Ƭ�Ĳ�������Ϣ�ص�
slice = repmat(slice,sliceNum,1);

% ���ÿһ����Ƭ������ʵ�դ��
for i = 1:sliceNum+1
    if i == 1
        slice(i).allowedPos = startPos;
    elseif i == sliceNum+1
        slice(i).allowedPos = goalPos;
    else
        h = startPos(2)+(i-1)*perSliceH;  % ��Ƭ�߶�
        for x = X(1,1):perSliceH:X(1,end)
            for y = Y(1,1):perSliceH:Y(end,1)
                if h > Z(x,y)
                    slice(i).allowedPos(end+1,:) = [x,y,h];
                end
            end
        end
    end
end

% ��ʼ����Ϣ�غ�����ֵ
for i = 1:sliceNum-1
    for j = 1:size(slice(i).allowedPos,1)
        pathNum = size(slice(i+1).allowedPos,1);
        slice(i).par(j).tau = ones(pathNum,1);
        deltaX = slice(i+1).allowedPos(:,1) - slice(i).allowedPos(j,1);
        deltaY = slice(i+1).allowedPos(:,2) - slice(i).allowedPos(j,2);
        deltaZ = slice(i+1).allowedPos(:,3) - slice(i).allowedPos(j,3);
        slice(i).par(j).eta = 100./sqrt(deltaX.^2 + deltaY.^2 + deltaZ.^2);
    end
end

%% ������Ⱥ�ṹ��
% ��Ⱥ��ض���
m = 20;                             % ��������,ÿ���������Լ���·
alpha = 10;                          % ��Ϣ����Ҫ�̶�����
beta = 1;                            % ����������Ҫ�̶�����
rho = 0.1;                           % ��Ϣ�ػӷ�����
Q = 1;                               % ����
intrp_num = 120;     % ��ֵ��Ϻ��·����ײ�жϵ�������
iter_max = 200;                      % ����������

% ������Ⱥ�ṹ��
antColony = struct;
antColony.pos= [];
antColony.path = [];
antColony.fitness = [];
antColony.Best.pos = [];
antColony.Best.path = [];
antColony.Best.fitness = inf;
antColony = repmat(antColony,m,1);

% ��������·�߼�¼
GlobalBest = struct;
GlobalBest.fitness=inf;
GlobalBest.pos = [];
GlobalBest.path = [];

NowBest = struct;
NowBest.fitness=inf;
NowBest.pos = [];
NowBest.path = [];

%% ����Ѱ�����·��
for  iter = 1:iter_max

    % ��ʼ����Ⱥ·���仯��Ϣ�ؽṹ��
    deltaTau = struct;
    deltaTau.start = [0,0,0];
    deltaTau.target = [0,0,0];
    deltaTau.delat_tau = (0);
    
    % �������·��ѡ��
    for i = 1:m
        
        % ����ʼλ�ú�Ŀ��λ�ô������Ⱥ�ṹ����
        antColony(i).pos(1,:) = startPos;
        antColony(i).pos(sliceNum,:) = goalPos;
        nowPos = startPos;
        idx = 1;
        
        % �����Ƭ��դ��ѡ��
        for j = 1:sliceNum-2            
            %������һ���ڵ�ķ��ʸ���
            P = slice(j).par(idx).tau .^alpha .* slice(j).par(idx).eta .^ beta;
            P = P/sum(P);
            
            % ���̶ķ�ѡ����һ�����ʽڵ�
            Pc = cumsum(P);
            Pc = [0; Pc];
            randnum = rand;
            for k = 1:length(Pc)-1
                if randnum > Pc(k) && randnum < Pc(k+1)
                    targetPos = slice(j+1).allowedPos(k,:);
                    break
                end
            end
            
            % �������ϵĵ�ǰλ�ú�����
            antColony(i).pos(j+1,:) = targetPos;
            nowPos = targetPos;
            idx = k; 
        end
        
        % ����ÿһ����Ƭ��դ��㣬���ò�ֵ��ϵõ���ά·��
        [flag,fitness,path] = calFitness(X,Y,Z, antColony(i).pos,intrp_num);
        antColony(i).path = path;
        
        % �ж�·��������
        if flag == 0
            % ������ײ
            antColony(i).fitness = fitness;

            % ���µ�ֻ���ϵ�����
            if antColony(i).fitness < antColony(i).Best.fitness
                antColony(i).Best.pos = antColony(i).pos ;
                antColony(i).Best.path = antColony(i).path ;
                antColony(i).Best.fitness = antColony(i).fitness ;
                NowBest = antColony(i).Best;
            end
            
            % ����ȫ������
            if antColony(i).Best.fitness < GlobalBest.fitness
                GlobalBest = antColony(i).Best;
            end
            
            % ͳ�ơ����¾���ĳ��·��·������Ϣ������
            for j = 1:sliceNum-1
                % ���жϸ�·���Ƿ��Ѿ�������deltaTau��
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
             % ����ײ
             antColony(i).fitness = 1000* fitness;
        end
    end
    
    % ���ǻӷ����ӣ�������Ϣ��
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
    
    % ��ÿһ�����������ϸ�ֵ��fitness_beat_iters
    fitness_beat_iters(iter) = GlobalBest.fitness;
    
    % �������д�����ʾÿһ������Ϣ
    disp(['��' num2str(iter) '��:' '������Ӧ�� = ' num2str(fitness_beat_iters(iter))]);
    
    % ��ͼ
    plotFigure(startPos,goalPos,X,Y,Z,GlobalBest,NowBest);
    pause(0.001);
end

%% ���չʾ
% ����Ӧ�ȵ���ͼ
plot(fitness_beat_iters*2200,'LineWidth',2);
legend('ACA�㷨');
xlabel('��������');
ylabel('������Ӧ��');
toc
