%% ��ʼ��
clc
clear
close all
tic

%% �����ͼ���������������
rng(2025)

%% ��ά��ͼģ��
startPos = [1, 1, 1]; % ���
goalPos = [200, 180, 150]; % �յ�
mapRange = [0,200;0,200;0,200;]; % ɽ���ͼ�Ĵ�С

% ��ͼʵ����
[X,Y,Z] = defMap(mapRange);

%% ������ά�ռ�����·���滮����Ƭ�ṹ��
sliceNum = 19; % �м�·�̵��դ�񻮷�,����ʼ����ڵ�����դ����
perSliceH = round((goalPos(3)-startPos(3)+1)/sliceNum);
slice = struct;
slice.allowedPos = []; % ÿһ����Ƭ���������դ��
slice = repmat(slice,sliceNum,1);

intrp_num =100;

% ���ÿһ����Ƭ������ʵ�դ��
for i = 1:sliceNum+1
    if i == 1
        slice(i).allowedPos = startPos;
    elseif i == sliceNum+1
        slice(i).allowedPos = goalPos;
    else
        h = startPos(2)+(i-1)*perSliceH; % ��Ƭ�߶�
        for x = X(1,1):perSliceH:X(1,end)
            for y = Y(1,1):perSliceH:Y(end,1)
                if h > Z(x,y)
                    slice(i).allowedPos(end+1,:) = [x,y,h];
                end
            end
        end
    end
end

%% A*�㷨ʵ��
% ��������ʽ����������������ֵ�ĵ����ͣ�
heuristic = @(pos1, pos2) sqrt(sum((pos1 - pos2).^2));
% calFitness

% ��ʼ�������б�͹ر��б�
openList = [];
closedList = [];

% ��ʼ�����
startNode.pos = startPos;
startNode.g = 0;
startNode.h = heuristic(startPos, goalPos);
startNode.f = startNode.g + startNode.h;
startNode.parent = [];
openList = [openList; startNode];

% A*�㷨��ѭ��
while ~isempty(openList)
    % �ӿ����б����ҵ�fֵ��С�Ľڵ�
    [~, idx] = min([openList.f]);
    currentNode = openList(idx);
    path_show = reconstructPath(currentNode);
    if size(path_show,1)>=4
        [~,~,path_show] = calH(X,Y,Z, path_show);
        plotFigure(startPos,goalPos,X,Y,Z,path_show);
        pause(0.001);
    end
    % �����ǰ�ڵ���Ŀ��ڵ㣬��·���ҵ�
    if isequal(currentNode.pos, goalPos)
        path = reconstructPath(currentNode);
        break;
    end
    
    % ����ǰ�ڵ�ӿ����б��Ƶ��ر��б�
    openList(idx) = [];
    closedList = [closedList; currentNode];
    
    % ��ȡ��ǰ�ڵ�������ھӽڵ�
    neighbors = getNeighbors(currentNode, slice, sliceNum,goalPos);
    disp(['��������' num2str(neighbors(1).pos)]);
    
    for i = 1:length(neighbors)
        neighbor = neighbors(i);
        
        % ����ھӽڵ��ڹر��б��У�������
        if any(ismember(vertcat(closedList.pos), neighbor.pos, 'rows'))
            continue;
        end

        % �����ھӽڵ��gֵ
        [flag,neighbor_g,path]=calH(X,Y,Z, [reconstructPath(currentNode); neighbor.pos]);
        % tentative_g = currentNode.g + heuristic(currentNode.pos, neighbor.pos);
        if flag==0
            tentative_g = neighbor_g;
        else
            tentative_g = 1000*neighbor_g;
        end
        % ����ھӽڵ㲻�ڿ����б��У������µ�gֵ��С
        if isempty(vertcat(openList.pos)) || ~any(ismember(vertcat(openList.pos), neighbor.pos, 'rows')) || tentative_g < neighbor.g
            neighbor.g = tentative_g;
            neighbor.h = heuristic(neighbor.pos, goalPos);
            neighbor.f = neighbor.g + neighbor.h;
            neighbor.parent = currentNode;
            
            % ����ھӽڵ㲻�ڿ����б��У�����뿪���б�
            if isempty(vertcat(openList.pos)) || ~any(ismember(vertcat(openList.pos), neighbor.pos, 'rows'))
                openList = [openList; neighbor];
            end
        end
    end
end

%% ���չʾ
% ����Ӧ�ȵ���ͼ
[flag,fitness,path] = calFitness(X,Y,Z, path,intrp_num);
disp(['�õ���Fitness' num2str(fitness)]);
plotFigure(startPos,goalPos,X,Y,Z,path);
toc