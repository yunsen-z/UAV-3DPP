clc
clear
close all
%% �����ͼ���������������
rng(2025)

%% ��ά��ͼģ��
startPos = [1, 1, 20]; % ���
goalPos = [200, 160, 90];% �յ�
mapRange = [0,200;0,200;0,200;]; % ɽ���ͼ�Ĵ�С

% ��ͼʵ����
[X,Y,Z] = defMap(mapRange);

%% ����˵��һ��
% �����Ŵ��㷨��ʱ��Ⱦɫ�峤�ȱ���Ϊ��·�߿��Ƶ�ĵ�������ôȾɫ���ϵĿ��Ƶ���γ���·��
% ��Ⱥ����pop�����д洢��

%% ���ó�����
chromLength = 20;     % Ⱦɫ�峤�ȣ�����·�ߵĿ��Ƶ�����δ����ĩ����
p_select = 0.5;      % ѡ�����,����ѡ����������
p_crs = 0.6;         % �������
p_mut = 0.8;         % �������
popNum = 100;         % ��Ⱥ��ģ
intrp_num = 120;     % ��ֵ��Ϻ��·����ײ�жϵ�������
iterMax = 200;       % ��������

fitness_beat_iters=[0,iterMax;];

%% ��Ⱥ��ʼ��
% ������ʼ��Ⱥ   
pop = initPop(popNum,chromLength,mapRange);

% ������Ⱥ��Ӧ��
pop = calFitness(startPos, goalPos, X,Y,Z,pop,intrp_num);

% ������Ⱥ����
GlobalBest.fitness = inf; % ��ʼ��ÿһ������������
[pop,GlobalBest,~] = calBest(pop,GlobalBest); % ������·�߽��и���

%% ������
for i = 1:iterMax    
    % �������̸������ѡ��ĸ
    parentPop = select(pop, p_select);

    % �������
    childPop = crossover(parentPop,p_crs);
    
    % �������
    childPop = mutation(childPop,p_mut,mapRange);
    
    % ���������Ӵ���ϵõ��µ���Ⱥ
    pop = [parentPop, childPop];
    
    % ������Ⱥ��Ӧ��
    pop = calFitness(startPos, goalPos, X,Y,Z,pop,intrp_num);

    % �������ż�¼
    [pop,GlobalBest,NowBest] = calBest(pop,GlobalBest);
    
    % ��ÿһ�����������Ӹ�ֵ��fitness_beat_iters
    fitness_beat_iters(i) = GlobalBest.fitness;
    
    % �������д�����ʾÿһ������Ϣ
    disp(['��' num2str(i) '��:' '������Ӧ�� = ' num2str(fitness_beat_iters(i))]);
    
    % ��ͼ
    plotFigure(startPos,goalPos,X,Y,Z,GlobalBest,NowBest);
    pause(0.001);
end

%% ���չʾ

% ����Ӧ�ȵ���ͼ
figure
plot(fitness_beat_iters*2200,'LineWidth',2);
hold on;
% ����Ӧ�ȵ���ͼ
legend('GA�㷨');
xlabel('��������');
ylabel('������Ӧ��');
toc

