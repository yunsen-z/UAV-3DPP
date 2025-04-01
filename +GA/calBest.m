function [pop,GlobalBest,NowBest] = calBest(pop,GlobalBest)
NowBest=pop(1).Best;
for i = 1:size(pop,1)
    % ���¸��������
    if pop(i).fitness < pop(i).Best.fitness
        pop(i).Best.pos = pop(i).pos;
        pop(i).Best.fitness = pop(i).fitness;
        pop(i).Best.path = pop(i).path;
    end
    
    % ����ȫ������
    if pop(i).Best.fitness < GlobalBest.fitness
        GlobalBest = pop(i).Best;
    end

    % ���µ�ǰ����
    if pop(i).Best.fitness < NowBest.fitness
        NowBest = pop(i).Best;
    end
end
