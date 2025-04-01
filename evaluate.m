clc
clear
close all
% run("ACA.m")
% run("GA.m")
% run("Astar.m")
% run("AACA.m")
GA_GlobalBest=load('exp\GA_GlobalBest.mat');
ACA_GlobalBest=load('exp\ACA_GlobalBest.mat');
Astar_GlobalBest=load('exp\Astar_GlobalBest.mat');
AACA_GlobalBest=load("exp\AACA_GlobalBest.mat");
figure(1);
bar(["GA","ACA","Astar","AACA"],...
    [GA_GlobalBest.GlobalBest.fitness,...
    ACA_GlobalBest.GlobalBest.fitness,...
    Astar_GlobalBest.GlobalBest.fitness,...
    Astar_GlobalBest.GlobalBest.fitness])

figure(2);
GA_fitness_beat_iters=load('exp\GA_fitness_beat_iters.mat');
ACA_fitness_beat_iters=load('exp\ACA_fitness_beat_iters.mat');
% Astar_fitness_beat_iters=load('exp\Astar_fitness_beat_iters.mat');
AACA_fitness_beat_iters=load("exp\AACA_fitness_beat_iters.mat");
plot(GA_fitness_beat_iters.fitness_beat_iters,'LineWidth',2);
hold on;
plot(ACA_fitness_beat_iters.fitness_beat_iters,'LineWidth',2);
hold on;
plot(AACA_fitness_beat_iters.fitness_beat_iters,'LineWidth',2);
legend('GA','ACA','AACA');
xlabel('迭代次数');
ylabel('最优适应度');
hold off;


iters.time.GA=20;
iters.time.ACA=53.842;
iters.time.Astar=100.186201;
iters.time.AACA=10.012269;

iters.best_iter.GA=13;
iters.best_iter.ACA=20;
iters.best_iter.Astar=17;
iters.best_iter.AACA=4;

iters.GA_fitness=GA_fitness_beat_iters.fitness_beat_iters;
iters.ACA_fitness=ACA_fitness_beat_iters.fitness_beat_iters;
iters.AACA_fitness=AACA_fitness_beat_iters.fitness_beat_iters;

time.GA=(1:1:size(iters.GA_fitness,2))*iters.time.GA/iters.best_iter.GA;
time.ACA=(1:1:size(iters.ACA_fitness,2))*iters.best_iter.ACA/iters.time.ACA;
time.AACA=(1:1:size(iters.AACA_fitness,2))*iters.best_iter.AACA/iters.time.AACA;
time.AACA=iters.time.AACA+time.AACA-time.AACA(end);

figure(3);
plot(time.GA, iters.GA_fitness,'LineWidth',2);
hold on;
plot(time.ACA, iters.ACA_fitness,'LineWidth',2);
hold on;
plot(time.AACA, iters.AACA_fitness,'LineWidth',2); 
legend('GA','ACA','AACA');
xlabel('运行时间');
ylabel('最优适应度');
hold off;


figure(4)
scatter([26.17,26.29,28.00,27.24,26.99],[392.15,351.25,433.69,417.91,440.97],100,'r', 'filled','MarkerEdgeColor', 'k');
hold on;
scatter([30.98,52.70,85.19],[365.27,352.34,	393.18],100,'b', 'filled','MarkerEdgeColor', 'k');
hold on;
scatter([18.27,100.19,2301.62],[314.28,313.59,320.42],100,'y', 'filled','MarkerEdgeColor', 'k');
hold on;
scatter([4.93,10.01,22.36],[334.52,316.18,315.48],100,'g', 'filled','MarkerEdgeColor', 'k');
legend('GA','ACA','A*','AACA');
set(gca, 'XScale', 'log'); % 将横轴设置为对数分布
xlabel('运行时间');
ylabel('最优适应度');
hold off;