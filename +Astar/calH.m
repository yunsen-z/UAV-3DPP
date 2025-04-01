function [flag,fitness,path] = calH(X,Y,Z, pos)
x_seq=pos(:,1)';
y_seq=pos(:,2)';
z_seq= pos(:,3)';

k = length(x_seq);

i_seq = linspace(0,1,k);
I_seq = linspace(0,1,20);
X_seq = pchip(i_seq,x_seq,I_seq);
Y_seq = pchip(i_seq,y_seq,I_seq);
Z_seq = pchip(i_seq,z_seq,I_seq);
path = [X_seq', Y_seq', Z_seq'];

% 判断生成的曲线是否与与障碍物相交
flag = 0;
for i = 2:size(path,1)
    x = path(i,1);
    y = path(i,2);
    z_interp = interp2(X,Y,Z,x,y);
    if path(i,3) < z_interp
        flag = 1;
        break
    end
end

%% 计算三次样条得到的离散点的路径长度（适应度）
dx = diff(X_seq);
dy = diff(Y_seq);
dz = diff(Z_seq);
fitness = sum(sqrt(dx.^2 + dy.^2 + dz.^2));