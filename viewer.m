% date:21-05-18

clc;
clear;


DLLdataFileName = 'log1.bin';
% DLLdataFileName = 'test_right.bin';
% DLLdataFileName = 'test_left.bin';
% DLLdataFileName = 'test_rear.bin';
DLLdataFile = fopen(DLLdataFileName);

dcsRange = 4;

limit = 3;

xMargin = 0;
yMargin = 0;

% ToF
x = 320;
y = 40;
ox = x/2;
oy = y/2;
fx = 2.5;   % mm
fy = fx/2;
lc = 0.02;  % mm

% P camera matrix
P = [fx/lc, 0,     ox; ...
     0,     fy/lc, oy; ...
     0,     0,     1];

f = figure(1);
clf;
hold on;
grid;
cc = colorbar;
cc.Color = [1 1 1];
colormap turbo;
set(gcf, 'Color', 'k');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.4);
axis equal;
axis([-limit/2, limit/2, 0, limit, -3, 3]);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
view(0, 90);
% view([-1 -1 1]);
p(1) = scatter3(0,0,0);
p(2) = scatter3(0,0,0);

hdrFlag = 0;

f2 = figure(2);
clf;
hold on;
axis equal;
cc = colorbar;
cc.Color = [1 1 1];
set(gcf, 'Color', 'k');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.4);
colormap turbo;
axis([1 80 1 320]);
view(90, 90);
p2(1) = scatter3(0,0,0);
p2(2) = scatter3(0,0,0);

depthImage = zeros(x,y);
depth3d = zeros(x*y, 3);

% currently, read the data in int16 casting
while(ishandle(f))
    row = fread(DLLdataFile, [325, 1], 'int16');
    if isempty(row)
        break;
    end
    index = mod(row(3),256);
    dll = floor(row(3)/256) - 10;
    sh = mod(row(4),256);
    dcs = floor(row(4)/256);
    if index < y
%         depthImage(:, index+1, dcs+1) = 0.2*depthImage(:, index+1, dcs+1) + 0.8*row(5:end-1);
        depthImage(:, index+1, 1) = row(5:end-1);
    end

    if index == (y - 1)
        depth = depthImage(:, :)/1000;
        
%         s*x = fx*X + ox*Z
%         s*y = fy*Y + oy*Z
%         s = Z
% 
%         X/Z = ax
%         Y/Z = ay
% 
%         d = sqrt(X^2+Y^2+Z^2)
% 
%         Z = 
% 
%         X = ax*sqrt(d^2 - X^2 - Y^2)
%         Y = ay*sqrt(d^2 - X^2 - Y^2)
% 
%         ((1+ax^2)/ax^2)*X^2 = d^2 - Y^2
%         X^2 = (d^2 - Y^2)/((1+ax^2)/ax^2)
% 
%         ((1+ay^2)/ay^2)*Y^2 = d^2 - X^2
% 
%         ((1+ay^2)/ay^2)*Y^2 = d^2 + (-d^2 + Y^2)/((1+ax^2)/ax^2)
% 
%         ((1+ay^2)/ay^2) = Ay
%         ((1+ax^2)/ax^2) = Ax
% 
%         Ay*Ax*Y^2 = Ax*d^2 - d^2 + Y^2
% 
%         Y^2 = (Ax-1)*d^2/(Ay*Ax - 1)
%         X^2 = (Ay-1)*d^2/(Ay*Ax - 1)
%         Z^2 = d^2 - X^2 - Y^2
        
        depth3d = zeros(x*y + 3, 3);
        depth3d(1, :) = [0, 0, limit];
        depth3d(2, :) = [0, 1.5, limit];
        depth3d(3, :) = [0, -1, limit];
        for i = 1+xMargin:x-xMargin
            for j = 1+yMargin:y-yMargin
                if depth(i, j) > 0.5 && depth(i, j) < limit
%                     d = [(i - ox)*lc/fx, (j - oy)*lc/fy, 1];
%                     depth3d((i-1)*y + j, :) = depth(i, j)*d/norm(d);

                    d = depth(i, j);

                    ax = (i - ox)*lc/fx;
                    ay = (j - oy)*lc/fy;
                    Ax = (1 + ax^2)/(ax^2);
                    Ay = (1 + ay^2)/(ay^2);
                    
                    X = sign(ax)*sqrt(((Ay - 1)*d^2) / (Ay*Ax - 1));
                    Y = sign(ay)*sqrt(((Ax - 1)*d^2) / (Ax*Ay - 1));
                    Z = sqrt(d^2 - X^2 - Y^2);
                    
%                     if Y > -1 && Y < 1.5 && X < limit*2/3 && X > -limit*2/3
                        depth3d((i-1)*y + j+4, :) = [X, Y, Z];
%                     end
                end
            end
        end
        
        if dcs == 0
            figure(1);
            delete(p(1));
            p(1) = scatter3(depth3d(:,1), depth3d(:,3), -depth3d(:,2), 30, -depth3d(:,2),'.');
            
%             figure(2);
%             delete(p2(1));
%             p2(1) = surf(2*(1:40), 1:320, depth, 'edgecolor', 'none');
        else
            figure(1);
            delete(p(2));
            p(2) = scatter3(depth3d(:,1), depth3d(:,3), -depth3d(:,2), 30, -depth3d(:,2),'.');
            
%             figure(2);
%             delete(p2(2));
%             p2(2) = surf(2*(1:40), 1:320, depth, 'edgecolor', 'none');
        
            pause(0.05);
        end
    end

    if dcs >= 0 && dcs <= 3
        priDCS = dcs;
    end

    if dll >= 0 && dll <= 49
        priDLL = dll;
    end

    if mod(sh, 10) == 0
        priSH = sh;
    end
end