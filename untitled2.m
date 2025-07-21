%% show map
figure;
imagesc(map);         % 把 map 當成一張圖片畫出來
colormap([1 1 1; 0 0 0]); % 0 畫白色、2畫黑色
colorbar off;          % 不要顏色條
axis equal tight;      % 把格子擠緊、比例正確
set(gca,'YDir','normal'); % Y軸正方向向上
title('Fast Map Display (0=Free, 2=Obstacle)');
%%