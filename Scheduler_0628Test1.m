function [coordix, coordiy, coorditheta] = Scheduler_0628Test1(robots)
clc;
clear all;
fprintf('Coverage Generation Start\n');
% global ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%파라미터 초기화%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
unit = 0.125;%m
patrol_period = 4000; %m
vel = 1;%m/s
dist_for_1second = 0.1/0.125;%0.1초 동안 간거리;
min_dist_for_1second = 0.01/0.125;
patrol_distance = 2000;%vel * patrol_period/2 ;%5m/s
Iteration_num = 100;
min_size = 20; %로봇 최소크기 50cm
%% 센싱 거리 L_range %%
L_range = 180; %% 9번지도는 20임 %%13번지 도 15
%  L_range = 100; %% 9번지도는 20임 %%13번지 도 15
L_range_base = 19;
x0=0;
y0=0;
width=1200;
%%변수선언
height=1000;
Rangeconstant = 3;

%% 가용 로봇대수 %% - 3대이상 테스트 진행해야
D_RobotNum = 3;
pathlength_parameter = 3;
areacompensation_parameter= 6;
%% 가용 로봇대수 %%
POI_RobotNum = 1;
POI = 2; % on off 2
POI_y = [];
POI_x = [];  
D_smallindex = 36;
D_mindist = L_range;
Debug = 1; % debug on 1 /off 2;
Debug_Plot =1;
%%free space 검출 파라미터
% RGB = 120;
% RGB2 = 200;
RGB = 200;
RGB2 = 230;
%%출력파라미터
w = 50;
h = 50;
%% 거리 최대값 0811
max_value = 10000;%
axis image 
title('Input Map','color','w');
axis off
set(gcf, 'InvertHardCopy', 'off');
hold off;
set(gcf,'position',[x0,y0,width,height])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%파라미터 초기화%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
binaryImage = Image_load(POI,0); %% image load.최신 지도 (아직 수정필요)
binaryImage1 = Image_load(POI,1); %% image load. 

%  binaryImage = Image_load(POI,2);  %% image load.
J=rgb2gray(binaryImage1);
J = flipud(J);
J = imresize(J, [2800 3400]); 
J = J';
J=imtranslate(J,[-70, -50]); 
hold on;

% imagesc(binaryImage ); %% represent the image
% imshow(binaryImage ); 

% imagesc(J, 'AlphaData', 0.5);

% imagesc(J);
% imagesc(binaryImage);
%  imagesc(J, 'AlphaData', 0.5) 
% colormap(gray)
set(gca,'YDir','normal')  
if(Debug==1)
%     pause;
end

fprintf('Image load completed\n');
% pause;
  
%     binaryImage = Image_load(POI,0);
    binaryImage(binaryImage<230)=0;
    binaryImage(binaryImage>=230)=255;

%     se = offsetstrel('ball',5,3);
    se = strel('disk',10);
    binaryImage = imclose(binaryImage,se);
    binaryImage = imopen(binaryImage,se);
    
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Node Generation%%%%%%%%%%%%%%%%%%%%%%%%%%%

gx = size(binaryImage,2);
gy = size(binaryImage,1);
dense_size = 100;

origin_image = binaryImage;
% se =strel('disk', 15);
% dilatedI = imdilate(origin_image,se);

binaryImage=imread('map_outline_ver3.jpg'); %%지도 파일
 binaryImage(binaryImage<230)=0;
    binaryImage(binaryImage>=230)=255;
      se = strel('disk',5);
    binaryImage = imclose(binaryImage,se);
    binaryImage = imopen(binaryImage,se);
    
% for i=1:gy
%     for j = 1:gx
%         if(origin_image(i,j) == 255)
%             if( 1<=(i)-dense_size && (i+dense_size)<=gy && 1<=(j-dense_size) && (j+dense_size)<=gx)
%                 for k = -dense_size:dense_size
%                     for z = -dense_size:dense_size
%                         if(origin_image(i+k,j+z) == 0)
%                               binaryImage(i+k,j+z)=0;
%                               break;
%                         end
%                     end
%                      if(binaryImage(i+k,j+z) == 0)                          
%                               break;
%                      end
%                 end
%             end
%         end
%     end
% end
imshow(binaryImage ); 
se = offsetstrel('ball',2,2);
binaryImage = imerode(binaryImage,se); 
imshow(origin_image)
% pause;

%% parameter save

min_size = 0.8;
unique_all_vertex = NodeConstruction(binaryImage,gx,gy,L_range,RGB,RGB2,POI);
% pause;
unique_all_vertex_base = NodeConstruction_base(binaryImage,gx,gy,L_range_base,RGB,RGB2,POI,min_size);
fprintf('Node Construction completed\n');

save('Parameter','binaryImage','gx','gy','Rangeconstant','max_value','L_range','L_range_base','RGB','RGB2','POI','unique_all_vertex_base');
fprintf('Parameter Save completed\n');

%% 0807추가
num_of_nodes = size(unique_all_vertex,1);
unique_all_vertex = [unique_all_vertex;unique_all_vertex_base];
%% 0807추가
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Node Generation%%%%%%%%%%%%%%%%%%%%%%%%%%%
% L_range2 = 5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%거리맵dmat 생성%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%Node간 연결 여부 및 거리 계산 (미연결일시 무한대로 둠) 
 
dmat = distancefunction(unique_all_vertex, binaryImage,L_range_base,Rangeconstant,max_value);
At = dmat.';
m  = tril(true(size(At)));
dv  = At(m).';
dv = dv(dv~=0);
size(dmat)
hold on; 

fprintf('Distance Function completed\n');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%거리맵dmat 생성%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% figure(2)
% for i=1:5 hold on;
%      if(i == 1)
%       plot(unique_all_vertex(find(idx==i),1),unique_all_vertex(find(idx==i),2),'Marker','o','color',[1 0 0],'LineStyle','none','LineWidth',4);   
%       count = 2;
% 
%     elseif(i == 2)
%            plot(unique_all_vertex(find(idx==i),1),unique_all_vertex(find(idx==i),2),'Marker','o','color',[0 1 0],'LineStyle','none','LineWidth',4);   
%             count = 2;
% 
%      elseif(i == 3)
%        plot(unique_all_vertex(find(idx==i),1),unique_all_vertex(find(idx==i),2),'Marker','o','color',[0 0 1],'LineStyle','none','LineWidth',4);    
%        count = 2;
% 
%     elseif(i == 4)
%         plot(unique_all_vertex(find(idx==i),1),unique_all_vertex(find(idx==i),2),'Marker','o','color',[0 1 1],'LineStyle','none','LineWidth',4);    
%         count = 2;
% 
%     elseif(i==5)
%         plot(unique_all_vertex(find(idx==i),1),unique_all_vertex(find(idx==i),2),'Marker','o','color',[1 1 0],'LineStyle','none','LineWidth',4);    
%         count = 2;
%      end
% end
% test=1
% pause;

%% 0807추가
dmat = recalc_distancefunction(num_of_nodes,dmat); %% 계산량 소요문제 존재
fprintf('recalc_distancefunction completed\n');
unique_all_vertex = unique_all_vertex(1:num_of_nodes,:);
dmat = dmat(1:num_of_nodes,1:num_of_nodes);
%% 0813 수정 *2->*3
dmat(L_range*3 < dmat) = 100000;
%% 0807추가
% unique_all_vertex(find(idx==i),1),unique_all_vertex(find(idx==i),2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Nearest Neighbor based method%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%Nearest Neighbor based method for TSP
cost = 2;
[shortestPath,shortestPathlength] = NNTSP(dmat,unique_all_vertex,Debug_Plot,cost);
fprintf('Single Robot TSP completed\n');


%  shortestPath =  solveTSP( dmat,unique_all_vertex, false);
%  shortestPath = TSP2(dv,unique_all_vertex,Debug_Plot);

% function varargout = solveTSP(dmat, cities, display,dist)


% pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Nearest Neighbor based method%%%%%%%%%%%%%%%%%%%%%%%%%%% 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Overlapping path rejection 및 Unique Path Cluster%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%unique path 결정 여부 
%% 경로 Cluster 생성
unique_shortestPath = unique(shortestPath,'stable');

fprintf('Clustering completed\n');

%%
% 
%   for x = 1:10
%       disp(x)
%   end
% 

save('Parameter_scheduler_pre','unique_shortestPath','shortestPath','shortestPathlength','dmat','unique_all_vertex','unique_all_vertex_base','num_of_nodes');
fprintf('Parameter2 Save completed\n');






