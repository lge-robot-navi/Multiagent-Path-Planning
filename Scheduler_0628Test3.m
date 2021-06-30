function [coordix, coordiy, coorditheta] = Scheduler_0628Test3(robots)
clc;
clear all;
fprintf('Coverage Generation Start\n');
% global ;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%파라미터 초기화%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
unit = 0.125;%m
patrol_period = 4000; %m
vel = 1;%m/s
dist_for_1second = 0.1/0.125;%0.1초 동안 간거리;
min_dist_for_1second = 0.01/0.125;
patrol_distance = 2000;%vel * patrol_period/2 ;%5m/s
Iteration_num = 100;
min_size = 20; %로봇 최소크기 50cm
cost = 5;
%% 센싱 거리 L_range %%
L_range = 172; %% 9번지도는 20임 %%13번지 도 15
%  L_range = 100; %% 9번지도는 20임 %%13번지 도 15
L_range_base = 20;
x0=0;
y0=0;
width=1200;
%%변수선언
height=1000;
Rangeconstant =3;

%% 가용 로봇대수 %% - 3대이상 테스트 진행해야
% D_RobotNum = 5;
% pathdivision_parameter = 4;
% areacompensation_parameter=19;
D_RobotNum =3;
pathdivision_parameter = 8;
areacompensation_parameter= 3;

% D_RobotNum =2;
% pathdivision_parameter = 16;
% % pathlength_parameter = 3;
% areacompensation_parameter= 8;

% D_RobotNum = 4;
% pathdivision_parameter =2;
% areacompensation_parameter= 4;
% D_RobotNum = 6;
% pathdivision_parameter =2;
% areacompensation_parameter= 4;
% D_RobotNum =2;
% pathdivision_parameter = 4;
% % pathlength_parameter = 3;
% areacompensation_parameter= 5;
D_RobotNum =1;
pathdivision_parameter = 4;
% pathlength_parameter = 3;
areacompensation_parameter= 5;
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
RGB = 150;
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
hold on;

set(gca,'YDir','normal')  


fprintf('Image load completed\n');

    binaryImage(binaryImage<230)=0;
    binaryImage(binaryImage>=230)=255;

%     se = offsetstrel('ball',5,3);
    se = strel('disk',10);
    binaryImage = imclose(binaryImage,se);
    binaryImage = imopen(binaryImage,se);
    
    imshow(binaryImage ); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Node Generation%%%%%%%%%%%%%%%%%%%%%%%%%%%

gx = size(binaryImage,2);
gy = size(binaryImage,1);

%% parameter save

min_size = 3;
load('Parameter_scheduler_pre','unique_shortestPath','shortestPath','shortestPathlength','dmat','unique_all_vertex','unique_all_vertex_base','num_of_nodes');
fprintf('Parameter2 Save load completed\n');
hold on;
% plot(unique_all_vertex(1:3,2),unique_all_vertex(1:3,1),'b','LineWidth',4);
% dmat(1:3,1:3)
% pause;

%%dmat: graph 간 연결도, unique_all_vertex: graph node, shortestPath: TSP Solver결과, patrol_distance: 순찰 거리 조건 
load('precompute','dmat','unique_all_vertex','unique_shortestPath','patrol_distance');
fprintf('loadFinish\n');
load('Debug_Test2','coordix','coordiy','dmat','idxx','L_range','Rangeconstant','binaryImage');
fprintf('loadFinish2\n');
% pause;


Rangeconstant = 2;

for jj = 1:size(coordix,2)
        %% 0807변경
         [shortestPath] = onewaypath(binaryImage,coordix{jj}(:),coordiy{jj}(:),dmat,idxx{jj}(:),L_range,Rangeconstant,cost); 

         coordiy{jj} = coordiy{jj}(shortestPath);
         coordix{jj} = coordix{jj}(shortestPath);
         idxx{jj} = idxx{jj}(shortestPath);
         PathLength1(jj) = 1000;    %%의미없음 pathlength
         
         figure(1)
hold on;
coordiy{jj}(:)
coordix{jj}(:)
plot(coordiy{jj}(:),coordix{jj}(:),'b','LineWidth',4);
pause;
end 
plot(coordiy{1}(1:5),coordix{1}(1:5),'r','LineWidth',5);
fprintf('Path Assignment completed\n');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('kw Start\n');
save('kumohResults5','coordix','coordiy');
% pause;
% load kumohResults5
%% 광운대학교 파트
coordix{1}
[coordix, coordiy, coorditheta, ~] = Func_fovPathPlanning('map_outline_modification_24.jpg', coordix, coordiy); % 1:출력, 2:생략
%% 재저장 %% 'unique_all_vertex'(모든 노드),'dmat------------------------------------------------------------------------------'(거리맵),'ccoordix','ccoordiy','idx' (x,y,path)
originalcoordix = coordix;
 originalcoordiy = coordiy;
 originalcoordith = coorditheta;


overlapped_all_vertex = [];
unique_all_vertex = [];
coordiy(:,cellfun(@any,cellfun(@isnan,coordiy(1,:),'uni',false)))=[];
coordix(:,cellfun(@any,cellfun(@isnan,coordix(1,:),'uni',false)))=[];
coordix(:,cellfun(@any,cellfun(@isempty,coordix(1,:),'uni',false)))=[];
coordiy(:,cellfun(@any,cellfun(@isempty,coordiy(1,:),'uni',false)))=[];
idx{D_RobotNum+1} = [];
coordix
% pause;
node1 = 0;
for i = 1:size(coordix,2)
    if(isempty(coordix{i}))
         idx{i} = [];
        continue;       
    end
    overlapped_all_vertex = [coordix{i}(:) coordiy{i}(:)];
    unique_overlapped_all_vertex = unique(overlapped_all_vertex,'rows','stable');
    unique_all_vertex = [unique_all_vertex;unique_overlapped_all_vertex];   
    ccoordix{i} = unique_overlapped_all_vertex(:,1)';
    ccoordiy{i} = unique_overlapped_all_vertex(:,2)';
    node_size = size(unique_overlapped_all_vertex,1);    
    idx{i} = (node1+1):node1+node_size; 
    node1 = node_size+node1;    
end
fprintf('Node ReConstruction completed\n');
num_of_nodes = size(unique_all_vertex,1);
unique_all_vertex = [unique_all_vertex;unique_all_vertex_base];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%거리맵dmat 생성%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%Node간 연결 여부 및 거리 계산 (미연결일시 무한대로 둠)  
dmat = distancefunction(unique_all_vertex, binaryImage,L_range_base,Rangeconstant,max_value);
At = dmat.';
m  = tril(true(size(At)));
dv  = At(m).';
dv = dv(dv~=0);
size(dmat)
fprintf('Recompute Distance completed\n');
dmat = recalc_distancefunction(num_of_nodes,dmat); %% 계산량 소요문제 존재
fprintf('recalc_distancefunction completed\n');
unique_all_vertex = unique_all_vertex(1:num_of_nodes,:);
dmat = dmat(1:num_of_nodes,1:num_of_nodes);
dmat(L_range*3 < dmat) = 100000;



fprintf('Recalculation & Coverage Generation End\n');
% ccoordix = coordix;
% ccoordiy = coordiy;
% idx = idxx;
ccoordix
coordiy{1}
ccoordix{1}
idx
idx{1}
save('FinalResults','unique_all_vertex','dmat','ccoordix','ccoordiy','idx'); 
coordix = originalcoordix;
coordiy = originalcoordiy;
coorditheta = originalcoordith;

% 66.4/0.05=-1328, -82.775/0.05 = -1655.5
% -109.0=-2180, -105.0=-2100
    for i=1:size(coordix,2)
          for j = 1:size(coordix{i},2)
             posx{i}(1,j) = 0.05*(  coordiy{i}(1,j)-2180);
             posy{i}(1,j) = 0.05*( 3968-2100 - coordix{i}(1,j));
             if(j ==size(coordix{i},2) )
                 posTheta{i}(1,j) = atan2(posy{i}(1,1)-posy{i}(1,j),posx{i}(1,1)-posx{i}(1,j));
             else
                 posTheta{i}(1,j) = atan2(0.05*(3968-2100 - coordix{i}(1,j+1))-posy{i}(1,j),0.05*(coordiy{i}(1,j+1)-2180)-posx{i}(1,j));
             end
          end
    end
    
    coordix = posx;
    coordiy = posy;
    coorditheta = posTheta;
fprintf('kw End\n');
save('kwResults','coordix','coordiy','coorditheta');

% pause;
    