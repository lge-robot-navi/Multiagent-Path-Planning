function [coordix, coordiy, coorditheta] = Scheduler_ph(robots)
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
L_range = 172; %% 9번지도는 20임 %%13번지 도 15
%  L_range = 100; %% 9번지도는 20임 %%13번지 도 15
L_range_base = 20;
x0=0;
y0=0;
width=1200;
%%변수선언
height=1000;
Rangeconstant = 2.5;

%% 가용 로봇대수 %%
D_RobotNum = 3;
%% 가용 로봇대수 %%
POI_RobotNum = 1;
POI = 2; % on off 2
POI_y = [];
POI_x = [];  
D_smallindex = 36;
D_mindist = L_range;
Debug = 2; % debug on 1 /off 2;
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
    pause;
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

%% parameter save

min_size = 3;
unique_all_vertex = NodeConstruction(binaryImage,gx,gy,L_range,RGB,RGB2,POI);
unique_all_vertex_base = NodeConstruction_base(binaryImage,gx,gy,L_range_base,RGB,RGB2,POI,min_size);
fprintf('Node Construction completed\n');

save('Parameter','binaryImage','gx','gy','Rangeconstant','max_value','L_range','L_range_base','RGB','RGB2','POI','unique_all_vertex_base');
fprintf('Parameter Save completed\n');

% pause;
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
save('Parameter_dist','num_of_nodes','unique_all_vertex','dmat');

pause;
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
[shortestPath,shortestPathlength] = NNTSP(dmat,unique_all_vertex,Debug_Plot);
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
patrol_distance = shortestPathlength/(2*D_RobotNum);
%%dmat: graph 간 연결도, unique_all_vertex: graph node, shortestPath: TSP Solver결과, patrol_distance: 순찰 거리 조건 
save('precompute','dmat','unique_all_vertex','unique_shortestPath','patrol_distance');
fprintf('SaveFinish\n');
%%dmat: graph 간 연결도, unique_all_vertex: graph node, shortestPath: TSP Solver결과, patrol_distance: 순찰 거리 조건 
[coordix,coordiy,current_robot,idxx,PathLength1] = pathdivision(dmat,unique_all_vertex,unique_shortestPath,patrol_distance);
   
coordiy= coordiy(~any(cellfun('isempty', coordiy), 1));
coordix= coordix(~any(cellfun('isempty', coordix), 1));
idxx= idxx(~any(cellfun('isempty', idxx), 1));
PathLength1 = PathLength1(1:size(coordix,2));      

%% 경로 길이 재계산 %%현재 의미 X
for k = 1:size(coordiy,2)
  PathLength1(k) = 0;
  if(size(coordiy{k},2) >=2)              
    for j = 2:size(coordiy{k},2)
      PathLength1(k) = PathLength1(k)+sqrt((coordiy{k}(j)-coordiy{k}(j-1))^2+(coordix{k}(j)-coordix{k}(j-1))^2);
    end
  end
end
size(coordix,2) 
% pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Overlapping path rejection 및 Unique Path Cluster%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%debug
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%Draw  Path Cluster%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
 %% Draw Path Cluster
if(Debug_Plot==1)
 for i=1:size(coordix,2) 
   hold on;
   color = [rand(1),rand(1),rand(1)];
      color2 = [color 0.1];
     for drawi = 1:size(coordiy{i},2)
        plot(coordiy{i}(drawi),coordix{i}(drawi),'Marker','o','color',color,'LineWidth',4);
        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', color2,'EdgeColor','none');
        pause(0.02);
     end
     pause(1);
 end
%  pause;
end
% pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%Draw  Path Cluster%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
        if(POI == 1)
             [POI_x,POI_y] = Temporary_Test(until_second_node_group0,L_range,RangeconstantPOI_x,POI_y);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%% %%% Path Cluster 추가 변경 %%%%%%%%%%%%%%%%%%%%%%%%%%% 
    DebugPathlength = PathLength1;
     Debugcurrent_robot=current_robot;
     count = 1;       
        total_node = 0;      
         for i=1:size(coordiy,2)
              total_node = total_node + size(coordiy{i},2);             
         end
          itr = size(coordiy,2);       
           min_length = 3;%floor(total_node/(4*D_RobotNum));
 %%% 로봇의 개수에 따른 노드의 개수를 일괄적으로 나눔%%%%%%%%%%%%%%%%%%%%%%%%%%%       (path cluster가 너무 클경우 또 나눔)   
        if(D_RobotNum~=0)
            for i=1:itr
                 %% 0813 수정 /2.5 * -> /3*
               if(size(coordiy{i},2) > min_length && min_length~=0)
                      N = floor(size(coordiy{i},2)/min_length)
                      count = 1;
                      originalcoordy = coordiy{i};
                      originalcoordx = coordix{i};
                      originalidxx = idxx{i};
                      coordiy{i}=coordiy{i}(1:end-(N-1)*min_length);
                      coordix{i}=coordix{i}(1:end-(N-1)*min_length);
                      idxx{i}=idxx{i}(1:end-(N-1)*min_length); 
                while(count < N)
                                   
                      coordiy{end+1}=originalcoordy(end-(N-1)*min_length+1+min_length*(count-1):end-(N-1)*min_length+min_length+min_length*(count-1));
                      coordix{end+1}=originalcoordx(end-(N-1)*min_length+1+min_length*(count-1):end-(N-1)*min_length+min_length+min_length*(count-1));
                      idxx{end+1}=originalidxx(end-(N-1)*min_length+1+min_length*(count-1):end-(N-1)*min_length+min_length+min_length*(count-1));
                       count = count+1;
                     
                     %% 0811 수정                      
%                         itr2 = size(coordiy,2);
%                          while(size(coordiy{itr2},2) > total_node/(2.5*D_RobotNum))                             
%                               coordiy{end+1}=coordiy{itr2}(1+floor(end/2):end);
%                               coordiy{itr2}=coordiy{itr2}(1:floor(end/2));
%                               coordix{end+1}=coordix{itr2}(1+floor(end/2):end);
%                               coordix{itr2}=coordix{itr2}(1:floor(end/2));
%                           %% 0811 수정
%                               %idxx{current_robot} = [idx{current_robot} unique_shortestPath(i)];    
%                                 idxx{end+1}=idxx{itr2}(1+floor(end/2):end);
%                               idxx{itr2}=idxx{itr2}(1:floor(end/2));
%                                 itr2 = size(coordiy,2);
%                              %% 0811 수정
%                          end
                end
%                   itr = size(coordiy,2);  
                 end
            end
        end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
 
%         pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Path Cluster Combination %%%%%%%%%%%%%%%%%%%%%%%%%%% 

originaly =  coordiy;
originalx =  coordix;
originalidx = idxx;


 %%%%%%%%%%%%%%%%%%%%%%% 나눠진 순찰 경로를 k-means 방법으로 결합 %%%%%%%%%%%%%%%%%%%%%     
 %% 0807변경
[ccoordix,ccoordiy,idx,cluster_save]=Kmeans_based_Clustering(unique_all_vertex_base,coordix,coordiy,idxx,D_RobotNum,Debug_Plot,w,h,L_range,Rangeconstant)    ;
 %%%%%%%%%%%%%%%%%%%%%%% 나눠진 순찰 경로를 k-means 방법으로 보정 %%%%%%%%%%%%%%%%%%%%%     

fprintf('Node Assignment completed\n');
 
 size_coordi = [];
 for i = 1:size(ccoordiy,2)
     size_coordi(i) = size(ccoordiy{i},2);
 end

 %% 영역 재보정
 while((max(size_coordi) - min(size_coordi)) > min(size_coordi)/2.5)     
     [maxa,maxidxb] = sort(size_coordi,'descend');
     [mina,minidxb] = min(size_coordi);
     assign_ok = false;
     count_array = 1;
     while(~assign_ok && count_array <= size(maxidxb,2))
         if(minidxb~=maxidxb(count_array))
            [ccoordiy,ccoordix,idx,assign_ok]=region_assign(ccoordiy,ccoordix,idx,minidxb,maxidxb(count_array),1,L_range*2);
           
            size_coordi(minidxb) = size(ccoordiy{minidxb},2);
            size_coordi(maxidxb(count_array)) = size(ccoordiy{maxidxb(count_array)},2);
             count_array = count_array+1;
            
         end
     end     
 end
 
 
coordiy = ccoordiy;
coordix = ccoordix;    
idxx = idx;
% coordiy = 

if(Debug==1)
    pause;
end
        
 %%%%%%%%%%%%%%%%%%%%%%% 개별 나눠진 순찰경로(nodes)를 기반으로 개별 로봇의 순찰 경로 할당 %%%%%%%%%%%%%%%%%%%%%%%
for jj = 1:size(coordix,2)
        %% 0807변경
         [shortestPath,shortestPathLength] = onewaypath(binaryImage,coordix{jj}(:),coordiy{jj}(:),dmat,idxx{jj}(:),L_range,Rangeconstant,2); 

%                  if(shortestPathLength > PathLength1(jj)*2)
%                      continue;
%                  end
         coordiy{jj} = coordiy{jj}(shortestPath);
         coordix{jj} = coordix{jj}(shortestPath);
         idxx{jj} = idxx{jj}(shortestPath);
         PathLength1(jj) = shortestPathLength;    
end 
fprintf('Path Assignment completed\n');

    
 %%%%%%%%%%%%%%%%%%%%%%% 개별 나눠진 순찰경로(nodes)를 기반으로 개별 로봇의 순찰 경로 할당 %%%%%%%%%%%%%%%%%%%%%%%
   
%% Text로 로봇 경로 추출
      
       if(Debug==1)
            SaveRobotPose(coordix,coordiy);
           testdebug1 = 2       
           pause;
       end
%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%실제 로봇 순찰 경로 시간에 따라 보여주기%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%                            
 %% 결과 disp 
   for j=1:1000000
                count = 1;
                if(POI == 1)
                    if(j < size(POI_y,2)+1)
                        plot(POI_y(j),POI_x(j),'Marker','o','color',[1 0 1],'LineWidth',4);   
                    end
                end
               for k = 1:size(coordiy,2)
               if(j < size(coordiy{k},2)+1 && k == 1)
                  plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 0 0],'LineWidth',4);   
                  count = 2;
               end
%                if(j >   size(coordiy{1},2)+1)
%                      break;
%                end 
                if(j < size(coordiy{k},2)+1 && k == 2)
                        plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[0 1 0],'LineWidth',4);   
                        count = 2;
                end
                 if(j < size(coordiy{k},2)+1 && k == 3)
                   plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[0 0 1],'LineWidth',4);    
                   count = 2;
                 end
                if(j < size(coordiy{k},2)+1 && k == 4)
                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[0 1 1],'LineWidth',4);    
                    count = 2;
                 end
                if(j < size(coordiy{k},2)+1 && k ==5)
                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 1 0],'LineWidth',4);    
                    count = 2;
                 end
                if(j < size(coordiy{k},2)+1 && k == 6)
                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 0 1],'LineWidth',4);    
                    count = 2;
                end
               end 
                if(count == 1)
                    break;
                end
%                pause(0.3);
   end
fprintf('Coverage Generation End\n');     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('kw Start\n');
save('kumohResults5','coordix','coordiy');
% pause;
%% 광운대학교 파트
[coordix, coordiy, coorditheta, ~] = Func_fovPathPlanning('map_outline.jpg', coordix, coordiy); % 1:출력, 2:생략
%% 재저장 %% 'unique_all_vertex'(모든 노드),'dmat'(거리맵),'ccoordix','ccoordiy','idx' (x,y,path)
 
overlapped_all_vertex = [];
unique_all_vertex = [];
coordiy(:,cellfun(@any,cellfun(@isnan,coordiy(1,:),'uni',false)))=[];
coordix(:,cellfun(@any,cellfun(@isnan,coordix(1,:),'uni',false)))=[];
idx{D_RobotNum+1} = [];

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
save('FinalResults','unique_all_vertex','dmat','ccoordix','ccoordiy','idx');

    for i=1:size(coordix,2)
          for j = 1:size(coordix{i},2)
             posx{i}(1,j) = 0.05*(  coordiy{i}(1,j)-1328);
             posy{i}(1,j) = 0.05*( 1655.5 - coordix{i}(1,j));
             if(j ==size(coordix{i},2) )
                 posTheta{i}(1,j) = atan2(posy{i}(1,1)-posy{i}(1,j),posx{i}(1,1)-posx{i}(1,j));
             else
                 posTheta{i}(1,j) = atan2(0.05*(1655.5 - coordix{i}(1,j+1))-posy{i}(1,j),0.05*(coordiy{i}(1,j+1)-1328)-posx{i}(1,j));
             end
          end
    end
    
    coordix = posx;
    coordiy = posy;
    coorditheta = posTheta;
fprintf('kw End\n');
save('kwResults','coordix','coordiy','coorditheta');
    