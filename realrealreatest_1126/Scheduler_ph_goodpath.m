function [coordix, coordiy, coorditheta] = Scheduler_ph_test1(robots)
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
cost = 300;
%% 센싱 거리 L_range %%
L_range = 172; %% 9번지도는 20임 %%13번지 도 15
%  L_range = 100; %% 9번지도는 20임 %%13번지 도 15
L_range_base = 20;
x0=0;
y0=0;
width=1200;
%%변수선언
height=1000;
Rangeconstant = 3;

%% 가용 로봇대수 %% - 3대이상 테스트 진행해야
% D_RobotNum =3;
% pathdivision_parameter = 4;
% % pathlength_parameter = 3;
% areacompensation_parameter= 5;
% D_RobotNum = 4;
% pathdivision_parameter = 4;
% pathlength_parameter = 3;
% areacompensation_parameter= 4.5;

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
binaryImage1 = Image_load(POI,1); %% image load. 

%  binaryImage = Image_load(POI,2);  %% image load.
J=rgb2gray(binaryImage1);
J = flipud(J);
J = imresize(J, [2800 3400]); 
J = J';
J=imtranslate(J,[-70, -50]); 
hold on;

% imagesc(binaryImage ); %% represent the image


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
    
    imshow(binaryImage ); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Node Generation%%%%%%%%%%%%%%%%%%%%%%%%%%%

gx = size(binaryImage,2);
gy = size(binaryImage,1);

%% parameter save

min_size = 3;
load('Parameter_scheduler_pre','unique_shortestPath','shortestPath','shortestPathlength','dmat','unique_all_vertex','unique_all_vertex_base','num_of_nodes');
fprintf('Parameter2 Save load completed\n');

% pause;


patrol_distance = shortestPathlength/(pathdivision_parameter*D_RobotNum);
%%dmat: graph 간 연결도, unique_all_vertex: graph node, shortestPath: TSP Solver결과, patrol_distance: 순찰 거리 조건 
save('precompute','dmat','unique_all_vertex','unique_shortestPath','patrol_distance');
fprintf('SaveFinish\n');

%%dmat: graph 간 연결도, unique_all_vertex: graph node, shortestPath: TSP Solver결과, patrol_distance: 순찰 거리 조건 
[coordix,coordiy,~,idxx,PathLength1] = pathdivision(dmat,unique_all_vertex,unique_shortestPath,patrol_distance);
   
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
%      pause(1);
 end
end
originaly =  coordiy;
originalx =  coordix;
originalidx = idxx;

 
size(originaly)
size(originalx)
size(originalidx)
 
 
%  pause;
[ccoordix,ccoordiy,idx,cluster_save]=Geometric_intersection(unique_all_vertex_base,coordix,coordiy,idxx,D_RobotNum,Debug_Plot,w,h,L_range,Rangeconstant)    ;

fprintf('Node Assignment completed\n');
 
% pause; 
 size_coordi = [];
 for i = 1:size(ccoordiy,2)
     size_coordi(i) = size(ccoordiy{i},2);
 end

 %% 영역 재보정
  count_max = 0;

 while((max(size_coordi) - min(size_coordi)) > min(size_coordi)/areacompensation_parameter)     %원래 /6
%      size_coordi
     [maxa,maxidxb] = sort(size_coordi,'descend');
     [mina,minidxb] = sort(size_coordi);
     maxidxb = randsample(maxidxb(1:2),2);
     minidxb = randsample(minidxb(1:2),1);
     assign_ok = false;    
     count_array = 1;
     count_local_max = 0;
     while(~assign_ok && count_array <= size(maxidxb,2))      
         if(minidxb~=maxidxb(count_array))
             minidxb
             maxidxb(count_array)
             count_array
            [ccoordiy,ccoordix,idx,assign_ok]=region_assign(ccoordiy,ccoordix,idx,minidxb,maxidxb(count_array),1,L_range*2);    
            % [coordiy,coordix,idxx,assign_ok]=region_assign(coordiy,coordix,idxx,i,or(end-j),1,L_range*2); 
            size_coordi(minidxb) = size(ccoordiy{minidxb},2);
            size_coordi(maxidxb(count_array)) = size(ccoordiy{maxidxb(count_array)},2);           
         end
         count_array = count_array+1;   
         count_local_max = count_local_max+1;
        if(count_local_max > 10)
            break;
        end        
     end 
     count_max = count_max+1;
      if(count_max > 1000)
          break;
      end
 end                                
                                      
fprintf('Node Assignment completed\n');
 
coordiy = ccoordiy;
coordix = ccoordix;
idxx = idx;                        
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
save('Debug_Test2','coordix','coordiy','dmat','idxx','L_range','Rangeconstant','binaryImage');