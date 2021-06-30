%% 금오 공대 데이터 로드 및 처리
clear; clc; close all;

"strat""strat""strat""strat""strat"

load('kumohResults.mat');
globalMap = imread('map_outline.jpg');
%imagesc(globalMap);
%axis equal;

global FOV_half dFOV radius_ch frame BW info color scolor dc;
info = imfinfo('map_outline.jpg');

clc;
%% 지도에 맞는 로봇의 parameter 설정 / control factor
% 1pixel = 4cm, 25pixel =1m, sensor 범위 15m
% FOV = 70, 7/18 pi

scolor = 70; dc = 1;

color = [[20, 255, 20];
        [255, 20, 20];
        [20, 20, 255];
        [20, 200, 200];
        [200, 200, 20];
        ];
    
FOV_half = pi*7/18/2;
dFOV = pi/720;
radius_ch = 375;
dvec = 60^2; % 15pixel 마다 1회 측정, Node를 skip할 개수(Node수/skip 만큼 남음)

BW = kw_BWfilter(rgb2gray(globalMap));       % rgb convert to gray, Normalizaion, morphology
frame = BW; skip = 1;                                     % copy

map_area = length(find(BW==255));
% empty_img = uint8(zeros(size(globalMap)));      % Node map

% frame     : color map img, showing coverage area
% BW        : frame's binary map img
% Node_map  : color map img, showing Node position
% empty_img : empty img


%% FOV를 고려하여 각각의 로봇 커버리지 계산
for robot = 1:5
    robot
    iy = coordiy{robot};
    ix = coordix{robot};
    
    newixnum = uint8(length(ix)/skip);
    new_ix = zeros(1,newixnum);
    new_iy = zeros(1,newixnum);
   
    % Node를 임의로 삭제
    for cont = 1:newixnum
        new_ix(cont) = ix((cont-1)*skip+1);
        new_iy(cont) = iy((cont-1)*skip+1);
    end
    
    coordix{robot} = new_ix;
    coordiy{robot} = new_iy;
    
    position = [new_ix(1), new_iy(1)];
    theta = 0;
    
    for cont = 2:length(new_ix)
        distans = [new_ix(cont), new_iy(cont)] - position;     
        delta = round(sum(distans.^2)/dvec);
        
        if(delta == 0 || delta > 100)
            "error : distans(delta)";
        else
            th = atan2(distans(2),distans(1)) - theta;
            for d = 1:delta
                position = position+distans/delta;
                theta = theta+th/delta;

                kw_sensing_greedy(position, theta);
            end
        end
    end
end

imshow(kw_imshow(BW, coordix, coordiy, [200, 10, 10]));
BW = kw_coverFilter(BW); 

cover_percent = kw_getCoveragePercent(map_area);

%% 각각의 로봇이 담당 영역 전체를 커버하도록 노드 수정

label=bwlabel(BW);
num_pole = max(label,[],'all');
Uncovered = zeros(num_pole,2);

for i = 1:num_pole
    [r, c] = find(label==i); % [x y];
    Uncovered(i,:) = round(sum([r c], 1)/length([r c]));
end

%% nearest

% for k = 1:num_pole                % pole의 중점 표시
%     for i = -15:15
%         for j = -15:15
%             BW(Uncovered(k,1)+i, Uncovered(k,2)+j) = 170;
%         end
%     end
% end

scoordixy = kw_sensing_center(coordix, coordiy);
nearest = zeros(num_pole,4);        % robot_num, Node index, x, y

for k = 1:num_pole                  % 근접 Node 찾기
    min = 3000^2;                   % 맵크기상 넘을 수 없음
    
    for robot = 1:5
        ix = coordix{robot};
        iy = coordiy{robot};
        sixy = scoordixy{robot};
        
        for cont = 1:length(sixy)
            des = sqrt(sum((sixy(cont)-Uncovered(k,:)).^2));
            if min > des
                min = des;
                nearest(k,:) = [robot, cont, ix(cont), iy(cont)];
            end
        end
    end
    dp = Uncovered(k,:) - nearest(k,3:4);
    kw_sensing(nearest(k,3:4), atan2(dp(2), dp(1)), 120);
end

%% 새로운 계산


%% Greedy
ds = 5;      % 120cm

for k = 1:num_pole
    position = [1798, 2533];
    th = 0;

    while(1)
        position = position + ds*[cos(th), sin(th)];
        th = 1.01*kw_sensing_greedy(position, th);

        for i = -16:14
            for j = -16:14
                BW(uint16(position(1)+i)+1, uint16(position(2)+j)+1) = 170;
            end
        end

        imshow(BW);
    end
end








