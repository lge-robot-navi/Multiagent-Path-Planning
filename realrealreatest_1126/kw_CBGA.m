function Tree = kw_CBGA(position, th, cell_index, ds, gain, show_flag)
global sensorParam coveragedMap binaryMap;
    
    i = -16:14;
    init_flag = 30; flag = init_flag; cont = 0; Node = position;
    save_map = coveragedMap;
   % position = position + (sensorParam.dvec*3)*[cos(th), sin(th)];
    Tree =  position';
    if show_flag
        coveragedMap( uint16(position(1)+i)+1, uint16(position(2)+i)+1 ) = 170;
    end
    
%      nearCell이 coveraged 되었는지 검사학하기

    while flag
        cont = cont+1;
        position = position + ds*[cos(th), sin(th)];
        [new_theta, flag_] = kw_sensing_greedy(position, th); % new_theta는 변화량
        
        if isnan(new_theta)      % 벽에 박은 경우 % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
            Tree = Tree(:,1:length(Tree)-1); % 이전 Node 제거
            
            position = Node;
            coveragedMap = save_map;
            cont = 0; flag = 100;
            label = save_map;
            
            label(label>230)=255; label(label<230)=0;
            label=bwlabel(imclose(label,strel('disk',20)));
            numCluster = max(label,[],'all');           % 클러스터의 갯수
            
            Cluster_num = 0;
            min = 3000^2;
            for k = 1:numCluster
                [r, c] = find(label==k); % [x y];
                Uncovered = round(sum([r c], 1)/length([r c]));    % 중점
                for t = Tree
                    distance = sum((Uncovered - t').^2);      % distance 변수 재활용
                    if min > distance
                        index = kw_point2ind(Node, Uncovered); 

                        wflag = 1;
                        for s = index
                            if (binaryMap(s) < 100)        % 장애물과 만남
                                wflag = 0;
                                
                                break;
                            end
                        end
                        if wflag % 갈 수 있어야함
                            min = distance;
                            position = t';
                            Cluster_num = k;
                            Cluster = Uncovered; % cell이 너무 작아서 중점으로
                        end
                    end
                end  
            end
            
            if Cluster_num == 0
                return
            end
            
            if show_flag
                coveragedMap( uint16(position(1)+i)+1, uint16(position(2)+i)+1 ) = 210;
                coveragedMap( uint16(Cluster(1)+i)+1, uint16(Cluster(2)+i)+1 ) = 30;
            end
            
            distance = Cluster - position;              % distance 변수 재활용
            th = atan2(distance(2),distance(1));
            
        else                    % 벽에 안박은 경우  % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

            if abs(new_theta) > 180
                new_theta = new_theta - 360*new_theta/abs(new_theta);
            end
            th = th + gain*new_theta; % GAIN

            if mod(cont, 20) == 0 % 20번 이동마다 1Node
                Node = position;
                coveragedMap( uint16(Node(1)+i)+1, uint16(Node(2)+i)+1 ) = 170;
                Tree = [Tree, Node'];
                save_map = coveragedMap;
            end
                    
            if flag_>0
                flag = init_flag; % 30 번의 이동
            else
                flag = flag - 1;
            end
        end
        % 공통
        if show_flag
            x1= position(1) +30*cos(th);
            y1= position(2) +30*sin(th);
            x2= position(1) +30*cos(th+3*pi/4);
            y2= position(2) +30*sin(th+3*pi/4);
            x3= position(1) +30*cos(th-3*pi/4);
            y3= position(2) +30*sin(th-3*pi/4);

            image(coveragedMap);
            patch([y1,y2,y3,y1],[x1,x2,x3,x1],[1, 0, 0]);
            pause(0.0001);
        end
%         title("The Robot number is " + robot,'FontSize', 22);
%         imshow(BW);
    end
    
    coveragedMap = save_map;
    index = [];
    ncell_index = [];
    for s = cell_index
        if coveragedMap(s) == 255
            index = [index, [mod(s,sensorParam.H)+1, floor(s/sensorParam.H)]'];
            ncell_index = [ncell_index, s];
        end
     end
    
    min = sum((Tree(:,1)' -  position).^2);
    Cluster = [];
    for ks = index
        for t = Tree
            dleta = ks' - t';
            distance = sum(dleta.^2);      % distance 변수 재활용
            if (min > distance) && (distance > sensorParam.mindis)
                index = kw_point2ind(ks', t'); 

                wflag = 1;
                for s = index
                    if (binaryMap(s) < 100)        % 장애물과 만남
                        wflag = 0;
                        break;
                    end
                end
                if wflag % 갈 수 있어야함

                    min = distance;
                    vec = dleta;
                end
            end
        end  
    end
    
    if ~isempty(Cluster)             % if is not zero
        th = atan2(vec(2),vec(1));
        Tree = [Tree, kw_greedyCoverage(Tree(end), th, ncell_index, ds, gain, show_flag)];
    end
    
    % 30 번의 이동 > 1개의 Node는 필요없음
    Tree = round(Tree);
    Tree = Tree(:,1:end-1);                                        % 3 789 4
    % Tree = [Tree(:,1:end-1), fliplr(Tree(:,1:end-2))];            % 3 78987 34
end
