function scheduling_response    
    clear; clc; close all;
    global transParam;
    
    fprintf('scheduling_response start\n');
    
    % exe test 1
    c1 = fix(clock);
	year = num2str(c1(1));
	month = num2str(c1(2));
	day = num2str(c1(3));
	hour = num2str(c1(4));
	minute = num2str(c1(5));
	second = num2str(c1(6));
	c_str1 = [year '-' month '-' day ' ' hour ':' minute ':' second];          
	fileName = strcat('exe test', '.txt');
	fid = fopen(fileName, 'a');
	fprintf(fid, c_str1);
	fprintf(fid, ' ');
	fprintf(fid, '1');
	fprintf(fid, '\n');
	fclose('all'); 
    
%% Parameter initialization
    
    % ph MQTT broker server
    % transParam.host = 'tcp://220.81.76.111';
    
    % test MQTT broker server
    transParam.host = 'tcp://192.168.0.19';
    
    transParam.revtopic = '/mams/+/scheduler/req';
    
%% Subscribe
    
	% Create mqtt connection.
    transParam.subMQTT = mqtt(transParam.host, 'Port', 1883);

    % Subscibe to a topicde
    transParam.subscribe = subscribe(transParam.subMQTT, transParam.revtopic, ...
        'Qos', 0);

    transParam.subscribe.Callback = @subMQTT_Callback;
    % transParam.subscribe.Callback = subMQTT_Callback(topic, msg);
    
    % Callback function that checks the topic and message
    % each time a new message is received.
    function subMQTT_Callback(topic, msg)
 
        flag = 0;
        if(topic == '/mams/ph/scheduler/req')
            flag = 1; % ph
            transParam.transtopic = '/mams/ph/scheduler/res';
            fprintf('\n%s\n', topic);
        elseif(topic == '/mams/gw/scheduler/req')
            flag = 2; % gw
            transParam.transtopic = '/mams/gw/scheduler/res';
            fprintf('\n%s\n', topic);
        else
            flag = 3; % not valid
            transParam.transtopic = '';
            fprintf('\nThis topic is not valid : %s\n', topic);
        end

        fprintf('%s\n', msg);
            
%% log file output (sub)

        c = fix(clock);
        year = num2str(c(1));
        month = num2str(c(2));
        day = num2str(c(3));
        hour = num2str(c(4));
        minute = num2str(c(5));
        second = num2str(c(6));
        c_str = [year '-' month '-' day ' ' hour ':' minute ':' second];
        c_str = jsonencode(c_str);

        fileName = strcat('log file','.json');
        fid = fopen(fileName, 'a');
        fprintf(fid, c_str);
        fprintf(fid, ' ');
        fprintf(fid, topic);
        fprintf(fid, ' ');
        fprintf(fid, msg);
        fprintf(fid, '\n');
        fclose('all');

%% Jsondecode

        transParam.revstruct = jsondecode(msg);
        
%% Coordinate initialization

        % Robot1
        coordix{1} = [0.0, 0.0, 0.0];
        coordiy{1} = [0.0, 0.0, 0.0];
        coorditheta{1} = [0.0, 0.0, 0.0];

        % Robot2
        coordix{2} = [0.0, 0.0, 0.0];
        coordiy{2} = [0.0, 0.0, 0.0];
        coorditheta{2} = [0.0, 0.0, 0.0];

        % Robot3
        coordix{3} = [0.0, 0.0, 0.0];
        coordiy{3} = [0.0, 0.0, 0.0];
        coorditheta{3} = [0.0, 0.0, 0.0];

        % Robot4
        coordix{4} = [0.0, 0.0, 0.0];
        coordiy{4} = [0.0, 0.0, 0.0];
        coorditheta{4} = [0.0, 0.0, 0.0];
            
        % Robot5
        coordix{5} = [0.0, 0.0, 0.0];
        coordiy{5} = [0.0, 0.0, 0.0];
        coorditheta{5} = [0.0, 0.0, 0.0];
            
        % Robot6
        coordix{6} = [0.0, 0.0, 0.0];
        coordiy{6} = [0.0, 0.0, 0.0];
        coorditheta{6} = [0.0, 0.0, 0.0];

        % RobotPoses
        % for i = 1:size(coordix, 2)
            % robotPoses{i}(:,1) = coordix{i}(:);
            % robotPoses{i}(:,2) = coordiy{i}(:);
            % robotPoses{i}(:,3) = coorditheta{i}(:);
        % end

%% Date indexing

        % reqType = transParam.revstruct.reqType;
        % agentId = transParam.revstruct.agentId;
        % posx = transParam.revstruct.posx;
        % posy = transParam.revstruct.posy;
    
        % 
            % robots{transParam.revstruct.robots(i).robotId}(1) = transParam.revstruct.robots(i).robotId;
            % robots{transParam.revstruct.robots(i).robotId}(2) = transParam.revstruct.robots(i).posx;
            % robots{transParam.revstruct.robots(i).robotId}(3) = transParam.revstruct.robots(i).posy;
            % robots{transParam.revstruct.robots(i).robotId}(4) = transParam.revstruct.robots(i).curTargetx;
            % robots{transParam.revstruct.robots(i).robotId}(5) = transParam.revstruct.robots(i).curTargety;
            % robots{transParam.revstruct.robots(i).robotId}(6) = transParam.revstruct.robots(i).curTargetTheta;
            % robots{transParam.revstruct.robots(i).robotId}(7) = transParam.revstruct.robots(i).prevTargetx;
            % robots{transParam.revstruct.robots(i).robotId}(8) = transParam.revstruct.robots(i).prevTargety;
            % robots{transParam.revstruct.robots(i).robotId}(9) = transParam.revstruct.robots(i).prevTargetTheta;
        % end

%% ReqType
    
        count = 0;
        True_ragentId = transParam.revstruct.agentId;
        True_robotId_array = [];
            
        % save the True_robotId_array
        for i=1:size(transParam.revstruct.robots,1)
            True_robotId_array = [True_robotId_array transParam.revstruct.robots(i).robotId];
        end 
            
        if (transParam.revstruct.reqType == "INIT")
            for i=1:size(transParam.revstruct.robots,1)
                transParam.revstruct.robots(i).robotId = i;
            end
            % [coordix, coordiy, coorditheta] = Scheduler_ph(transParam.revstruct.robots);
            % [coordix, coordiy, coorditheta] = Scheduler_gw(transParam.revstruct.robots);
            if(flag == 1) % ph
                if(size(transParam.revstruct.robots,1) == 1)
                    load('kwResults_ph1.mat');
                elseif(size(transParam.revstruct.robots,1) == 2)
                    load('kwResults_ph2.mat');
                elseif(size(transParam.revstruct.robots,1) == 3)
                    load('kwResults_ph3.mat');
                elseif(size(transParam.revstruct.robots,1) == 4)
                    load('kwResults_ph4.mat');
                elseif(size(transParam.revstruct.robots,1) == 5)
                    load('kwResults_ph5.mat');
                elseif(size(transParam.revstruct.robots,1) == 6)
                    load('kwResults_ph6.mat');
                end
            elseif(flag == 2) % gw
                if(size(transParam.revstruct.robots,1) == 1)
                    load('kwResults_gw1.mat');
                elseif(size(transParam.revstruct.robots,1) == 2)
                    load('kwResults_gw2.mat');
                elseif(size(transParam.revstruct.robots,1) == 3)
                    load('kwResults_gw3.mat');
                elseif(size(transParam.revstruct.robots,1) == 4)
                    load('kwResults_gw4.mat');
                elseif(size(transParam.revstruct.robots,1) == 5)
                    load('kwResults_gw5.mat');
                elseif(size(transParam.revstruct.robots,1) == 6)
                    load('kwResults_gw6.mat');
                end
            end
        else % ABNORMAL OR LOWBAT
            for i=1:size(transParam.revstruct.robots,1)
                if((transParam.revstruct.agentId > transParam.revstruct.robots(i).robotId))
                    transParam.revstruct.robots(i).robotId = i;
                else
                    if(count == 0)
                        count = count+1;
                        transParam.revstruct.agentId = i;
                    end
                    transParam.revstruct.robots(i).robotId = i+1;
                end
            end
            if(count == 0 &&  transParam.revstruct.agentId~=0)
                transParam.revstruct.agentId = size(transParam.revstruct.robots,1)+1;
            end
                
            if (transParam.revstruct.reqType == "ABNORMAL") % ABNORMAL
                if(flag == 1) % ph
                    % 대략 size(transParam.revstruct.robots,1)이 몇이냐에 따라서
                    % 1~5가 있을텐데... 1의 경우는 mat파일(2)이런식으로 가고
                    % 리스케줄러안에서 robots size 재서 1일경우 load_2 이런식으로 간다.
                    % 그럼 여기서는 코드 변경없이 리스케줄러에서 변경해야하네.
                    [coordix, coordiy, coorditheta] = Rescheduler_ph(transParam.revstruct.agentId, transParam.revstruct.robots);
                elseif(flag == 2) % gw
                    [coordix, coordiy, coorditheta] = Rescheduler_gw(transParam.revstruct.agentId, transParam.revstruct.robots);
                end
                coordix{transParam.revstruct.agentId} = [transParam.revstruct.posx, transParam.revstruct.posx];
                coordiy{transParam.revstruct.agentId} = [transParam.revstruct.posy, transParam.revstruct.posy];
                coorditheta{transParam.revstruct.agentId} = [0, 0];
                fprintf('%f\n', transParam.revstruct.posx);
                fprintf('%f\n', transParam.revstruct.posy);   
            elseif (transParam.revstruct.reqType == "LOWBAT") % LOWBAT
                if(flag == 1) % ph
                    [coordix, coordiy, coorditheta] = Rescheduler_ph(transParam.revstruct.agentId, transParam.revstruct.robots);
                elseif(flag == 2) % gw
                    [coordix, coordiy, coorditheta] = Rescheduler_gw(transParam.revstruct.agentId, transParam.revstruct.robots);
                end
            end
        end
   
%% wayPoints
        
        for i = 1:size(coordix, 2)           
            wayPoints_posx{i} = num2cell(coordix{i});
            wayPoints_posy{i} = num2cell(coordiy{i});
            wayPoints_postheta{i} = num2cell(coorditheta{i});
        end
        cccount = 0;
        for i = 1:size(wayPoints_posx, 2)
            if(isempty(wayPoints_posx{i}))
                continue;
            end
            cccount = cccount + 1;
            wwayPoints_posx{cccount} = wayPoints_posx{i};
            wwayPoints_posy{cccount} = wayPoints_posy{i};
            wwayPoints_postheta{cccount} = wayPoints_postheta{i};
        end

%% robotId

        robotId = [];           
        if (transParam.revstruct.reqType == "INIT")
            robotId = True_robotId_array;
        elseif (transParam.revstruct.reqType == "ABNORMAL")
            count = 0;
            for i = 1:size(True_robotId_array,2)
                if(True_ragentId > True_robotId_array(i)) 
                    robotId = cat(2, robotId, True_robotId_array(i));  
                elseif(True_ragentId < True_robotId_array(i)) 
                    if(count == 0 && True_ragentId~=0)
                        count = count+1;
                        robotId = cat(2, robotId, True_ragentId);   
                        robotId = cat(2, robotId, True_robotId_array(i));   
                    else
                        robotId = cat(2, robotId, True_robotId_array(i));   
                    end
                end
            end
            if(count == 0 && True_ragentId~=0)
                count = count+1;
                robotId = cat(2, robotId, True_ragentId);   
            end
        elseif (transParam.revstruct.reqType == "LOWBAT")
            robotId = True_robotId_array;
        end
     
%% Jsonencode

        robotId = num2cell(robotId)
        robots2 = struct("robotId", robotId);
            
        for i = 1:size(robotId, 2)
            robots2(i).wayPoints = struct("posx", wwayPoints_posx{i}, "posy", ...
            wwayPoints_posy{i}, "postheta", wwayPoints_postheta{i});
        end

        transParam.transstruct = struct("robots", robots2);
        transParam.sendmsg = jsonencode(transParam.transstruct);
  
%% Publish

        % Create mqtt connection.
        transParam.pubMQTT = mqtt(transParam.host, 'Port', 1883);

        % Create topic and message.        
        % Publish a message to a topic.
        publish(transParam.pubMQTT, transParam.transtopic, transParam.sendmsg, 'Qos', 0);
            
        fprintf('%s\n', transParam.sendmsg);
        
        % exe test 10
        c1 = fix(clock);
        year = num2str(c1(1));
        month = num2str(c1(2));
        day = num2str(c1(3));
        hour = num2str(c1(4));
        minute = num2str(c1(5));
        second = num2str(c1(6));
        c_str1 = [year '-' month '-' day ' ' hour ':' minute ':' second];
        fileName = strcat('exe test', '.txt');
        fid = fopen(fileName, 'a');
        fprintf(fid, c_str1);
        fprintf(fid, ' ');
        fprintf(fid, '10');
        fprintf(fid, '\n');
        fclose('all'); 

%% log file output (pub)

        c = fix(clock);
        year = num2str(c(1));
        month = num2str(c(2));
        day = num2str(c(3));
        hour = num2str(c(4));
        minute = num2str(c(5));
        second = num2str(c(6));
        c_str = [year '-' month '-' day ' ' hour ':' minute ':' second];
        c_str = jsonencode(c_str);

        fileName = strcat('log file','.json');
        fid = fopen(fileName, 'a');
        fprintf(fid, c_str);
        fprintf(fid, ' ');
        fprintf(fid, transParam.transtopic);
        fprintf(fid, ' ');
        fprintf(fid, transParam.sendmsg);
        fprintf(fid, '\n');
        fclose('all');
            
    end
end