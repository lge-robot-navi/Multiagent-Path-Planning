function kw_topic(transhost,revtopic,transtopic,portNum)
    global transParam;
    
    
    %request를 받았을때 Root의 ID와 waypoint들을 송신해주는 함수 입니다.
    
%input :
%     host                :정보를 주어야하는 곳의 주소정보
%     revtopic            :정보를 받을 토픽 이름     '/mams/ph/scheduler/req'; 
%     transtopic          :정보를 전송할 토픽 이름   '/mams/ph/scheduler/res'
%     RobotID             :로봇의 ID 
%     coordix             :waypoint의 x좌표
%     coordiy             :waypoint의 y좌표
%     theta               :waypoint의 theta
% 
% output
%        Json형식의 txt

% example: 
%  
% input:
%     host = 'tcp://localhost'
%     revtopic   = '/mams/ph/scheduler/req'
%     transtopic = '/mams/ph/scheduler/res'
%     RobotID   = [1 2 3 4 5];
%     coordix   = [ 24 23 42 52 55];
%     coordiy   =  [ 12 44 22 33 22 ];
%     theata     =  [0.22 0.52 0.25 0.25 0.77];

% output =
% 
%     '[{"robotID":1,"posx":[24,23,42,52,55],"posy":[12,44,22,33,22],"postheata":[0.22,0.52,0.25,0.25,0.77]},
%     {"robotID":2,"posx":[24,23,42,52,55],"posy":[12,44,22,33,22],"postheata":[0.22,0.52,0.25,0.25,0.77]},
%     {"robotID":3,"posx":[24,23,42,52,55],"posy":[12,44,22,33,22],"postheata":[0.22,0.52,0.25,0.25,0.77]},
%     {"robotID":4,"posx":[24,23,42,52,55],"posy":[12,44,22,33,22],"postheata":[0.22,0.52,0.25,0.25,0.77]},
%     {"robotID":5,"posx":[24,23,42,52,55],"posy":[12,44,22,33,22],"postheata":[0.22,0.52,0.25,0.25,0.77]}]'
    
%% 
    %initialize example 
%     host = 'tcp://localhost'
%     revtopic   = '/mams/ph/scheduler/req'
%     transtopic = '/mams/ph/scheduler/res'

%     RobotID   = [1 2 3 4 5];
%     coordix   = [ 24 23 42 52 55];
%     coordiy   =  [ 12 44 22 33 22 ];
%     theata     =  [0.22 0.52 0.25 0.25 0.77];


%%  Test Section 
%     global transParam;
%     KWloadSensorParam();

%     load('kumohResults114.mat');
% 
%     RobotID   = [1 2 3 4 5];
%     coordix   = [ 24 23 42 52 55];
%     coordiy   =  [ 12 44 22 33 22 ];
%     theata     =  [0.22 0.52 0.25 0.25 0.77];
% 
%     transParam.transhost = 'tcp://192.168.0.31';
%     transParam.revhost = 'tcp://localhost';                                
%     transParam.revtopic   = '/mams/ph/scheduler/req';
%     transParam.transtopic  =  '/mams/ph/scheduler/res';
%     transParam.sendMsg = ''; 
%     transParam.subMQTT = '';
%     transParam.KWsubscrible = '';
%     transParam.revflags1 = 'A';
%     transParam.revflags2 ='B';

    transParam.transhost = transhost;
    transParam.revtopic   = revtopic;
    transParam.transtopic = transtopic;
    transParam.portNum = portNum;

    
%     RobotID = 1: size(coordix,2);
    
%     for i=1:size(RobotID,2)
%         Waypoint(RobotID(i)).robotID = RobotID(i);
%         Waypoint(RobotID(i)).posx = coordix;
%         Waypoint(RobotID(i)).posy = coordiy;
%         Waypoint(RobotID(i)).postheata   = theata;
%     end
%    
%     output = jsonencode(struct(Waypoint));
%     transParam.sendMsg  = output;
    
%     myMQTT = mqtt(transParam.transhost);
%     % Publish a message to a topic
%     publish(myMQTT, transParam.transtopic, transParam.sendMsg);

    transParam.subMQTT = mqtt(transParam.transhost);
    transParam.KWsubscrible = subscribe(transParam.subMQTT,transParam.revtopic,'QoS' ,2);
    transParam.KWsubscrible.Callback = @myMQTT_Callback;
    
% mySub.Callback = myMQTT_Callback(topic, msg);

% send a test message to check callback
% publish(myMQTT, 'myTopic', 'testMessage03');
% txt = jsonencode(data)
% publish(myMQTT, 'output','good');

% 매트랩에서는 동적할당을 지원하지 않기 때문에 전역 변수로 사용
    function myMQTT_Callback(topic, msg)
        fprintf('MQTT callback topic "%s": "%s"\n', topic, msg);
        transParam.recVal = jsondecode(msg);   % 두번해줘야지 완벽히 변형되는것같음 처음에는 \없애고 다음에 완전변형
        disp("Recieved Msg"); 
        
        
        % input 반영하기 
        fileName = strcat(transParam.recVal.reqType,'Input.json');
        fileID = fopen(fileName,'w');
        fprintf(fileID,msg);
        fclose('all'); 

        %index가 몇번쨰 로봇인지 말해주고 없는 로봇은 []처리 ~
%         coordix= {};
%         coordiy = {};
%         coorditheta ={}; %명시적 변수 지정
        switch(transParam.recVal.reqType)
            case 'INIT'
                disp('INIT');
                %금오공대 코드 실행  (필요정보 : )
                %광운대학교 코드 실행  (필요정보 cooridx cooriy)
                %통합코드 -- 스케줄링 --> 오래걸리니까 데이터를 Load해서 사용
                %Load할때 스케줄링에 저장한값 load하는 함수로 바꿈
                [coordix, coordiy, coorditheta] = Scheduler(transParam.recVal.robots); 
                
                for i=1:size(transParam.recVal.robots,1)
                    for j = 1:size(coordiy{transParam.recVal.robots(i).robotId},2)
                        % 새로운 x좌표 = 0.05*(coordiy-1328)
                        posx(j) = coordix{transParam.recVal.robots(i).robotId}(1,j);
                        % 새로운 y좌표 = 0.05*(1655.5-coordix)
                        posy(j)  = coordiy{transParam.recVal.robots(i).robotId}(1,j);
                        posTheta(j) = coorditheta{transParam.recVal.robots(i).robotId}(1,j);
                        wayPoints(j) = struct('posx', posx(j),'posy', posy(j),'posTheta', posTheta(j));
                    end
                    SubStruct(i) = struct("robotId", transParam.recVal.robots(i).robotId,"wayPoints", wayPoints(1,1:size(coordiy{transParam.recVal.robots(i).robotId},2)));
                end
                disp('INIT Excuted!');
            case 'ABNORMAL'
                % 비정상 상활(적이 나타난 상황?)
                disp('ABNORMAL');
                %금오공대 코드 실행  (필요정보 : )
%                 [coordix, coordiy, coorditheta] = Scheduling(robots);
                %광운대학교 코드 실행  (필요정보 cooridx cooriy)
                [coordix, coordiy, coorditheta] = Rescheduler(transParam.recVal.agentId, transParam.recVal.robots);
                
                
                for i=1:size(coordix,2)-1             %로봇이 4대니까
                    if( isempty(coordiy{i}))
                         posx(1:2) = transParam.recVal.posx;
                         posy(1:2) = transParam.recVal.posy;
                         posTheta(1:2) = 0;
                         wayPoints(1:2) = struct('posx', posx(1:2),'posy', posy(1:2),'posTheta', posTheta(1:2));
                         SubStruct(i) = struct(struct("robotId", i,"wayPoints", wayPoints(1:2)));
                         continue;
                    end
                    for j = 1:size(coordiy{i},2)
                        % 새로운 x좌표 = 0.05*(coordiy-1328)
                        posx(j) = coordix{i}(1,j);
                        % 새로운 y좌표 = 0.05*(1655.5-coordix)
                        posy(j)  = coordiy{i}(1,j);
                        posTheta(j) = coorditheta{i}(1,j);
                        wayPoints(j) = struct('posx', posx(j),'posy', posy(j),'posTheta', posTheta(j));
                    end
                    SubStruct(i) = struct("robotId",i,"wayPoints", wayPoints(1,1:size(coordiy{i},2)));
                end
                
                
                
                disp('ABNORMAL Excuted!');
            case 'LOWBAT'
                % 베터리가 없는 상황
                disp('LOWBAT');
                [coordix, coordiy, coorditheta] = Rescheduler(transParam.recVal.agentId, transParam.recVal.robots);
                
                for i=1:size(transParam.recVal.robots,1)
                    for j = 1:size(coordiy{transParam.recVal.robots(i).robotId},2)
                        % 새로운 x좌표 = 0.05*(coordiy-1328)
                        posx(j) = coordix{transParam.recVal.robots(i).robotId}(1,j);
                        % 새로운 y좌표 = 0.05*(1655.5-coordix)
                        posy(j)  = coordiy{transParam.recVal.robots(i).robotId}(1,j);
                        posTheta(j) = coorditheta{transParam.recVal.robots(i).robotId}(1,j);
                        wayPoints(j) = struct('posx', posx(j),'posy', posy(j),'posTheta', posTheta(j));
                    end
                    SubStruct(i) = struct("robotId", transParam.recVal.robots(i).robotId,"wayPoints", wayPoints(1,1:size(coordiy{transParam.recVal.robots(i).robotId},2)));
                end
                %금오공대 코드 실행  (필요정보 : )
%                 [coordix, coordiy, coorditheta] = Scheduling(robots);
                %광운대학교 코드 실행  (필요정보 cooridx cooriy)

                disp('LOWBAT Excuted!');
        end

        %load('kwResults.mat')
        % Jason type transformation
        
        MsgStruct = struct("robots",SubStruct(1,:));
        transParam.sendMsg= jsonencode(MsgStruct);
        
        myMQTTFunc = mqtt(transParam.transhost);
        publish(myMQTTFunc, transParam.transtopic,transParam.sendMsg,'Qos', 0);
        disp('Publish Finished!');
% 
%         Jason formation 변형
        fileName = strcat(transParam.recVal.reqType,'.json');
        fileID = fopen(fileName,'w');
        fprintf(fileID,transParam.sendMsg);
        fclose('all'); 
    end
end
%% Test module
%kw_topic('tcp://220.81.76.111:1883','/mams/ph/scheduler/req','/mams/ph/scheduler/res',coordix,coordiy,10,'A','B');
%{"reqType":"INIT","agentId":0,"posx":0,"posy":0, "robots":[{"robotId":1,"posx":0,"posy":0,"curTargetx": 0,"curTargety": 0,"curTargetTheta":0,"prevTargetx":0,"prevTargety":0,"prevTargetTheta":0},{"robotId":2,"posx":0,"posy":0,"curTargetx": 0,"curTargety": 0,"curTargetTheta":0,"prevTargetx":0,"prevTargety":0,"prevTargetTheta":0},{"robotId":3,"posx":0,"posy":0,"curTargetx": 0,"curTargety": 0,"curTargetTheta":0,"prevTargetx":0,"prevTargety":0,"prevTargetTheta":0}]}

%         switch(msg)
%             case transParam.revflags1
%                 disp("First");
%                 myMQTTFunc = mqtt(transParam.transhost);
%                 publish(myMQTTFunc, transParam.transtopic,transParam.sendMsg);
%             case transParam.revflags2
%                 disp("Second");
%                 myMQTTFunc = mqtt(transParam.transhost);
%                 publish(myMQTTFunc, transParam.transtopic,transParam.sendMsg);
%         end

% 
%     wayPoints_posx = [0;10;20;30;40;50;60;70];
%     wayPoints_posy = [0;10;20;30;40;50;60;70];
%     wayPoints_posTheta = [0;10;20;30;40;50;60;70];
%     wayPoints = table(wayPoints_posx,wayPoints_posy,wayPoints_posTheta);
% 
%     S = struct("robots", struct("robotId", 1, "wayPoints", wayPoints));
%     s = jsonencode(S);


%% 
% 백터로 나타내기 실패

% iter = 1:size(transParam.recVal.robots,1)

% robotNum(iter) = transParam.recVal.robots(iter).robotId

% 
%         for i=1:size(transParam.recVal.robots,1)
%             robotId(i) = transParam.recVal.robots(i).robotId;    %패스 플래닝 할 로봇 넘버
%             robots{robotId(i)}(1,:) = transParam.recVal.robots(i).posx;
%             robots{robotId(i)}(2,:) = transParam.recVal.robots(i).posy;
%             robots{robotId(i)}(3,:) = transParam.recVal.robots(i).curTargetx;
%             robots{robotId(i)}(4,:) = transParam.recVal.robots(i).curTargety;
%             robots{robotId(i)}(5,:) = transParam.recVal.robots(i).curTargetTheta;
%             robots{robotId(i)}(6,:) = transParam.recVal.robots(i).prevTargetx;
%             robots{robotId(i)}(7,:) = transParam.recVal.robots(i).prevTargety;
%             robots{robotId(i)}(8,:)= transParam.recVal.robots(i).prevTargetTheta;
%         end



%         transParam.recVal = jsondecode(msg);   % 두번해줘야지 완벽히 변형되는것같음 처음에는 \없애고 다음에 완전변형
%         disp("Recieved Msg"); 
%         
%         agentId = transParam.recVal.agentId;
%         reqType = transParam.recVal.reqType;
%         
%         robots = {};                % Cell 형 사전 할당
%         %초기화과정 필요한 정보추출 
%         for i=1:size(transParam.recVal.robots,1)
%             robotId(i) = transParam.recVal.robots(i).robotId;    %패스 플래닝 할 로봇 넘버
%             robots{robotId(i)}.posx = transParam.recVal.robots(i).posx';
%             robots{robotId(i)}.posy = transParam.recVal.robots(i).posy';
%             robots{robotId(i)}.curTargetx = transParam.recVal.robots(i).curTargetx';
%             robots{robotId(i)}.curTargety = transParam.recVal.robots(i).curTargety';
%             robots{robotId(i)}.curTargetTheta = transParam.recVal.robots(i).curTargetTheta';
%             robots{robotId(i)}.prevTargetx= transParam.recVal.robots(i).prevTargetx';
%             robots{robotId(i)}.prevTargety = transParam.recVal.robots(i).prevTargety';
%             robots{robotId(i)}.prevTargetTheta= transParam.recVal.robots(i).prevTargetTheta';
%         end

%% 받은값 바꿔버리기
%         for i=1:size(transParam.recVal.robots,1)
%             for j = 1:size(transParam.recVal.robots(i).wayPoints,1)
%                  % 새로운 x좌표 = 0.05*(coordiy-1328)
%                  posx(j) = 0.05*(1328 - transParam.recVal.robots(i).wayPoints(j).posx);
%                  % 새로운 y좌표 = 0.05*(1655.5-coordix)
%                  posy(j)  = 0.05*(1655.5 - transParam.recVal.robots(i).wayPoints(j).posy);
%                  posTheta(j) = transParam.recVal.robots(i).wayPoints(j).posTheta;
%                  wayPoints(j) = struct('posx', posx(j),'posy', posy(j),'posTheta', posTheta(j));
%             end
%             SubStruct(i) = struct("robotId", transParam.recVal.robots(i).robotId,"wayPoints", wayPoints(1,:));
%         end


%% KW Result 변환
% clc;
% clear;
% load('kwResults.mat')
%         for i=1:size(coordix,2)
%             for j = 1:size(coordix{i},2)
%                  % 새로운 x좌표 = 0.05*(coordiy-1328)
%                  posx(j) = 0.05*(1328 - coordiy{i}(1,j))
%                  % 새로운 y좌표 = 0.05*(1655.5-coordix)
%                  posy(j)  = 0.05*(1655.5 - coordix{i}(1,j))
%                  if(j ==size(coordix{i},2) )
%                      posTheta(j) = atan2(posy(1)-posy(j),posx(1)-posx(j))
%                  else
%                      posTheta(j) = atan2(0.05*(1655.5 - coordix{i}(1,j+1))-posy(j),0.05*(1328 - coordiy{i}(1,j+1))-posx(j))
%                  end
%                  wayPoints(j) = struct('posx', posx(j),'posy', posy(j),'posTheta', posTheta(j));
%             end
%             SubStruct(i) = struct("robotId", i,"wayPoints", wayPoints(1,:));
%         end
%         
%          MsgStruct = struct("robots",SubStruct(1,:));
%         transParam.sendMsg= jsonencode(MsgStruct);
%         
%         fileID = fopen('RRINIT.json','w');
%         fprintf(fileID,transParam.sendMsg);
%         fclose('all'); 