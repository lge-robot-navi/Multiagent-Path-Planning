%% publish to client INIT

global transParam;
reqType = "INIT";    %ABNORMAL  LOWBAT INIT
agentId = 0;

robotId = [1;2;3;4];
posx = [2;2;2;7];
posy = [1;3;2;3];
curTargetx = [3;2;2;9];
curTargety = [3;6;2;5];
curTargetTheta = [3;2;2;4];
prevTargetx = [1;3;2;2];
prevTargety = [0;3;3;2];
prevTargetTheta = [3;0;2;4];

robots = table(robotId, posx, posy, curTargetx, curTargety, ...
    curTargetTheta, prevTargetx, prevTargety, prevTargetTheta);

S = struct("reqType", reqType, "agentId", agentId, "posx", ...
    0, "posy", 0, "robots", robots);
s = jsonencode(S);

% fileID = fopen('pubTest.json','w');
% fprintf(fileID,s);
% fclose('all');
        %%
% myMQTTFunc = mqtt(transParam.transhost);
% publish(myMQTTFunc, transParam.transtopic,s);
% disp('published to Server');

myMQTTFunc = mqtt(transParam.revhost);
publish(myMQTTFunc, transParam.revtopic,s,'Qos', 0, 'Retain', false);
disp('published to Client');
%% publish to client 



% {
%     "reqType":"ABNORMAL",
%     "agentId":1,
%     "posx":10,
%     "posy":10, 
%     "robots":[
%         {
%             "robotId":1,
%             "posx":10,
%             "posy":10,
%             "curTargetx": 10,
%             "curTargety": 10,
%             "curTargetTheta":10,
%             "prevTargetx":10,
%             "prevTargety":10,
%             "prevTargetTheta":10
%         },
%         {
%             "robotId":2,
%             "posx":10,
%             "posy":10,
%             "curTargetx": 10,
%             "curTargety": 10,
%             "curTargetTheta":10,
%             "prevTargetx":10,
%             "prevTargety":10,
%             "prevTargetTheta":10
%         },
%         {
%             "robotId":3,
%             "posx":10,
%             "posy":10,
%             "curTargetx": 10,
%             "curTargety": 10,
%             "curTargetTheta":10,
%             "prevTargetx":10,
%             "prevTargety":10,
%             "prevTargetTheta":10
%         }
%     ]
% }


% {
%   "reqType": "LOWBAT",
%   "agentId": 1,
%   "posx": 0,
%   "posy": 0,
%   "robots": [
%     {
%       "robotId": 1,
%       "posx": 10,
%       "posy": 10,
%       "curTargetx": 10,
%       "curTargety": 10,
%       "curTargetTheta": 10,
%       "prevTargetx": 10,
%       "prevTargety": 10,
%       "prevTargetTheta": 10
%     },
%     {
%       "robotId": 2,
%       "posx": 10,
%       "posy": 10,
%       "curTargetx": 10,
%       "curTargety": 10,
%       "curTargetTheta": 10,
%       "prevTargetx": 10,
%       "prevTargety": 10,
%       "prevTargetTheta": 10
%     },
%     {
%       "robotId": 3,
%       "posx": 10,
%       "posy": 10,
%       "curTargetx": 10,
%       "curTargety": 10,
%       "curTargetTheta": 10,
%       "prevTargetx": 10,
%       "prevTargety": 10,
%       "prevTargetTheta": 10
%     }
%   ]
% }
% 


% {
%     "reqType":"INIT",
%     "agentId":0,
%     "posx":0,
%     "posy":0, 
%     "robots":[
%         {
%             "robotId":1,
%             "posx":0,
%             "posy":0,
%             "curTargetx": 0,
%             "curTargety": 0,
%             "curTargetTheta":0,
%             "prevTargetx":0,
%             "prevTargety":0,
%             "prevTargetTheta":0
%         },
%         {
%             "robotId":2,
%             "posx":0,
%             "posy":0,
%             "curTargetx": 0,
%             "curTargety": 0,
%             "curTargetTheta":0,
%             "prevTargetx":0,
%             "prevTargety":0,
%             "prevTargetTheta":0
%         },
%         {
%             "robotId":3,
%             "posx":0,
%             "posy":0,
%             "curTargetx": 0,
%             "curTargety": 0,
%             "curTargetTheta":0,
%             "prevTargetx":0,
%             "prevTargety":0,
%             "prevTargetTheta":0
%         }
%     ]
% }