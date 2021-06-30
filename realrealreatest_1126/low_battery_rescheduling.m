% Rescheduling request (Low battery rescheduling)

clear; clc; close all;

reqType = "LOWBAT";
agentId = 2;

% 1->0 (원래 없음) 1대

% 2->1 (정상 동작) 2대
% robots = struct("robotId", {4}, "posx", {350}, "posy", ...
%     {350}, "curTargetx", {360}, "curTargety", ...
%     {360}, "curTargetTheta", {0}, "prevTargetx", ...
%     {340}, "prevTargety", {340}, "prevTargetTheta", ...
%     {0});

% 3->2 (정상 동작) 3대
robots = struct("robotId", {3, 4}, "posx", {-5.149230857735421, 18.249688773360806}, "posy", ...
    {4.23000325226286, 36.10615118585168}, "curTargetx", {-4.775, 19.3}, "curTargety", ...
    {6.8500000000000005, 41.2}, "curTargetTheta", {1.5707963267948966, 1.541660675882435}, "prevTargetx", ...
    {-4.9, 17.925}, "prevTargety", {-4.525, 29.625}, "prevTargetTheta", ...
    {1.559807758112163, 1.452559904470229});

% 4->3 (정상 동작) 4대
% robots = struct("robotId", {1, 4, 5}, "posx", {50, 150, 350}, "posy", ...
%     {50, 150, 350}, "curTargetx", {60, 160, 360}, "curTargety", ...
%     {60, 160, 360}, "curTargetTheta", {0, 0, 0}, "prevTargetx", ...
%     {40, 140, 340}, "prevTargety", {40, 140, 340}, "prevTargetTheta", ...
%     {0, 0, 0});

% 5->4 (정상 동작) 5대
% robots = struct("robotId", {1, 3, 4, 5}, "posx", {50, 150, 350, 450}, "posy", ...
%     {50, 150, 350, 450}, "curTargetx", {60, 160, 360, 460}, "curTargety", ...
%     {60, 160, 360, 460}, "curTargetTheta", {0, 0, 0, 0}, "prevTargetx", ...
%     {40, 140, 340, 440}, "prevTargety", {40, 140, 340, 440}, "prevTargetTheta", ...
%     {0, 0, 0, 0});

S = struct("reqType", reqType, "agentId", agentId, "posx", ...
    -14.948353781564926, "posy", 1.3835342817668723, "robots", robots);
s = jsonencode(S);

% 1대에서만 수행
if(size(robots, 2) == 1)
    idx = strfind(s, "{");
    s = insertBefore(s, idx(2), "[");
    idx = strfind(s, "}");
    s = insertAfter(s, idx(1), "]");
end

% Create mqtt connection.
myMQTT = mqtt('tcp://192.168.0.3', 'Port', 1883);
% myMQTT = mqtt('tcp://220.81.76.111', 'Port', 1883);

% Create topic and message.
topic = '/mams/ph/scheduler/req';
message = s;

% Publish a message to a topic.
publish(myMQTT, topic, message, 'Qos', 0);