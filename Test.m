function Test    
clear; clc; close all;
    global transParam;
%% Parameter initialization
    
    % ph MQTT broker server
    transParam.host = 'tcp://220.81.76.111';
    
    % cloud MQTT broker server
    % transParam.host = 'tcp://52.79.114.42';
    
    % test MQTT broker server
    % transParam.host = 'tcp://192.168.0.8';
    
    transParam.revtopic = '/mams/ph/scheduler/req';
    transParam.transtopic = '/mams/ph/scheduler/res';
    
%% Subscribe
    
	% Create mqtt connection.
    transParam.subMQTT = mqtt(transParam.host, 'Port', 1883);
end