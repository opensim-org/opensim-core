clear all; close all; clc; format long;

% Optimize to identify curve.
opts = optimset('LargeScale','off', 'Display','off');

states_initialGuess  = [0 0 0 0];

[states_new, J] = fminunc(@DesignMainStarter_Manager, states_initialGuess, opts);

