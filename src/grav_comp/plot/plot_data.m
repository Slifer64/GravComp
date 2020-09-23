clc;
close all;
clear;

addpath('utils/io_lib/');
addpath('utils/math_lib/');


data_filename = '../data/data.bin';

%% ========  Read exec data  ============
fid = fopen(data_filename,'r');
if (fid < 0), error(['Failed to open "' data_filename '"']); end

wrench_data = read_mat(fid, true);
quat_data = read_mat(fid, true);

fclose(fid);


%% ========  process data  ============
g = [0; 0; -9.81];

force_data = wrench_data(1:3,:);
torque_data = wrench_data(4:6,:);

mass = mean(force_data(3,:));
% CoM = 


%% ========  plot data  ============

% figure;
% for i=1:6
%     subplot(6,1,i);
%     stem(wrench_data(i,:));
% end

