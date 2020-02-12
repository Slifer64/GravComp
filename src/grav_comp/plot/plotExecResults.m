function plotExecResults(data_filename)

clc;
close all;
clear;

addpath('utils/io_lib/');
addpath('utils/math_lib/');

if (nargin < 1), data_filename = 'windshield_wrench_orient_data.bin'; end

data_filename = ['../data/' data_filename];

%% ========  Read exec data  ============
fid = fopen(data_filename,'r');
if (fid < 0), error(['Failed to open "' data_filename '"']); end
wrench_data = read_mat(fid, true);
quat_data = read_mat(fid, true);
fclose(fid);

figure;
plot(abs(wrench_data(3,:)), 'LineWidth',2);

mean(abs(wrench_data(3,:)) / 9.81)


end