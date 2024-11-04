%%  EX2.4 A) and B)

clc
clear all

Data = load("distributions.mat");

% COL 1
col1 = Data.data(:,1);
mean_col1 = sum(col1) / length(col1);
var_col1 = sum((col1 - mean_col1).^2) / (length(col1) - 1);
Std_col1 = sqrt(var_col1);
moment3_col1 = sum((col1 - mean_col1).^3) / (length(col1));
moment4_col1 = sum((col1 - mean_col1).^4) / (length(col1));

% Verify COL 1
mean_builtin1 = mean(col1);
var_builtin1 = var(col1);
std_builtin1 = std(col1);
moment3_builtin1 = moment(col1, 3);
moment4_builtin1 = moment(col1, 4);

fprintf('Column 1 Verification:\n');
fprintf('Mean - Calculated: %.4f, Built-in: %.4f\n', mean_col1, mean_builtin1);
fprintf('Variance - Calculated: %.4f, Built-in: %.4f\n', var_col1, var_builtin1);
fprintf('Std Dev - Calculated: %.4f, Built-in: %.4f\n', Std_col1, std_builtin1);
fprintf('3rd Moment - Calculated: %.4f, Built-in: %.4f\n', moment3_col1, moment3_builtin1);
fprintf('4th Moment - Calculated: %.4f, Built-in: %.4f\n\n', moment4_col1, moment4_builtin1);

% Repeat for COL 2
col2 = Data.data(:,2);
mean_col2 = sum(col2) / length(col2);
var_col2 = sum((col2 - mean_col2).^2) / (length(col2) - 1);
Std_col2 = sqrt(var_col2);
moment3_col2 = sum((col2 - mean_col2).^3) / (length(col2) );
moment4_col2 = sum((col2 - mean_col2).^4) / (length(col2) );

% Verify COL 2
mean_builtin2 = mean(col2);
var_builtin2 = var(col2);
std_builtin2 = std(col2);
moment3_builtin2 = moment(col2, 3);
moment4_builtin2 = moment(col2, 4);

fprintf('Column 2 Verification:\n');
fprintf('Mean - Calculated: %.4f, Built-in: %.4f\n', mean_col2, mean_builtin2);
fprintf('Variance - Calculated: %.4f, Built-in: %.4f\n', var_col2, var_builtin2);
fprintf('Std Dev - Calculated: %.4f, Built-in: %.4f\n', Std_col2, std_builtin2);
fprintf('3rd Moment - Calculated: %.4f, Built-in: %.4f\n', moment3_col2, moment3_builtin2);
fprintf('4th Moment - Calculated: %.4f, Built-in: %.4f\n\n', moment4_col2, moment4_builtin2);

% Repeat for COL 3
col3 = Data.data(:,3);
mean_col3 = sum(col3) / length(col3);
var_col3 = sum((col3 - mean_col3).^2) / (length(col3) - 1);
Std_col3 = sqrt(var_col3);
moment3_col3 = sum((col3 - mean_col3).^3) / (length(col3));
moment4_col3 = sum((col3 - mean_col3).^4) / (length(col3));

% Verify COL 3
mean_builtin3 = mean(col3);
var_builtin3 = var(col3);
std_builtin3 = std(col3);
moment3_builtin3 = moment(col3, 3);
moment4_builtin3 = moment(col3, 4);

fprintf('Column 3 Verification:\n');
fprintf('Mean - Calculated: %.4f, Built-in: %.4f\n', mean_col3, mean_builtin3);
fprintf('Variance - Calculated: %.4f, Built-in: %.4f\n', var_col3, var_builtin3);
fprintf('Std Dev - Calculated: %.4f, Built-in: %.4f\n', Std_col3, std_builtin3);
fprintf('3rd Moment - Calculated: %.4f, Built-in: %.4f\n', moment3_col3, moment3_builtin3);
fprintf('4th Moment - Calculated: %.4f, Built-in: %.4f\n\n', moment4_col3, moment4_builtin3);

% Repeat for COL 4
col4 = Data.data(:,4);
mean_col4 = sum(col4) / length(col4);
var_col4 = sum((col4 - mean_col4).^2) / (length(col4) - 1);
Std_col4 = sqrt(var_col4);
moment3_col4 = sum((col4 - mean_col4).^3) / (length(col4));
moment4_col4 = sum((col4 - mean_col4).^4) / (length(col4));

% Verify COL 4
mean_builtin4 = mean(col4);
var_builtin4 = var(col4);
std_builtin4 = std(col4);
moment3_builtin4 = moment(col4, 3);
moment4_builtin4 = moment(col4, 4);

fprintf('Column 4 Verification:\n');
fprintf('Mean - Calculated: %.4f, Built-in: %.4f\n', mean_col4, mean_builtin4);
fprintf('Variance - Calculated: %.4f, Built-in: %.4f\n', var_col4, var_builtin4);
fprintf('Std Dev - Calculated: %.4f, Built-in: %.4f\n', Std_col4, std_builtin4);
fprintf('3rd Moment - Calculated: %.4f, Built-in: %.4f\n', moment3_col4, moment3_builtin4);
fprintf('4th Moment - Calculated: %.4f, Built-in: %.4f\n\n', moment4_col4, moment4_builtin4);

% Repeat for COL 5
col5 = Data.data(:,5);
mean_col5 = sum(col5) / length(col5);
var_col5 = sum((col5 - mean_col5).^2) / (length(col5) - 1);
Std_col5 = sqrt(var_col5);
moment3_col5 = sum((col5 - mean_col5).^3) / (length(col5) );
moment4_col5 = sum((col5 - mean_col5).^4) / (length(col5) );

% Verify COL 5
mean_builtin5 = mean(col5);
var_builtin5 = var(col5);
std_builtin5 = std(col5);
moment3_builtin5 = moment(col5, 3);
moment4_builtin5 = moment(col5, 4);

fprintf('Column 5 Verification:\n');
fprintf('Mean - Calculated: %.4f, Built-in: %.4f\n', mean_col5, mean_builtin5);
fprintf('Variance - Calculated: %.4f, Built-in: %.4f\n', var_col5, var_builtin5);
fprintf('Std Dev - Calculated: %.4f, Built-in: %.4f\n', Std_col5, std_builtin5);
fprintf('3rd Moment - Calculated: %.4f, Built-in: %.4f\n', moment3_col5, moment3_builtin5);
fprintf('4th Moment - Calculated: %.4f, Built-in: %.4f\n\n', moment4_col5, moment4_builtin5);

%% C)


x_limits = [-5, 20];
bins = linspace(x_limits(1), x_limits(2), 100); % 100 bins from -5 to 20

figure;
hold on;

for i = 1:5
    
    col = Data.data(:, i);
    
    
    counts = histc(col, bins);
    relative_freq = counts / sum(counts);
    

    % Plot relative frequency distribution with specified line style
    plot(bins, relative_freq, '.-', 'DisplayName', ['Column ' num2str(i)]);
end

xlim(x_limits);
legend show;
xlabel('Value');
ylabel('Relative Frequency');
title('Normalized Histograms of Distributions from Data Columns');
hold off;

%% D) 
% Answer :col1 and col2- Gaussian,col3 - Poisson, col4 -
% Binomial and col5 - Chi-Squared.

%% E) 
% col1 and 2 are Gaussian and params for col1 are:'
% Mean - Calculated: 5.9857 and Variance - Calculated: 3.9768, Std Dev - Calculated: 1.9942
% col2 - Mean - Calculated: -3.0028, Variance - Calculated: 0.2553,Std_Dev - Calculated: 0.5052



