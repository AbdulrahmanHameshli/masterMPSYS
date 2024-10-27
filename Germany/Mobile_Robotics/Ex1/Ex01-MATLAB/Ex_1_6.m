% 1.6 A-Ec
clc 
clear all 
scan = load("scan.txt");
phi = scan(:,1);
rho = scan(:,2);
[x y]=pol2cart(phi ,rho);
% figure
% plot(x, y,".");  


valid = rho <= 7.5;
rho = rho(valid);
phi = phi(valid);
[x y]=pol2cart(phi ,rho);
% figure
% plot(x, y,".");
index = find(abs(diff(rho)) > 0.3);
break_point_indices = [0, index', length(rho)];
segments = struct("id", [] ,"begin",[],"end",[],"point",[],"CenterOfGravity",[]);

figure
for i=1:length(index)-1
%     indexes
    begin_inx = break_point_indices(i) + 1;
    end_inx = break_point_indices(i + 1);
%     positions polar
    x_cart = x(begin_inx:end_inx);
    y_cart = y(begin_inx:end_inx);
% Center of segment
    center_of_gravity_x = mean(x_cart);
    center_of_gravity_y = mean(y_cart);
%    strucT
    segments(i).id = i;
    segments(i).begin = begin_inx;
    segments(i).end = end_inx;
    segments(i).point = [x_cart ,y_cart];
    segments(i).CenterOfGravity = [center_of_gravity_x,center_of_gravity_y];
    hold on
    plot(x_cart,y_cart,".")
    text(center_of_gravity_x, center_of_gravity_y, num2str(i))
    
end
% 
%
%% F
figure 
for i=1:length(segments)
    diff = segments(i).end - segments(i).begin;

    if diff >= 3 
        hold on 
        plot(segments(i).point(:,1),segments(i).point(:,2),'.')
        text(segments(i).CenterOfGravity(1), segments(i).CenterOfGravity(2), num2str(i))
    end

end

