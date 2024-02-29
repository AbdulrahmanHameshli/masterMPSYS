%**********************************************************************
% For for reading and plotting current step of separately excited DC-machine
%**********************************************************************

clc;            % clear command window
clear;          % clear workspace memory
close all;      % closing all plot windows

set(0,'defaulttextfontsize',14);
set(0,'defaultaxesfontsize',14);
set(0,'defaultaxesfontweight','bold');
set(0,'defaultlinelinewidth',2);
%**********************************************************************


lokal = load('DCStep.txt');   

t = lokal (1,:);        % time [s]
i = lokal(2,:);         % armature current [A]
iref = lokal(3,:);      % armature current reference [A]
w = lokal(4,:);         % rotor speed [rad/s]
v = lokal(5,:);         % armature voltage [V]


figure('Name','Determination of J with a current step')
subplot(2,1,1)
plot(t,i,'b',t,iref,'r')
grid on
xlabel('Time [s]','FontWeight','Bold')
ylabel('Current [A]','FontWeight','Bold')
legend('i_a','i_a_,_r_e_f')

subplot(2,1,2)
% calculation of the line equation for the speed
j=5;
while(w(j) <10)
   j=j+1;
end
p=polyfit(t(j:end),w(j:end),1)    
t_new=[t(j-200:end),t(end)+0.02];
y=p(1).*t_new+p(2);

plot(t_new,y,'r','LineWidth',2)
hold on
plot(t,w,'b',t,v,'g')
hold off
legend('Line-fit','\omega_r','v_T')
grid on
inclination=['y = ',num2str(p(1),'%.2f'),' x -  ',num2str(abs(p(2)),'%.2f')];
text(0.4,30,inclination,'FontWeight','Bold','Color','r','FontSize',11,'BackgroundColor',[1 1 1]);

xlabel('Time [s]','FontWeight','Bold')
ylabel('Speed [rad/s]','FontWeight','Bold')
print current_step -dpdf





