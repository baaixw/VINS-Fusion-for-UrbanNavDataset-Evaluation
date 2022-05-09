clear;
close all;

%%%%%%%%%%trajectory
traj_data1 = csvread('groundTruth.csv');
traj_epoch = traj_data1(:,1);
east_ref1=traj_data1(:,2);
north_ref1 = traj_data1(:,3);


traj_data = csvread('vio.csv');
traj_epoch = traj_data(:,1);
east_vio=traj_data(:,2);
north_vio = traj_data(:,3);

figure(1)
axis equal
plot(east_ref1,north_ref1,'k-','LineWidth',1.9 )
hold on;
plot(east_vio,north_vio,'b-o','LineWidth',1.2 )
hold on;
grid on;
hold on;
axis equal
ax = gca;
ax.FontSize = 16; 
xlabel('east (meters)');
ylabel('north (meters)');
legend('\fontsize{16} GroundTruth','\fontsize{16} VIO');


error_data = csvread('error.csv');

figure(2)
axis equal
%error_epoch = error_data(:,1);
error_vio=error_data(:,1);


%plot(error_epoch,error_gnss1,'m-','LineWidth',1.9 )
%hold on;
plot(error_vio,'b-','LineWidth',1.9 )
hold on;
grid on;
hold on;
ax = gca;
ax.FontSize = 16; 
xlabel('time (seconds)');
ylabel('error (meters)');
legend('\fontsize{16} VIO');

mean(error_vio)
