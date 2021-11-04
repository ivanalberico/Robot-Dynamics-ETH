% FW SIMULATION PLOTS

% Thomas Stastny
% 2018.12.12

%% control signals

figure('color','w');

subplot(4,1,1); hold on; grid on; box on;
plot(tout,uout.signals.values(:,1));
ylabel('u_T [~]')
ylim([0 1])

subplot(4,1,2); hold on; grid on; box on;
plot(tout,uout.signals.values(:,2));
ylabel('\delta_E [~]')
ylim([-1,1])

subplot(4,1,3); hold on; grid on; box on;
plot(tout,uout.signals.values(:,3));
ylabel('\delta_A [~]')
ylim([-1,1])

subplot(4,1,4); hold on; grid on; box on;
plot(tout,uout.signals.values(:,4));
ylabel('\delta_R [~]')
ylim([-1,1])

xlabel('t [s]')

%% delta

figure('color','w');

subplot(4,1,1); hold on; grid on; box on;
plot(tout,delta_out.signals.values(:,1));
ylabel('u_T [~]')
ylim([0 1])

subplot(4,1,2); hold on; grid on; box on;
plot(tout,rad2deg(delta_out.signals.values(:,2)));
ylabel('\delta_E [deg]')
ylim([-20,20])

subplot(4,1,3); hold on; grid on; box on;
plot(tout,rad2deg(delta_out.signals.values(:,3)));
plot(tout,rad2deg(delta_out.signals.values(:,4)));
ylabel('\delta_A [deg]')
legend('Right','Left')
ylim([-20,20])

subplot(4,1,4); hold on; grid on; box on;
plot(tout,rad2deg(delta_out.signals.values(:,5)));
ylabel('\delta_R [deg]')
ylim([-20,20])

xlabel('t [s]')


%% u,v,w

figure('color','w');

subplot(3,1,1); hold on; grid on; box on;
plot(tout,xout.signals.values(:,1));
ylabel('u [m/s]')

subplot(3,1,2); hold on; grid on; box on;
plot(tout,xout.signals.values(:,2));
ylabel('v [m/s]')

subplot(3,1,3); hold on; grid on; box on;
plot(tout,xout.signals.values(:,3));
ylabel('w [m/s]')

xlabel('t [s]')

%% p,q,r

figure('color','w');

subplot(3,1,1); hold on; grid on; box on;
plot(tout,rad2deg(control_ref_out.signals.values(:,1)),'--');
plot(tout,rad2deg(xout.signals.values(:,4)));
ylabel('p [deg/s]')

subplot(3,1,2); hold on; grid on; box on;
plot(tout,rad2deg(control_ref_out.signals.values(:,2)),'--');
plot(tout,rad2deg(xout.signals.values(:,5)));
ylabel('q [deg/s]')

subplot(3,1,3); hold on; grid on; box on;
plot(tout,rad2deg(control_ref_out.signals.values(:,3)),'--');
plot(tout,rad2deg(xout.signals.values(:,6)));
ylabel('r [deg/s]')

xlabel('t [s]')

%% phi,theta,psi

figure('color','w');

subplot(3,1,1); hold on; grid on; box on;
plot(tout,rad2deg(control_ref_out.signals.values(:,4)),'--');
plot(tout,rad2deg(xout.signals.values(:,7)));
ylabel('\phi [deg]')

subplot(3,1,2); hold on; grid on; box on;
plot(tout,rad2deg(control_ref_out.signals.values(:,5)),'--');
plot(tout,rad2deg(xout.signals.values(:,8)));
ylabel('\theta [deg]')

subplot(3,1,3); hold on; grid on; box on;
plot(tout,rad2deg(xout.signals.values(:,9)));
ylabel('\psi [deg]')

xlabel('t [s]')

%% position

plot_path = 1;

figure('color','w');

subplot(5,1,1:4); hold on; grid on; box on;

if plot_path
    tt = linspace(0,2*pi,501);
    plot3(path_out.signals.values(1,4)+path_out.signals.values(1,1)*sin(tt), ...
        path_out.signals.values(1,3)+path_out.signals.values(1,1)*cos(tt), ...
        -path_out.signals.values(1,5)*ones(1,length(tt)), ...
        '--');
end
plot3(xout.signals.values(:,11),xout.signals.values(:,10),-xout.signals.values(:,12));
xlabel('Easting [m]')
ylabel('Northing [m]')
zlabel('Height [m]')

subplot(5,1,5); hold on; grid on; box on;
if plot_path
    plot(tout([1 end]),-path_out.signals.values(1,5)*ones(1,2),'--');
end
plot(tout,-xout.signals.values(:,12));
ylabel('Height [m]')
xlabel('t [s]')

