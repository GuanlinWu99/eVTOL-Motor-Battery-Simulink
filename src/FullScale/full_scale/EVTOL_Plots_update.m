function EVTOL_Plots(simOut)

    fontsize              =  12;

    Time                  =  simOut.UAV_State.RotorParameters.w1.Time;
    simTime               =  15;

    %% Flying performance

    airspeed              =  simOut.UAV_State.airspeed.Data;

    %% Position data

    % positionFeedbackData  =  squeeze(simOut.PositionCmdFdbk.signals.values);

    %% Propulsion data

    %W1                    =  simOut.UAV_State.RotorParameters.w1.Data;
    %W2                    =  simOut.UAV_State.RotorParameters.w2.Data;
    %W3                    =  simOut.UAV_State.RotorParameters.w3.Data;
    %W4                    =  simOut.UAV_State.RotorParameters.w4.Data;

    % M1                    =  simOut.Moment1.Data(:,3);
    % M2                    =  simOut.Moment2.Data(:,3);
    % M3                    =  simOut.Moment3.Data(:,3);
    % M4                    =  simOut.Moment4.Data(:,3);

    Omega1                =  simOut.UAV_State.RotorParameters.w1.Data;
    Omega2                =  simOut.UAV_State.RotorParameters.w2.Data;
    Omega3                =  simOut.UAV_State.RotorParameters.w3.Data;
    Omega4                =  simOut.UAV_State.RotorParameters.w4.Data;

    % Propulsion_Fx         =  simOut.Propulsion_Force.Data(:,1);
    % Propulsion_Fy         =  simOut.Propulsion_Force.Data(:,2);
    % Propulsion_Fz         =  simOut.Propulsion_Force.Data(:,3);
    
    % Required_Power        =  M1 .* Omega1 + M2 .* Omega2 + M3 .* Omega3 + M4 .* Omega4;

    %Gravity               =  simOut.Gravity.Data(:,3);

    %Roll                  =  reshape(simOut.UAV_State.Euler.Data(1,:,:),1,length(Time));
    %Pitch                 =  reshape(simOut.UAV_State.Euler.Data(2,:,:),1,length(Time));
    %Yaw                   =  reshape(simOut.UAV_State.Euler.Data(3,:,:),1,length(Time));

    Vb_x                  =  reshape(simOut.UAV_State.Vb.Data(:,1,:),1,length(Time));
    Vb_y                  =  reshape(simOut.UAV_State.Vb.Data(:,2,:),1,length(Time));
    Vb_z                  =  reshape(simOut.UAV_State.Vb.Data(:,3,:),1,length(Time));

    Xe                  =  reshape(simOut.UAV_State.Xe.Data(1,:,:),1,length(Time));
    Ye                  =  reshape(simOut.UAV_State.Xe.Data(2,:,:),1,length(Time));
    Ze                  =  reshape(simOut.UAV_State.Xe.Data(3,:,:),1,length(Time));

    Phi                 =   reshape(simOut.UAV_State.Euler.Data(1,:,:),1,length(Time))/pi*180;
    Theta               =   reshape(simOut.UAV_State.Euler.Data(2,:,:),1,length(Time))/pi*180;
    Psi                 =   reshape(simOut.UAV_State.Euler.Data(3,:,:),1,length(Time))/pi*180;


    % Ve_x                  =  reshape(simOut.UAV_State.Ve.Data(1,:,:),1,length(Time));
    % Ve_y                  =  reshape(simOut.UAV_State.Ve.Data(2,:,:),1,length(Time));
    % Ve_z                  =  reshape(simOut.UAV_State.Ve.Data(3,:,:),1,length(Time));

    %% Total Force x y z 
    % figure(1)
    % plot(Time,Total_Fx,'LineWidth',2);
    % hold on;
    % plot(Time,Total_Fy,'LineWidth',2);
    % hold on;
    % plot(Time,Total_Fz,'LineWidth',2);
    % hold on; grid on
    % xlabel('Time [s]')
    % legend('Fx','Fy','Fz')

    % %% Motor spec... what is this?
    % figure(2)
    % plot(Time,W1,'LineWidth',2);
    % hold on;
    % plot(Time,W2,'LineWidth',2);
    % hold on;
    % plot(Time,W3,'LineWidth',2);
    % hold on;    
    % plot(Time,W4,'LineWidth',2);
    % hold on; grid on

    %%
    % figure(3)
    % plot(time,Pitch_Error(:,2)*180/pi,'LineWidth',2);
    % xlabel('Time (sec)')
    % ylabel('')
    % grid on;

    %% Plot in X-Y-Z
    figure(4)
    % %plot3(positionFeedbackData(1,:),-positionFeedbackData(2,:),-positionFeedbackData(3,:),'LineWidth',2); hold on
    plot3(Xe,Ye,-Ze,'LineWidth',3); grid on;
    xlabel('North [m]')
    ylabel('West [m]')
    zlabel('Altitude [m]')
    zlim([-1 100]);
    legend('VTOL Trajectory')

    %% Position plot versus time
    figure(5)
    subplot(3,1,1)
    %plot(simOut.PositionCmdFdbk.time,positionFeedbackData(1,:),'LineWidth',2)
    %hold on;
    plot(Time,Xe,'LineWidth',2)
    hold on;
    grid on
    xlim([0 simTime]);
    ylabel('North (m)')
    legend('VTOL Trajectory')
    title('VTOL Position')

    subplot(3,1,2)
    %plot(simOut.PositionCmdFdbk.time,positionFeedbackData(2,:),'LineWidth',2)
    hold on;
    plot(Time,Ye,'LineWidth',2)
    xlim([0 simTime]);
    grid on
    ylabel('West (m)')

    subplot(3,1,3)
    %plot(simOut.PositionCmdFdbk.time,-positionFeedbackData(3,:),'LineWidth',2)
    %hold on;
    plot(Time,-Ze,'LineWidth',2)
    grid on
    xlim([0 simTime]);
    xlabel('Time [s]')
    ylabel('Altitude [m]')

    %% Propulsion Force x y z 
    % figure(6)
    % plot(Time,Propulsion_Fx,'LineWidth',3);
    % hold on;
    % plot(Time,Propulsion_Fy,'LineWidth',3);
    % hold on;
    % plot(Time,Propulsion_Fz,'LineWidth',3);
    % hold on; 
    % grid on;
    % xlabel('Time [s]')
    % ylabel('Thrust [N]')
    % legend('$T_x$','$T_y$','$T_z$','Interpreter','latex','Fontsize',fontsize)
    % title('Total Rotor Thrust [N]')
    % xlim([0 simTime]);

    figure(7)
    plot(Time,Omega1,'LineWidth',3);
    hold on;
    plot(Time,Omega2,'-.','LineWidth',3);
    hold on;
    plot(Time,Omega3,'--','LineWidth',3);
    hold on; 
    plot(Time,Omega4,':','LineWidth',3);
    hold on; grid on
    xlabel('Time [s]')
    ylabel('Angular Velocity [rad/s]')
    legend('$\Omega_1$: Front Left','$\Omega_2$: Front Right','$\Omega_3$: Rear Right','$\Omega_4$: Rear Left','Interpreter','latex','Fontsize',fontsize)
    title('Rotor Angular Velocity')
    xlim([0 simTime]);

    % figure(8)
    % plot(Time,Roll*180/pi,'LineWidth',2);
    % hold on;
    % plot(Time,Pitch*180/pi,'LineWidth',2);
    % hold on;
    % plot(Time,Yaw*180/pi,'LineWidth',2);
    % hold on; grid on
    % xlabel('Time [s]')
    % ylabel('Attitude Angle [deg]')
    % legend('Roll: $\phi$','Pitch: $\theta$','Yaw: $\psi$','Interpreter','latex','Fontsize',fontsize)
    % title('VTOL Attitude')

    %% Airspeed

    figure(9)
    plot(Time,airspeed,'LineWidth',2);
    grid on
    xlabel('Time [s]')
    ylabel('Airspeed')
    title('Airspeed')
    xlim([0 simTime]);

    %% Body speed

    figure(10)
    plot(Time,Vb_x,'LineWidth',2);
    hold on;
    plot(Time,Vb_y,'LineWidth',2);
    hold on;
    plot(Time,-Vb_z,'LineWidth',2);
    hold on; grid on
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    legend('$U$','$V$','$W$','Interpreter','latex','Fontsize',fontsize)
    title('VTOL Velocity')
    xlim([0 simTime]);

    figure(11)
    plot(Time,Phi,'LineWidth',2);
    hold on;
    plot(Time,Theta,'LineWidth',2);
    hold on;
    plot(Time,Psi,'LineWidth',2);
    hold on; grid on
    xlabel('Time [s]')
    ylabel('Attitude [deg]')
    legend('Roll','Pitch','Yaw','Fontsize',fontsize)
    title('VTOL Attitude')
    xlim([0 simTime]);


    % figure(11)
    % tLayout=tiledlayout(4,1);
    % exampleHelperPlotTransitionResults(tLayout,simOut);
    % xlim([0 simTime]);


    %% Required Power

    % figure(12)
    % plot(Time,-Required_Power/1000,'LineWidth',3);
    % grid on;
    % xlabel('Time [s]')
    % ylabel('Required Power [kW]')
    % %legend('$V_{x}$','$V_{y}$','$V_{z}$','Interpreter','latex','Fontsize',fontsize)
    % title('Mechanical Power')
    % xlim([0 simTime]);

end
