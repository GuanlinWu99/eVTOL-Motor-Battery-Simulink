% ========================================================================
% Plot trajectory of eVTOL
% ------------------------------------------------------------------------
% Plot the trajectory of eVTOL using 3d animation
% ========================================================================

function animate_3d_traj_eVTOL(simOut, mission)

    %.. get hex color code
    RGB =   orderedcolors("gem");
    H   =   rgb2hex(RGB);
    
    %.. load stl and pre-process for triangulation object
    fv  =   stlread('SUAVE_full.stl');
    if isa(fv, 'triangulation')
        faces       =   fv.ConnectivityList;
        vertices    =   fv.Points-ones(size(fv.Points,1),1)*[3.5,0.0,0.0];
    else
        faces       =   fv.faces;
        vertices    =   fv.vertices-ones(size(fv.Points,1),1)*[3.5,0.0,0.0];
    end
    [faces_reduce,vertices_reduce]  =   reducepatch(faces,vertices,0.05);
    vertices_reduce =   ([cos(pi) sin(pi) 0; -sin(pi) cos(pi) 0; 0 0 1]*vertices_reduce')';

    %.. extract trajectory data
    if isa(simOut.UAV_State.Xe, 'timeseries')
        X_traj_plot     =   reshape(simOut.UAV_State.Xe.Data,size(simOut.UAV_State.Xe.Data,1),size(simOut.UAV_State.Xe.Data,3))';
        X_euler_plot    =   reshape(simOut.UAV_State.Euler.Data,size(simOut.UAV_State.Euler.Data,1),size(simOut.UAV_State.Euler.Data,3))';
        t_traj_plot     =   simOut.UAV_State.Xe.Time(:);

    elseif isstruct(simOut.UAV_State.Xe) && isfield(simOut.UAV_State.Xe,'signals') && isfield(simOut.UAV_State.Xe,'time')
        X_traj_plot     =   reshape(simOut.UAV_State.Xe.signals.values,size(simOut.UAV_State.Xe.Data,1),size(simOut.UAV_State.Xe.Data,3))';
        X_euler_plot    =   reshape(simOut.UAV_State.Euler.signals.values,size(simOut.UAV_State.Euler.Data,1),size(simOut.UAV_State.Euler.Data,3))';
        t_traj_plot     =   simOut.UAV_State.Xe.time;

    else
        X_traj_plot     =   reshape(simOut.UAV_State.Xe.Data,size(simOut.UAV_State.Xe.Data,1),size(simOut.UAV_State.Xe.Data,3))';
        X_euler_plot    =   reshape(simOut.UAV_State.Euler.Data,size(simOut.UAV_State.Euler.Data,1),size(simOut.UAV_State.Euler.Data,3))';
        t_traj_plot     =   (0:size(X_traj_plot,2)-1)'/1000;

    end

    if size(X_traj_plot,2) ~= 3
        error('Trajectory must be Nx3 (North, East, Down).');
    end

    %.. remove any NaNs
    nanMask     =   any(isnan(X_traj_plot),2);
    
    if any(nanMask)
        X_traj_plot(nanMask,:)  =   [];
        t_traj_plot(nanMask,:)  =   [];
    end

    %.. obtain reference profile
    X_traj_ref  =   zeros(size(mission,2)+1,3);

    for idx = 2:size(X_traj_ref,1)
        X_traj_ref(idx,:)   =   mission(idx-1).position';
    end

    %.. apply plot convention (NED -> NWU)
    X_traj_plot     =   [X_traj_plot(:,1), -X_traj_plot(:,2), -X_traj_plot(:,3)];
    X_traj_ref      =   [X_traj_ref(:,1), -X_traj_ref(:,2), -X_traj_ref(:,3)];

    %.. plot the animation of trajectory data using stl file
    figure('Color','w'); 
    clf;
    ax = gca; 
    hold(ax,'on'); 
    grid(ax,'on');
    plot3(ax, X_traj_plot(:,1), X_traj_plot(:,2), X_traj_plot(:,3), 'LineWidth', 2, 'Color', [0 0.45 0.74]);
    plot3(ax, X_traj_ref(:,1), X_traj_ref(:,2), X_traj_ref(:,3), '--', 'LineWidth', 1.5, 'Color', [0.3 0.3 0.3]);
    xlabel(ax, 'North [m]'); 
    ylabel(ax, 'West [m]'); 
    zlabel(ax, 'Up [m]');
    title(ax, 'eVTOL trajectory (P4 profile)'); 
    ylim(ax, [-250 250]);
    axis(ax, 'equal');
    legend(ax, 'UAV-3D Trajectory', 'P4-profile (Modified)', 'Location','best');
    view(35, 20);
    1

end






% 
% 
% time = linspace(0, 10, 100); % 10 seconds, 100 points
% trajectory = [sin(time); cos(time); time]; % Example 3D trajectory
% plot3(trajectory(1, :), trajectory(2, :), trajectory(3, :), 'LineWidth', 2);
% grid on;
% xlabel('X Position');
% ylabel('Y Position');
% zlabel('Z Position');
% title('3D Trajectory of eVTOL');
% view(3);