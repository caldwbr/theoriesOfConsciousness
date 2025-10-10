%% 3D Ring Theory Animation with Video Recording
% Clear workspace
clear; clc; close all;

%% Video setup
video_filename = 'ring_theory_3d_animation.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 20;  % 20 fps for smooth animation
v.Quality = 95;    % High quality
open(v);

fprintf('Recording video: %s\n', video_filename);

%% Animation parameters
total_time = 15;  % Shorter for demo
dt = 0.05;
t = 0:dt:total_time;

%% 3D parameters
radius = 1;
time_scale = 0.5;  % Scaling for time dimension (z-axis)

%% Mode parameters
continuous_speed = 2*pi/4;  % rad/sec

% Discontinuous: 80ms active, 20ms gap (scaled for visualization)
disc_active_time = 3.2;  % 80% of 4sec cycle  
disc_gap_time = 0.8;     % 20% gap

% Compressed: faster active phase
comp_active_time = 2.0;
comp_cycle_time = 4.0;

%% Create figure
figure('Position', [50, 50, 1600, 600], 'Color', 'white');

for i = 1:length(t)
    clf;
    
    % Calculate helix points for history
    history_points = max(1, i-50):i;  % Show last 50 points
    
    %% Continuous mode (left) - Perfect helix
    subplot(1,3,1);
    hold on;
    
    % Draw current ring
    theta_ring = linspace(0, 2*pi, 50);
    x_ring = radius * cos(theta_ring);
    y_ring = radius * sin(theta_ring);
    z_ring = t(i) * time_scale * ones(size(x_ring));
    plot3(x_ring, y_ring, z_ring, 'b-', 'LineWidth', 2);
    
    % Draw helix history
    if length(history_points) > 1
        helix_theta = mod(t(history_points) * continuous_speed, 2*pi);
        x_helix = radius * cos(helix_theta);
        y_helix = radius * sin(helix_theta);
        z_helix = t(history_points) * time_scale;
        plot3(x_helix, y_helix, z_helix, 'b-', 'LineWidth', 1, 'Color', [0.7 0.7 1]);
    end
    
    % Draw tracer and arm
    angle_cont = mod(t(i) * continuous_speed, 2*pi);
    x_tracer = radius * cos(angle_cont);
    y_tracer = radius * sin(angle_cont);
    z_tracer = t(i) * time_scale;
    
    plot3(x_tracer, y_tracer, z_tracer, 'o', 'MarkerSize', 10, ...
          'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
    plot3([0, x_tracer], [0, y_tracer], [z_tracer, z_tracer], 'y-', 'LineWidth', 3);
    
    title('Continuous Mode', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X'); ylabel('Y'); zlabel('Time →');
    axis equal; grid on;
    view(45, 30);
    xlim([-1.5,1.5]); ylim([-1.5,1.5]); zlim([0, max(t)*time_scale]);
    hold off;
    
    %% Discontinuous mode (middle) - Interrupted helix
    subplot(1,3,2);
    hold on;
    
    % Calculate discontinuous phase
    cycle_time_disc = mod(t(i), disc_active_time + disc_gap_time);
    current_cycle = floor(t(i) / (disc_active_time + disc_gap_time));
    cycle_start_time = current_cycle * (disc_active_time + disc_gap_time);
    
    if cycle_time_disc < disc_active_time
        % Active phase
        angle_disc = 2*pi * (cycle_time_disc / disc_active_time);
        tracer_color = 'y';
        arm_color = 'y';
        ring_color = 'r-';
        phase_text = 'ACTIVE (printing reality)';
        bg_color = 'white';
        
        % Draw current ring
        x_ring = radius * cos(theta_ring);
        y_ring = radius * sin(theta_ring);
        z_ring = t(i) * time_scale * ones(size(x_ring));
        plot3(x_ring, y_ring, z_ring, ring_color, 'LineWidth', 2);
        
    else
        % Gap phase - NO RING, just void
        angle_disc = 2*pi;
        tracer_color = [0.3 0.3 0.3];
        arm_color = [0.3 0.3 0.3];
        phase_text = 'GAP (void/blackness)';
        bg_color = [0.95 0.95 0.95];
        
        % Show ghost ring at last position
        ghost_time = cycle_start_time + disc_active_time;
        x_ring = radius * cos(2*pi);
        y_ring = radius * sin(2*pi);
        z_ring = ghost_time * time_scale;
        plot3(x_ring, y_ring, z_ring, 'o', 'MarkerSize', 8, ...
              'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerEdgeColor', [0.5 0.5 0.5]);
    end
    
    % Draw helix history with gaps
    for hist_idx = history_points
        hist_cycle_time = mod(t(hist_idx), disc_active_time + disc_gap_time);
        if hist_cycle_time < disc_active_time
            % Was in active phase - draw point
            hist_angle = 2*pi * (hist_cycle_time / disc_active_time);
            hist_cycle = floor(t(hist_idx) / (disc_active_time + disc_gap_time));
            hist_cycle_start = hist_cycle * (disc_active_time + disc_gap_time);
            hist_active_time = hist_cycle_time;
            
            x_hist = radius * cos(hist_angle);
            y_hist = radius * sin(hist_angle);
            z_hist = (hist_cycle_start + hist_active_time) * time_scale;
            
            plot3(x_hist, y_hist, z_hist, 'ro', 'MarkerSize', 4, ...
                  'MarkerFaceColor', [1 0.5 0.5], 'MarkerEdgeColor', 'r');
        end
    end
    
    % Draw tracer and arm (even in gap phase)
    x_tracer = radius * cos(angle_disc);
    y_tracer = radius * sin(angle_disc);
    z_tracer = t(i) * time_scale;
    
    plot3(x_tracer, y_tracer, z_tracer, 'o', 'MarkerSize', 10, ...
          'MarkerFaceColor', tracer_color, 'MarkerEdgeColor', 'k');
    plot3([0, x_tracer], [0, y_tracer], [z_tracer, z_tracer], '-', ...
          'Color', arm_color, 'LineWidth', 3);
    
    title('Discontinuous Mode', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X'); ylabel('Y'); zlabel('Time →');
    axis equal; grid on;
    view(45, 30);
    xlim([-1.5,1.5]); ylim([-1.5,1.5]); zlim([0, max(t)*time_scale]);
    set(gca, 'Color', bg_color);
    hold off;
    
    %% Compressed mode (right) - Compressed active phases
    subplot(1,3,3);
    hold on;
    
    % Calculate compressed phase
    cycle_time_comp = mod(t(i), comp_cycle_time);
    current_cycle_comp = floor(t(i) / comp_cycle_time);
    cycle_start_comp = current_cycle_comp * comp_cycle_time;
    
    if cycle_time_comp < comp_active_time
        % Active phase (compressed)
        angle_comp = 2*pi * (cycle_time_comp / comp_active_time);
        tracer_color = 'y';
        arm_color = 'y';
        ring_color = 'g-';
        phase_text = 'ACTIVE (compressed)';
        bg_color = 'white';
        
        % Draw current ring
        x_ring = radius * cos(theta_ring);
        y_ring = radius * sin(theta_ring);
        z_ring = t(i) * time_scale * ones(size(x_ring));
        plot3(x_ring, y_ring, z_ring, ring_color, 'LineWidth', 2);
        
    else
        % Extended gap phase
        angle_comp = 2*pi;
        tracer_color = [0.3 0.3 0.3];
        arm_color = [0.3 0.3 0.3];
        phase_text = 'EXTENDED GAP';
        bg_color = [0.95 0.95 0.95];
        
        % Show ghost ring at last position
        ghost_time = cycle_start_comp + comp_active_time;
        x_ring = radius * cos(2*pi);
        y_ring = radius * sin(2*pi);
        z_ring = ghost_time * time_scale;
        plot3(x_ring, y_ring, z_ring, 'o', 'MarkerSize', 8, ...
              'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerEdgeColor', [0.5 0.5 0.5]);
    end
    
    % Draw helix history for compressed mode
    for hist_idx = history_points
        hist_cycle_time = mod(t(hist_idx), comp_cycle_time);
        if hist_cycle_time < comp_active_time
            % Was in active phase - draw point
            hist_angle = 2*pi * (hist_cycle_time / comp_active_time);
            hist_cycle = floor(t(hist_idx) / comp_cycle_time);
            hist_cycle_start = hist_cycle * comp_cycle_time;
            hist_active_time = hist_cycle_time;
            
            x_hist = radius * cos(hist_angle);
            y_hist = radius * sin(hist_angle);
            z_hist = (hist_cycle_start + hist_active_time) * time_scale;
            
            plot3(x_hist, y_hist, z_hist, 'go', 'MarkerSize', 4, ...
                  'MarkerFaceColor', [0.5 1 0.5], 'MarkerEdgeColor', 'g');
        end
    end
    
    % Draw tracer and arm
    x_tracer = radius * cos(angle_comp);
    y_tracer = radius * sin(angle_comp);
    z_tracer = t(i) * time_scale;
    
    plot3(x_tracer, y_tracer, z_tracer, 'o', 'MarkerSize', 10, ...
          'MarkerFaceColor', tracer_color, 'MarkerEdgeColor', 'k');
    plot3([0, x_tracer], [0, y_tracer], [z_tracer, z_tracer], '-', ...
          'Color', arm_color, 'LineWidth', 3);
    
    title('Compressed Mode', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X'); ylabel('Y'); zlabel('Time →');
    axis equal; grid on;
    view(45, 30);
    xlim([-1.5,1.5]); ylim([-1.5,1.5]); zlim([0, max(t)*time_scale]);
    set(gca, 'Color', bg_color);
    hold off;
    
    % Add overall title
    sgtitle({'Ring Theory: 3D Temporal Printing', ...
             'Z-axis represents time flow of consciousness'}, ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    % Capture frame and add to video
    frame = getframe(gcf);
    writeVideo(v, frame);
    
    % Display progress
    if mod(i, 10) == 0
        fprintf('Progress: %.1f%%\n', 100*i/length(t));
    end
    
    drawnow;
end

% Close video file
close(v);
fprintf('Video saved as: %s\n', video_filename);
fprintf('Animation complete!\n');