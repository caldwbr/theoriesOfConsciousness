%% 3D Ring Theory Animation with Video Recording - High Speed with Helices
% Clear workspace
clear; clc; close all;

%% Video setup - 50 FPS for better temporal resolution
video_filename = 'ring_theory_helix_medium.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 50;  % 50 fps for high temporal resolution
v.Quality = 95;
open(v);

fprintf('Recording video: %s\n', video_filename);

%% Animation parameters - MUCH FASTER CYCLES
total_time = 8;     % Shorter since it's much faster
dt = 0.02;         % 50 fps
t = 0:dt:total_time;

%% 3D parameters
radius = 1;
ring_height = 2.5;  % Fixed height for print head
flow_speed = 2.0;   % Keep current flow speed (good as is)
max_history_time = 1.5; % Keep 1.5 seconds of history

%% Mode parameters - MUCH FASTER CYCLES (0.5-1 second cycles)
continuous_speed = 2*pi/0.5;  % 0.5 sec per revolution!

% Discontinuous: 80ms active, 20ms gap (scaled for visualization)
disc_active_time = 0.4;   % 80% of 0.5sec cycle (400ms active)
disc_gap_time = 0.1;      % 20% gap (100ms gap)

% Compressed: faster active phase
comp_active_time = 0.25;  % 250ms active
comp_cycle_time = 0.5;    % 500ms total cycle

% Interrupt: ONE ring per second exactly
interrupt_cycle_time = 1.0;     % 1 second total cycle
interrupt_print_duration = 0.04; % Very brief print (4% of cycle)
last_interrupt_print = -10;     % Initialize to prevent immediate print

%% Create figure with 4 subplots
figure('Position', [50, 50, 2000, 600], 'Color', 'white');

% Store helix history for continuous tracing (many points for smooth curves)
helix_history_cont = [];  % [x, y, z, time, angle]
helix_history_disc = [];
helix_history_comp = [];
% Store printed RINGS only for interrupt mode
printed_rings_interrupt = [];

for i = 1:length(t)
    clf;
    
    %% Continuous mode (left) - Steady print head with flowing HELIX
    subplot(1,4,1);
    hold on;
    
    % Draw steady print head ring at fixed height
    theta_ring = linspace(0, 2*pi, 100);  % More points for smoother ring
    x_head = radius * cos(theta_ring);
    y_head = radius * sin(theta_ring);
    z_head = ring_height * ones(size(x_head));
    plot3(x_head, y_head, z_head, 'b-', 'LineWidth', 4, 'Color', [0 0 1]);
    
    % Calculate current angle and position
    angle_cont = mod(t(i) * continuous_speed, 2*pi);
    x_tracer = radius * cos(angle_cont);
    y_tracer = radius * sin(angle_cont);
    z_tracer = ring_height;
    
    % Add current point to helix history
    current_point = [x_tracer, y_tracer, z_tracer, t(i), angle_cont];
    helix_history_cont = [helix_history_cont; current_point];
    
    % Remove old history points
    helix_history_cont = helix_history_cont(helix_history_cont(:,4) > t(i) - max_history_time, :);
    
    % Draw smooth helix from history (if we have enough points)
    if size(helix_history_cont, 1) > 2
        % Create interpolated helix with many points
        helix_time = helix_history_cont(:,4);
        helix_x = helix_history_cont(:,1);
        helix_y = helix_history_cont(:,2);
        
        % Calculate flow-down positions
        time_since_print = t(i) - helix_time;
        helix_z = ring_height - time_since_print * flow_speed;
        
        % Only draw points that are still visible
        visible_idx = helix_z > -1.5;
        if sum(visible_idx) > 2
            % Draw smooth helix curve
            plot3(helix_x(visible_idx), helix_y(visible_idx), helix_z(visible_idx), ...
                  'b-', 'LineWidth', 2, 'Color', [0 0.5 1]);
        end
    end
    
    % Draw tracer and arm on print head
    plot3(x_tracer, y_tracer, z_tracer, 'o', 'MarkerSize', 12, ...
          'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    plot3([0, x_tracer], [0, y_tracer], [z_tracer, z_tracer], 'y-', 'LineWidth', 4);
    
    title('Continuous Phasic', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X'); ylabel('Y'); zlabel('Time Flow ↓');
    axis equal; grid on;
    view(45, 30);
    xlim([-1.5,1.5]); ylim([-1.5,1.5]); zlim([-2, 3.5]);
    hold off;
    
    %% Discontinuous mode (middle left) - Interrupted printing with HELIX
    subplot(1,4,2);
    hold on;
    
    % Draw steady print head ring
    plot3(x_head, y_head, z_head, 'r-', 'LineWidth', 4, 'Color', [1 0 0]);
    
    % Calculate discontinuous phase
    cycle_time_disc = mod(t(i), disc_active_time + disc_gap_time);
    
    if cycle_time_disc < disc_active_time
        % Active phase
        angle_disc = 2*pi * (cycle_time_disc / disc_active_time);
        tracer_color = 'y';
        arm_color = 'y';
        phase_text = 'ACTIVE';
        bg_color = 'white';
        
        % Draw tracer and arm
        x_tracer = radius * cos(angle_disc);
        y_tracer = radius * sin(angle_disc);
        z_tracer = ring_height;
        
        % Add current point to helix history (only during active phase)
        current_point = [x_tracer, y_tracer, z_tracer, t(i), angle_disc];
        helix_history_disc = [helix_history_disc; current_point];
        
    else
        % Gap phase
        tracer_color = [0.3 0.3 0.3];
        arm_color = [0.3 0.3 0.3];
        phase_text = 'GAP';
        bg_color = [0.95 0.95 0.95];
        
        % Show tracer at end position
        x_tracer = radius * cos(2*pi);
        y_tracer = radius * sin(2*pi);
        z_tracer = ring_height;
    end
    
    % Remove old history points
    helix_history_disc = helix_history_disc(helix_history_disc(:,4) > t(i) - max_history_time, :);
    
    % Draw smooth helix from history (only active phase points)
% Draw separate helix segments for discontinuous mode (NO CONNECTION ACROSS GAPS)
if size(helix_history_disc, 1) > 2
    helix_time = helix_history_disc(:,4);
    helix_x = helix_history_disc(:,1);
    helix_y = helix_history_disc(:,2);
    helix_z = ring_height - (t(i) - helix_time) * flow_speed;
    
    visible_idx = helix_z > -1.5;
    if sum(visible_idx) > 2
        % Find gap boundaries by detecting angle resets
        segment_starts = [1];
        for k = 2:length(helix_time)
            time_gap = helix_time(k) - helix_time(k-1);
            angle_gap = abs(helix_history_disc(k,5) - helix_history_disc(k-1,5));
            % If large time gap OR angle reset (jump from 2π to 0), it's a segment break
            if time_gap > disc_gap_time * 0.8 || angle_gap > pi
                segment_starts = [segment_starts, k];
            end
        end
        segment_starts = [segment_starts, length(helix_time)+1];
        
        % Draw each segment separately
        for seg = 1:(length(segment_starts)-1)
            seg_start = segment_starts(seg);
            seg_end = segment_starts(seg+1) - 1;
            if seg_end > seg_start && sum(visible_idx(seg_start:seg_end)) > 1
                seg_idx = seg_start:seg_end;
                seg_visible = visible_idx(seg_idx);
                if sum(seg_visible) > 1
                    plot3(helix_x(seg_idx(seg_visible)), helix_y(seg_idx(seg_visible)), helix_z(seg_idx(seg_visible)), ...
                          'r-', 'LineWidth', 2, 'Color', [1 0.3 0.3]);
                end
            end
        end
    end
end
    
    % Draw tracer and arm
    plot3(x_tracer, y_tracer, z_tracer, 'o', 'MarkerSize', 12, ...
          'MarkerFaceColor', tracer_color, 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    plot3([0, x_tracer], [0, y_tracer], [z_tracer, z_tracer], '-', ...
          'Color', arm_color, 'LineWidth', 4);
    
    title('Discontinuous Phasic', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X'); ylabel('Y'); zlabel('Time Flow ↓');
    axis equal; grid on;
    view(45, 30);
    xlim([-1.5,1.5]); ylim([-1.5,1.5]); zlim([-2, 3.5]);
    set(gca, 'Color', bg_color);
    hold off;
    
    %% Compressed mode (middle right) - Faster printing with HELIX
    subplot(1,4,3);
    hold on;
    
    % Draw steady print head ring
    plot3(x_head, y_head, z_head, 'g-', 'LineWidth', 4, 'Color', [0 0.8 0]);
    
    % Calculate compressed phase
    cycle_time_comp = mod(t(i), comp_cycle_time);
    
    if cycle_time_comp < comp_active_time
        % Active phase (compressed)
        angle_comp = 2*pi * (cycle_time_comp / comp_active_time);
        tracer_color = 'y';
        arm_color = 'y';
        phase_text = 'ACTIVE (COMP)';
        bg_color = 'white';
        
        % Draw tracer and arm
        x_tracer = radius * cos(angle_comp);
        y_tracer = radius * sin(angle_comp);
        z_tracer = ring_height;
        
        % Add current point to helix history (only during active phase)
        current_point = [x_tracer, y_tracer, z_tracer, t(i), angle_comp];
        helix_history_comp = [helix_history_comp; current_point];
        
    else
        % Extended gap phase
        tracer_color = [0.3 0.3 0.3];
        arm_color = [0.3 0.3 0.3];
        phase_text = 'EXTENDED GAP';
        bg_color = [0.95 0.95 0.95];
        
        % Show tracer at end position
        x_tracer = radius * cos(2*pi);
        y_tracer = radius * sin(2*pi);
        z_tracer = ring_height;
    end
    
    % Remove old history points
    helix_history_comp = helix_history_comp(helix_history_comp(:,4) > t(i) - max_history_time, :);
    
    % Draw smooth helix from history
% Draw separate helix segments for compressed mode (NO CONNECTION ACROSS GAPS)
if size(helix_history_comp, 1) > 2
    helix_time = helix_history_comp(:,4);
    helix_x = helix_history_comp(:,1);
    helix_y = helix_history_comp(:,2);
    helix_z = ring_height - (t(i) - helix_time) * flow_speed;
    
    visible_idx = helix_z > -1.5;
    if sum(visible_idx) > 2
        % Find gap boundaries
        segment_starts = [1];
        for k = 2:length(helix_time)
            time_gap = helix_time(k) - helix_time(k-1);
            angle_gap = abs(helix_history_comp(k,5) - helix_history_comp(k-1,5));
            if time_gap > (comp_cycle_time - comp_active_time) * 0.8 || angle_gap > pi
                segment_starts = [segment_starts, k];
            end
        end
        segment_starts = [segment_starts, length(helix_time)+1];
        
        % Draw each segment separately
        for seg = 1:(length(segment_starts)-1)
            seg_start = segment_starts(seg);
            seg_end = segment_starts(seg+1) - 1;
            if seg_end > seg_start && sum(visible_idx(seg_start:seg_end)) > 1
                seg_idx = seg_start:seg_end;
                seg_visible = visible_idx(seg_idx);
                if sum(seg_visible) > 1
                    plot3(helix_x(seg_idx(seg_visible)), helix_y(seg_idx(seg_visible)), helix_z(seg_idx(seg_visible)), ...
                          'g-', 'LineWidth', 2, 'Color', [0.3 1 0.3]);
                end
            end
        end
    end
end
    
    % Draw tracer and arm
    plot3(x_tracer, y_tracer, z_tracer, 'o', 'MarkerSize', 12, ...
          'MarkerFaceColor', tracer_color, 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    plot3([0, x_tracer], [0, y_tracer], [z_tracer, z_tracer], '-', ...
          'Color', arm_color, 'LineWidth', 4);
    
    title('Compressed Discontinuous Phasic', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X'); ylabel('Y'); zlabel('Time Flow ↓');
    axis equal; grid on;
    view(45, 30);
    xlim([-1.5,1.5]); ylim([-1.5,1.5]); zlim([-2, 3.5]);
    set(gca, 'Color', bg_color);
    hold off;
    
%% Interrupt mode (far right) - Instant full ring printing with gaps
subplot(1,4,4);
hold on;

% Draw steady print head ring
plot3(x_head, y_head, z_head, 'm-', 'LineWidth', 4, 'Color', [1 0 1]);

% INTERRUPT: Change to 0.5 second cycles to match continuous mode
interrupt_cycle_time = 0.5;     % 0.5 second total cycle (same as continuous!)
interrupt_print_duration = 0.02; % Very brief print (4% of cycle)
cycle_time_int = mod(t(i), interrupt_cycle_time);

if cycle_time_int < interrupt_print_duration
    % INSTANT PRINTING PHASE - Show complete ring
    tracer_color = 'y';
    arm_color = 'y';
    phase_text = 'INSTANT PRINT!';
    bg_color = [1 0.9 0.9];  % Light red flash
    
    % Draw complete ring at print head (all points simultaneously)
    x_ring = radius * cos(theta_ring);
    y_ring = radius * sin(theta_ring);
    z_ring = ring_height * ones(size(x_ring));
    plot3(x_ring, y_ring, z_ring, 'm-', 'LineWidth', 4, 'Color', [1 0 1]);
    
    % Show tracer at phase=0 position
    x_tracer = radius * cos(0);
    y_tracer = radius * sin(0);
    z_tracer = ring_height;
    
    plot3(x_tracer, y_tracer, z_tracer, 'o', 'MarkerSize', 14, ...
          'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    plot3([0, x_tracer], [0, y_tracer], [z_tracer, z_tracer], 'y-', 'LineWidth', 5);
    
    % Print a new ring to flow downward (ONLY ONCE per cycle)
    if cycle_time_int < dt/2  % More precise timing to prevent double-printing
        printed_rings_interrupt = [printed_rings_interrupt; [t(i), 0]];
    end
    
else
    % GAP PHASE - Show nothing printing, just waiting
    tracer_color = [0.3 0.3 0.3];
    arm_color = [0.3 0.3 0.3];
    
    % Show dim tracer at phase=0 position
    x_tracer = radius * cos(0);
    y_tracer = radius * sin(0);
    z_tracer = ring_height;
    
    plot3(x_tracer, y_tracer, z_tracer, 'o', 'MarkerSize', 10, ...
          'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', 'k');
    plot3([0, x_tracer], [0, y_tracer], [z_tracer, z_tracer], '-', ...
          'Color', [0.5 0.5 0.5], 'LineWidth', 2);
    
    time_until_next = interrupt_cycle_time - cycle_time_int;
    phase_text = sprintf('GAP: next in %.1fs', time_until_next);
    bg_color = [0.95 0.95 0.95];  % Gray background during gap
end
    
    % Manage printed RINGS for interrupt mode
    new_printed_int = [];
    for j = 1:size(printed_rings_interrupt, 1)
        ring_data = printed_rings_interrupt(j,:);
        print_time = ring_data(1);
        print_angle = ring_data(2);
        age = t(i) - print_time;
        
        if age < max_history_time
            z_pos = ring_height - age * flow_speed;
            opacity = 1 - (age / max_history_time);
            base_color = [1 0 1];  % Magenta color for interrupt mode
            faded_color = base_color * opacity;
            
            % Draw complete ring (always at phase=0 for interrupt mode)
            x_print = radius * cos(theta_ring);
            y_print = radius * sin(theta_ring);
            z_print = z_pos * ones(size(x_print));
            
            plot3(x_print, y_print, z_print, '-', 'LineWidth', 2, ...
                  'Color', faded_color);
            
            new_printed_int = [new_printed_int; [print_time, print_angle]];
        end
    end
    printed_rings_interrupt = new_printed_int;
    
    title('Interrupt Mode', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X'); ylabel('Y'); zlabel('Time Flow ↓');
    axis equal; grid on;
    view(45, 30);
    xlim([-1.5,1.5]); ylim([-1.5,1.5]); zlim([-2, 3.5]);
    set(gca, 'Color', bg_color);
    hold off;
    
    % Add overall title (respecting your changes)
    sgtitle({'Three Phasic and One Interrupt Mode'}, ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    % Capture frame and add to video
    frame = getframe(gcf);
    writeVideo(v, frame);
    
    % Display progress
    if mod(i, 25) == 0
        fprintf('Progress: %.1f%%\n', 100*i/length(t));
    end
    
    drawnow;
end

% Close video file
close(v);
fprintf('Video saved as: %s\n', video_filename);
fprintf('Animation complete!\n');