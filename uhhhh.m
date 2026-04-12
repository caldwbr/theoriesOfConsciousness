
% Parameters
num_segments = 20;
points_per_segment = 100;

% Initialize arrays
Phi = [];
Theta = [];

% Generate the sawtooth pattern
for seg = 0:num_segments-1
    % Only use first 1/4 of each segment for the 0→2π theta progression
    segment_fraction = linspace(0, 1, points_per_segment);
    valid_indices = segment_fraction < 0.25;  % Only first 1/4 of segment
    
    % Phi: continuous progression from 0 to 2π over all segments
    phi_segment = linspace(seg * 2*pi/num_segments, (seg+1) * 2*pi/num_segments, points_per_segment);
    
    % Theta: 0→2π in first 1/4, then undefined (NaN) for remaining 3/4
    theta_segment = nan(1, points_per_segment);
    theta_segment(valid_indices) = linspace(0, 2*pi, sum(valid_indices));
    
    % Append to main arrays
    Phi = [Phi, phi_segment];
    Theta = [Theta, theta_segment];
end

% Create the plot
figure('Position', [100, 100, 1200, 900]);
hold on;

% Add grey vertical stripes for undefined portions
for seg = 0:num_segments-1
    % Each segment covers 2π/20 = π/10 in Phi
    segment_start = seg * 2*pi/num_segments;
    segment_end = (seg + 1) * 2*pi/num_segments;
    
    % Grey stripe covers last 3/4 of each segment
    stripe_start = segment_start + 0.25 * (segment_end - segment_start);
    stripe_end = segment_end;
    
    % Draw grey rectangle for undefined portion
    patch([stripe_start, stripe_end, stripe_end, stripe_start], ...
          [0, 0, 2*pi, 2*pi], [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end

% Plot the line on top of the stripes
plot(Phi, Theta, 'b-', 'LineWidth', 2);

% Set axis limits and labels
xlim([0, 2*pi]);
ylim([0, 2*pi]);

% LARGE TICK LABELS FIRST
set(gca, 'FontSize', 20);  % This makes the tick numbers large
xticks([0, pi/2, pi, 3*pi/2, 2*pi]);
xticklabels({'0', '\pi/2', '\pi', '3\pi/2', '2\pi (77 ms)'});
yticks([0, pi/2, pi, 3*pi/2, 2*pi]);
yticklabels({'0', '\pi/2', '\pi', '3\pi/2', '2\pi'});

% Add grid for better visualization
grid on;
set(gca, 'GridAlpha', 0.3);

% NOW FORCE THE LABELS TO BE LARGE - NUCLEAR OPTION
% Get explicit handles to the labels and set properties individually
xlab = xlabel('$\Phi$', 'Interpreter', 'latex');
ylab = ylabel('$\theta$', 'Interpreter', 'latex');
titl = title('13 Hz Photic Drive Breakdown (20 Cards, 4 ms apart, 260 Hz): $\Phi$ continuous, $\theta$ in 1/4 segments', 'Interpreter', 'latex');

% FORCE the font sizes using the handle objects
set(xlab, 'FontSize', 40);
set(ylab, 'FontSize', 40);
set(titl, 'FontSize', 20);

% EXTRA FORCE: Also set the parent axes label font size multiplier
set(gca, 'LabelFontSizeMultiplier', 2.0);

% FINAL FORCE: Refresh everything
drawnow;

hold off;







% Parameters
num_segments = 20;
points_per_segment = 100;

% Initialize arrays
Phi = [];
Theta = [];

% Generate the sawtooth pattern
for seg = 0:num_segments-1
    % Only use first 1/4 of each segment for the 0→2π theta progression
    segment_fraction = linspace(0, 1, points_per_segment);
    valid_indices = segment_fraction < 0.25;  % Only first 1/4 of segment
    
    % Phi: continuous progression from 0 to 2π over all segments
    phi_segment = linspace(seg * 2*pi/num_segments, (seg+1) * 2*pi/num_segments, points_per_segment);
    
    % Theta: 0→2π in first 1/4, then undefined (NaN) for remaining 3/4
    theta_segment = nan(1, points_per_segment);
    theta_segment(valid_indices) = linspace(0, 2*pi, sum(valid_indices));
    
    % Append to main arrays
    Phi = [Phi, phi_segment];
    Theta = [Theta, theta_segment];
end

%% 3D Torus Plot
figure('Position', [100, 100, 1000, 800]);
hold on;

% Torus parameters
R = 3;  % Major radius
r = 1;  % Minor radius

% Create torus surface
[u, v] = meshgrid(linspace(0, 2*pi, 50), linspace(0, 2*pi, 30));
X_torus = (R + r*cos(v)) .* cos(u);
Y_torus = (R + r*cos(v)) .* sin(u);
Z_torus = r*sin(v);

% Plot semi-transparent torus
surf(X_torus, Y_torus, Z_torus, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);

% Plot the 3D path on torus (only defined segments in blue)
for seg = 0:num_segments-1
    seg_start = seg * points_per_segment + 1;
    seg_end = (seg + 1) * points_per_segment;
    seg_phi = Phi(seg_start:seg_end);
    seg_theta = Theta(seg_start:seg_end);
    
    % Find valid (defined) points in this segment
    valid_mask = ~isnan(seg_theta);
    valid_phi = seg_phi(valid_mask);
    valid_theta = seg_theta(valid_mask);
    
    if length(valid_phi) > 1
        % Convert to 3D coordinates on torus
        X_seg = (R + r*cos(valid_theta)) .* cos(valid_phi);
        Y_seg = (R + r*cos(valid_theta)) .* sin(valid_phi);
        Z_seg = r*sin(valid_theta);
        
        % Plot blue line for defined portions
        plot3(X_seg, Y_seg, Z_seg, 'b-', 'LineWidth', 3);
        
        % Plot grey dashed line for undefined portions (connecting segments)
        if seg < num_segments-1
            next_seg_start = (seg + 1) * points_per_segment + 1;
            next_valid_phi = Phi(next_seg_start);
            next_valid_theta = Theta(next_seg_start);
            
            if ~isnan(next_valid_theta)
                % Connect end of current segment to start of next segment
                X_conn = [X_seg(end), (R + r*cos(next_valid_theta)) * cos(next_valid_phi)];
                Y_conn = [Y_seg(end), (R + r*cos(next_valid_theta)) * sin(next_valid_phi)];
                Z_conn = [Z_seg(end), r*sin(next_valid_theta)];
                
                plot3(X_conn, Y_conn, Z_conn, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
            end
        end
    end
end

% 3D plot formatting
axis equal;
grid on;

% LARGE LABELS
xlab = xlabel('X', 'FontSize', 20);
ylab = ylabel('Y', 'FontSize', 20);
zlab = zlabel('Z', 'FontSize', 20);
titl = title('13 Hz Photic Drive Breakdown on Torus', 'FontSize', 24);

% Add toroidal and poloidal direction indicators with labels
% Toroidal arrow (blue)
phi_arrow = linspace(0, pi/2, 20);
theta_fixed = 0;
X_toroidal = (R + r*cos(theta_fixed)) .* cos(phi_arrow);
Y_toroidal = (R + r*cos(theta_fixed)) .* sin(phi_arrow);
Z_toroidal = r*sin(theta_fixed) * ones(size(phi_arrow));
plot3(X_toroidal, Y_toroidal, Z_toroidal, 'c-', 'LineWidth', 4);
text(X_toroidal(end), Y_toroidal(end), Z_toroidal(end), '  Φ (toroidal)', 'FontSize', 16, 'Color', 'c');

% Poloidal arrow (red)
phi_fixed = 0;
theta_arrow = linspace(0, pi/2, 20);
X_poloidal = (R + r*cos(theta_arrow)) .* cos(phi_fixed);
Y_poloidal = (R + r*cos(theta_arrow)) .* sin(phi_fixed);
Z_poloidal = r*sin(theta_arrow);
plot3(X_poloidal, Y_poloidal, Z_poloidal, 'r-', 'LineWidth', 4);
text(X_poloidal(end), Y_poloidal(end), Z_poloidal(end), '  θ (poloidal)', 'FontSize', 16, 'Color', 'r');

% Legend
legend('Torus Surface', 'Defined Path (θ: 0→2π)', 'Undefined Jumps', 'Location', 'best', 'FontSize', 14);

view(45, 30);  % Nice 3D view
set(gca, 'FontSize', 16);  % Large tick labels

drawnow;