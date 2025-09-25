%% Three-neuron sinusoid -> time plots + PCA ring
% Requires Statistics & Machine Learning Toolbox for 'pca'.

clear; clc;

% ----- Parameters -----
fs   = 1000;                 % sample rate (Hz) for smooth plots
T    = 5;                    % total duration (s)
t    = (0:1/fs:T)';          % time column
f0   = 35;                   % mean firing rate (Hz)
A    = 25;                   % modulation amplitude (Hz) => 10..60
fmod = 1;                    % 1 cycle per second
phi  = [0, 2*pi/3, 4*pi/3];  % 0°, 120°, 240° (neuron 1 is cosine)

% ----- Frequency traces (Hz) -----
theta = 2*pi*fmod*t;  % base phase vs time
f1 = f0 + A*cos(theta + phi(1));
f2 = f0 + A*cos(theta + phi(2));
f3 = f0 + A*cos(theta + phi(3));

% Zero-baseline (ΔHz from mean)
d1 = f1 - f0;
d2 = f2 - f0;
d3 = f3 - f0;

% ----- Plot setup -----
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

% 1) N1
nexttile;
plot(t,d1,'LineWidth',1.5); hold on; yline(0,'k:');
xlabel('Time (s)'); ylabel('\DeltaHz'); title('Neuron 1 (cos, 0^\circ)');
ylim([-A A]); grid on;

% 2) N2
nexttile;
plot(t,d2,'LineWidth',1.5); hold on; yline(0,'k:');
xlabel('Time (s)'); ylabel('\DeltaHz'); title('Neuron 2 (+120^\circ)');
ylim([-A A]); grid on;

% 3) N3
nexttile;
plot(t,d3,'LineWidth',1.5); hold on; yline(0,'k:');
xlabel('Time (s)'); ylabel('\DeltaHz'); title('Neuron 3 (+240^\circ)');
ylim([-A A]); grid on;

% ----- 4) PCA tracer -----
% Data matrix (samples x neurons)
X = [f1, f2, f3];

% PCA (centers by column mean internally) -> centroid is ~[35 35 35]
[coeff, score, latent, ~, explained, mu] = pca(X);

% 3D trajectory in PC space
nexttile;
% color by time to see motion
cmap = turbo(length(t));
plot3(score(:,1), score(:,2), score(:,3), 'Color',[0.2 0.2 0.2], 'LineWidth',1); hold on;
scatter3(score(1:50:end,1), score(1:50:end,2), score(1:50:end,3), 10, cmap(1:50:end,:), 'filled');

% Mark the centroid (mean firing rates of all 3 neurons)
scatter3(0,0,0,60,'kp','filled');  % centroid is origin in score space

% ---- Compute analytic min/max times and mark them along the PCA curve ----
% For cosine: max when cos=1, min when cos=-1.
% t_max_k = k - phi/(2pi);  t_min_k = k + 0.5 - phi/(2pi)
markStyles = {'ro','go','bo'; 'r^','g^','b^'}; % [min; max] by neuron colors
namesMin   = {'N1 min','N2 min','N3 min'};
namesMax   = {'N1 max','N2 max','N3 max'};
legHandles = gobjects(1,7); legText = strings(1,7);
L = 0;  % legend counter

for n = 1:3
    ph  = phi(n);
    % all integer k that yield times in [0,T]
    kmin = ceil( (0   + ph/(2*pi)) - 0.5 );
    kmax = floor((T   + ph/(2*pi)) - 0.5 );
    tmins = ( (0.5 + (kmin:kmax)) - ph/(2*pi) )';
    kmin = ceil( 0   + ph/(2*pi) );
    kmax = floor( T  + ph/(2*pi) );
    tmaxs = ( ((kmin:kmax) - ph/(2*pi)) )';

    % map times to nearest indices
    idxMin = max(1, min(length(t), round(tmins*fs)+1));
    idxMax = max(1, min(length(t), round(tmaxs*fs)+1));

    % PCA coordinates at those instants
    sMin = score(idxMin, 1:3);
    sMax = score(idxMax, 1:3);

    h1 = scatter3(sMin(:,1), sMin(:,2), sMin(:,3), 40, markStyles{1,n}(1), ...
        'filled','MarkerEdgeColor','k'); %#ok<NASGU>
    h2 = scatter3(sMax(:,1), sMax(:,2), sMax(:,3), 60, markStyles{2,n}(1), ...
        'filled','MarkerEdgeColor','k'); %#ok<NASGU>

    % put one representative handle in legend
    L=L+1; legHandles(L)=scatter3(sMin(1,1),sMin(1,2),sMin(1,3),40,markStyles{1,n}(1),'filled','MarkerEdgeColor','k');
    legText(L)=namesMin{n};
    L=L+1; legHandles(L)=scatter3(sMax(1,1),sMax(1,2),sMax(1,3),60,markStyles{2,n}(1),'filled','MarkerEdgeColor','k');
    legText(L)=namesMax{n};
end

axis equal; grid on;
xlabel(sprintf('PC1 (%.1f%%)',explained(1)));
ylabel(sprintf('PC2 (%.1f%%)',explained(2)));
zlabel(sprintf('PC3 (%.1f%%)',explained(3)));
title('PCA Trajectory (centroid at mean rates)');
view(35,25);

% Legend (plus centroid)
L=L+1; legHandles(L)=scatter3(0,0,0,60,'kp','filled'); legText(L)="Centroid (mean 35 Hz each)";
legend(legHandles(1:L), legText(1:L), 'Location','bestoutside');

% ----- Optional: print min/max per neuron in original units -----
fprintf('Means (mu): N1=%.2f, N2=%.2f, N3=%.2f Hz\n', mu);
fprintf('Ranges (Hz):\n');
fprintf('  N1: [%.1f, %.1f]\n', min(f1), max(f1));
fprintf('  N2: [%.1f, %.1f]\n', min(f2), max(f2));
fprintf('  N3: [%.1f, %.1f]\n', min(f3), max(f3));







%Second try
%% Three neurons: animated plots + PCA + audio clicks -> MP4 (5 s)
% If you have Computer Vision Toolbox, the MP4 will include audio.
% Otherwise, the script saves the video (silent) + a separate WAV.

clear; clc; rng(7);

% -------------------- Parameters --------------------
T       = 5;          % seconds
fpsVid  = 60;         % video frame rate
fsAud   = 44100;      % audio sample rate
fsPlot  = 1000;       % for smooth plotted curves

meanHz  = 35;         % average firing rate
ampHz   = 25;         % modulation amplitude => 10..60 Hz
fmod    = 1;          % 1 cycle / second
phi     = [0, 2*pi/3, 4*pi/3];  % N1 cosine, N2 +120°, N3 +240°

% audio click tones per neuron (short decays)
toneHz  = [1100, 1500, 1900];   % different "pitch" for each neuron
clickDur= 0.008;                % 8 ms click
tau     = 0.003;                % decay constant (s)
panLR   = [0.8 0.2; 0.5 0.5; 0.2 0.8]; % L/R panning per neuron

% -------------------- Time bases --------------------
tPlot = (0:1/fsPlot:T)';                % for plotting
theta = 2*pi*fmod*tPlot;

% frequencies (Hz)
f1 = meanHz + ampHz*cos(theta + phi(1));
f2 = meanHz + ampHz*cos(theta + phi(2));
f3 = meanHz + ampHz*cos(theta + phi(3));

% -------------------- PCA (on absolute Hz) --------------------
X = [f1, f2, f3];                       % samples x neurons
[coeff, score, latent, ~, explained] = pca(X); %#ok<ASGLU>

% -------------------- Generate spike trains & audio --------------------
tAud  = (0:1/fsAud:T-1/fsAud)';         % exactly 5 s
thetaA= 2*pi*fmod*tAud;
F     = [ meanHz + ampHz*cos(thetaA + phi(1)), ...
          meanHz + ampHz*cos(thetaA + phi(2)), ...
          meanHz + ampHz*cos(thetaA + phi(3)) ];
p     = F / fsAud;                      % Bernoulli p per sample

spikes = rand(size(F)) < p;             % inhomog. Poisson (discretized)
N      = size(spikes,1);

% click kernels (decaying short tones)
k = round(clickDur*fsAud);
tC = (0:k-1)'/fsAud;
H = [sin(2*pi*toneHz(1)*tC).*exp(-tC/tau), ...
     sin(2*pi*toneHz(2)*tC).*exp(-tC/tau), ...
     sin(2*pi*toneHz(3)*tC).*exp(-tC/tau)];

% convolve impulses with kernels, then pan to stereo
yL = zeros(N,1); yR = zeros(N,1);
for n = 1:3
    imp = double(spikes(:,n));
    y  = conv(imp, H(:,n), 'same');
    yL = yL + panLR(n,1)*y;
    yR = yR + panLR(n,2)*y;
end
audio = [yL yR];
audio = 0.95 * audio / max(1e-9, max(abs(audio),[],'all'));  % normalize

% samples per video frame (exact integer at 44.1k/60 fps)
spf = fsAud / fpsVid;    % 735 exactly
assert(abs(spf-round(spf))<1e-9,'fsAud/fpsVid must be integer.');
spf = round(spf);
nFrames = T*fpsVid;      % 300 frames

% -------------------- Figure & static plots --------------------
fig = figure('Color','w','Position',[100 100 900 1100]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

% Axes 1..3: absolute Hz with 35 Hz baseline
yl = [10 60];

nexttile; p1 = plot(tPlot,f1,'b','LineWidth',1.2); hold on;
h1 = plot(tPlot(1), f1(1), 'bo','MarkerFaceColor','b'); yline(meanHz,'k:','35 Hz');
title('Neuron 1 (cos, 0^\circ)'); xlabel('Time (s)'); ylabel('Hz'); ylim(yl); grid on;

nexttile; p2 = plot(tPlot,f2,'m','LineWidth',1.2); hold on;
h2 = plot(tPlot(1), f2(1), 'mo','MarkerFaceColor','m'); yline(meanHz,'k:','35 Hz');
title('Neuron 2 (+120^\circ)'); xlabel('Time (s)'); ylabel('Hz'); ylim(yl); grid on;

nexttile; p3 = plot(tPlot,f3,'g','LineWidth',1.2); hold on;
h3 = plot(tPlot(1), f3(1), 'go','MarkerFaceColor','g'); yline(meanHz,'k:','35 Hz');
title('Neuron 3 (+240^\circ)'); xlabel('Time (s)'); ylabel('Hz'); ylim(yl); grid on;

% PCA ring (centroid at origin in score-space)
nexttile;
plot3(score(:,1),score(:,2),score(:,3),'Color',[.7 .7 .7],'LineWidth',1); hold on;
hp = plot3(score(1,1),score(1,2),score(1,3),'ko','MarkerFaceColor','r','MarkerSize',6);
scatter3(0,0,0,70,'kp','filled'); % centroid
axis equal; grid on;
xlabel(sprintf('PC1 (%.1f%%)', explained(1)));
ylabel(sprintf('PC2 (%.1f%%)', explained(2)));
zlabel(sprintf('PC3 (%.1f%%)', explained(3)));
title('PCA Trajectory with Tracer'); view(35,25);

% -------------------- Video writer (with/without audio) --------------------
haveCV = (exist('vision.VideoFileWriter','class')==8);
if haveCV
    vw = vision.VideoFileWriter('neuron_pca_demo.mp4', ...
         'FileFormat','MPEG4','FrameRate',fpsVid, ...
         'AudioInputPort',true, 'AudioDataType','double', ...
         'AudioSampleRate',fsAud);
else
    vw = VideoWriter('neuron_pca_demo.mp4','MPEG-4');
    vw.FrameRate = fpsVid; open(vw);
end

% -------------------- Animate & write --------------------
for kf = 1:nFrames
    % time for this frame
    tNow = (kf-1)/fpsVid;
    % closest plot index
    idxP = max(1, min(length(tPlot), round(tNow*fsPlot)+1));
    set(h1, 'XData', tPlot(idxP), 'YData', f1(idxP));
    set(h2, 'XData', tPlot(idxP), 'YData', f2(idxP));
    set(h3, 'XData', tPlot(idxP), 'YData', f3(idxP));
    set(hp, 'XData', score(idxP,1), 'YData', score(idxP,2), 'ZData', score(idxP,3));

    drawnow limitrate;

    % grab frame
    Fr = getframe(fig);
    img = Fr.cdata;

    % corresponding audio slice
    a0 = (kf-1)*spf + 1;
    a1 = kf*spf;
    a1 = min(a1, size(audio,1));  % safety
    aChunk = audio(a0:a1,:);

    if haveCV
        step(vw, img, aChunk);
    else
        writeVideo(vw, img);
    end
end

if haveCV
    release(vw);
else
    close(vw);
    audiowrite('neuron_clicks.wav', audio, fsAud);
    fprintf('Wrote neuron_pca_demo.mp4 (silent) + neuron_clicks.wav (audio).\n');
end



%Third try
%% Three neurons -> animated time plots + RAW 3D ring + audio clicks (5 s)
clear; clc; rng(7);

% -------------------- Parameters --------------------
T       = 5;          % seconds
fpsVid  = 60;         % video frame rate
fsAud   = 44100;      % audio sample rate
fsPlot  = 1000;       % plotting sample rate

meanHz  = 35;         % mean firing rate
ampHz   = 25;         % amplitude => 10..60 Hz
fmod    = 1;          % 1 cycle / second
phi     = [0, 2*pi/3, 4*pi/3];     % N1=cos, N2 +120°, N3 +240°

% audio click params (different pitch per neuron)
toneHz  = [1100, 1500, 1900];
clickDur= 0.008;      % 8 ms
tau     = 0.003;      % decay (s)
panLR   = [0.8 0.2; 0.5 0.5; 0.2 0.8]; % stereo pan L/R

% -------------------- Time bases --------------------
tPlot = (0:1/fsPlot:T)';             % for curves
theta = 2*pi*fmod*tPlot;

% Frequencies (absolute Hz)
f1 = meanHz + ampHz*cos(theta + phi(1));
f2 = meanHz + ampHz*cos(theta + phi(2));
f3 = meanHz + ampHz*cos(theta + phi(3));

% -------------------- Spike trains & audio --------------------
tAud  = (0:1/fsAud:T-1/fsAud)';      % exactly 5 s
thetaA= 2*pi*fmod*tAud;
F     = [ meanHz + ampHz*cos(thetaA + phi(1)), ...
          meanHz + ampHz*cos(thetaA + phi(2)), ...
          meanHz + ampHz*cos(thetaA + phi(3)) ];
p     = F / fsAud;                   % Bernoulli per-sample rate
spikes = rand(size(F)) < p;

% click kernels
k  = round(clickDur*fsAud);
tC = (0:k-1)'/fsAud;
H  = [sin(2*pi*toneHz(1)*tC).*exp(-tC/tau), ...
      sin(2*pi*toneHz(2)*tC).*exp(-tC/tau), ...
      sin(2*pi*toneHz(3)*tC).*exp(-tC/tau)];

% convolve & pan to stereo
N   = size(spikes,1);
yL = zeros(N,1); yR = zeros(N,1);
for n = 1:3
    y  = conv(double(spikes(:,n)), H(:,n), 'same');
    yL = yL + panLR(n,1)*y;
    yR = yR + panLR(n,2)*y;
end
audio = [yL yR];
audio = 0.95 * audio / max(1e-9, max(abs(audio),[],'all')); % normalize

% tie audio to frames
spf     = fsAud / fpsVid;            % 735 exactly at 44.1k/60fps
assert(abs(spf-round(spf))<1e-9);
spf     = round(spf);
nFrames = T*fpsVid;

% -------------------- Figure --------------------
fig = figure('Color','w','Position',[120 80 980 1120]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
yl = [10 60];

% 1) N1 time plot (absolute Hz) with tracer
nexttile; plot(tPlot,f1,'b','LineWidth',1.2); hold on;
h1 = plot(tPlot(1), f1(1), 'bo','MarkerFaceColor','b');
yline(meanHz,'k:','35 Hz'); ylim(yl); grid on;
title('Neuron 1 (cos, 0^\circ)'); xlabel('Time (s)'); ylabel('Hz');

% 2) N2
nexttile; plot(tPlot,f2,'m','LineWidth',1.2); hold on;
h2 = plot(tPlot(1), f2(1), 'mo','MarkerFaceColor','m');
yline(meanHz,'k:','35 Hz'); ylim(yl); grid on;
title('Neuron 2 (+120^\circ)'); xlabel('Time (s)'); ylabel('Hz');

% 3) N3
nexttile; plot(tPlot,f3,'g','LineWidth',1.2); hold on;
h3 = plot(tPlot(1), f3(1), 'go','MarkerFaceColor','g');
yline(meanHz,'k:','35 Hz'); ylim(yl); grid on;
title('Neuron 3 (+240^\circ)'); xlabel('Time (s)'); ylabel('Hz');

% 4) RAW 3-D ring in neuron frequency space
nexttile;
plot3(f1, f2, f3, 'Color',[.7 .7 .7], 'LineWidth',1); hold on;
hp = plot3(f1(1), f2(1), f3(1), 'ro','MarkerFaceColor','r','MarkerSize',6);
scatter3(meanHz, meanHz, meanHz, 70, 'kp', 'filled'); % centroid
xlim(yl); ylim(yl); zlim(yl); axis vis3d;
grid on; view(35,25);
xlabel('Neuron 1 (Hz)'); ylabel('Neuron 2 (Hz)'); zlabel('Neuron 3 (Hz)');
title('Raw Frequency Space: [f1(t), f2(t), f3(t)]');

% Optional: line from centroid to tracer (nice visual)
lineCentroid = plot3([meanHz f1(1)],[meanHz f2(1)],[meanHz f3(1)],'r-','LineWidth',1);

% -------------------- Video writer --------------------
haveCV = (exist('vision.VideoFileWriter','class')==8);
if haveCV
    vw = vision.VideoFileWriter('neuron_raw3D_demo.mp4', ...
         'FileFormat','MPEG4','FrameRate',fpsVid, ...
         'AudioInputPort',true, 'AudioDataType','double', ...
         'AudioSampleRate',fsAud);
else
    vw = VideoWriter('neuron_raw3D_demo.mp4','MPEG-4');
    vw.FrameRate = fpsVid; open(vw);
end

% -------------------- Animate --------------------
for kf = 1:nFrames
    tNow  = (kf-1)/fpsVid;
    idxP  = max(1, min(length(tPlot), round(tNow*fsPlot)+1));
    % move tracers
    set(h1,'XData',tPlot(idxP),'YData',f1(idxP));
    set(h2,'XData',tPlot(idxP),'YData',f2(idxP));
    set(h3,'XData',tPlot(idxP),'YData',f3(idxP));
    set(hp,'XData',f1(idxP),'YData',f2(idxP),'ZData',f3(idxP));
    set(lineCentroid,'XData',[meanHz f1(idxP)], ...
                     'YData',[meanHz f2(idxP)], ...
                     'ZData',[meanHz f3(idxP)]);
    drawnow limitrate;

    % capture frame + aligned audio slice
    Fr = getframe(fig); img = Fr.cdata;
    a0 = (kf-1)*spf + 1; a1 = min(kf*spf, size(audio,1));
    aChunk = audio(a0:a1,:);

    if haveCV, step(vw, img, aChunk); else, writeVideo(vw, img); end
end

if haveCV
    release(vw);
else
    close(vw); audiowrite('neuron_clicks.wav', audio, fsAud);
    fprintf('Wrote neuron_raw3D_demo.mp4 (silent) + neuron_clicks.wav.\n');
end



%Fourth try
%% Three neurons -> 2x2 animated plots + RAW 3D ring + audio clicks (5 s) + muxed video
clear; clc; rng(7);

% -------------------- Params --------------------
T        = 5;            % seconds
fpsVid   = 60;           % frames/s
fsAud    = 44100;        % audio rate
fsPlot   = 1000;         % plot curve sampling
meanHz   = 35; ampHz = 25; fmod = 1;       % 10..60 Hz, 1 Hz modulation
phi      = [0, 2*pi/3, 4*pi/3];            % 0°,120°,240°
toneHz   = [1100, 1500, 1900];             % per-neuron click pitch
clickDur = 0.008; tau = 0.003;             % click envelope
panLR    = [0.8 0.2; 0.5 0.5; 0.2 0.8];    % stereo panning
yl       = [10 60];

% -------------------- Time bases + curves --------------------
tPlot = (0:1/fsPlot:T)'; th = 2*pi*fmod*tPlot;
f1 = meanHz + ampHz*cos(th + phi(1));
f2 = meanHz + ampHz*cos(th + phi(2));
f3 = meanHz + ampHz*cos(th + phi(3));

% ---------- Deterministic spike trains (exact instantaneous rate) ----------
tAud  = (0:1/fsAud:T-1/fsAud)';           % exactly 5 s
thetaA= 2*pi*fmod*tAud;
F     = [ meanHz + ampHz*cos(thetaA + phi(1)), ...
          meanHz + ampHz*cos(thetaA + phi(2)), ...
          meanHz + ampHz*cos(thetaA + phi(3)) ];   % Hz, Nx3

N = size(F,1);
acc = zeros(1,3);                          % phase accumulator in "cycles"
spikes = false(N,3);

for n = 1:N
    acc = acc + F(n,:)/fsAud;              % add cycles this sample
    hits = floor(acc);                     % number of crossings (0 or 1 here)
    spikes(n, hits>0) = true;              % emit click(s)
    acc = acc - hits;                      % wrap accumulator
end

% ---------- Click kernel (crisper & shorter so 60 Hz stays punchy) ----------
clickDur = 0.004;                          % 4 ms
tau      = 0.0015;                         % faster decay
toneHz   = [2000, 2600, 3300];             % brighter pitches

k  = round(clickDur*fsAud);
tC = (0:k-1)'/fsAud;
% half-sine "tick" with exp decay (crisper than a long tone)
base = sin(pi * (tC/clickDur));
H = [base.*exp(-tC/tau), base.*exp(-tC/tau), base.*exp(-tC/tau)];
% optionally modulate each with a short carrier to give pitch:
H = H .* [sin(2*pi*toneHz(1)*tC), sin(2*pi*toneHz(2)*tC), sin(2*pi*toneHz(3)*tC)];

% Convolve & pan to stereo
yL = zeros(N,1); yR = zeros(N,1);
panLR = [0.8 0.2; 0.5 0.5; 0.2 0.8];
for i = 1:3
    y  = conv(double(spikes(:,i)), H(:,i), 'same');
    yL = yL + panLR(i,1)*y;
    yR = yR + panLR(i,2)*y;
end
audio = [yL yR];
audio = 0.95 * audio / max(1e-9, max(abs(audio),[],'all'));  % normalize


% -------------------- Figure & axes (2x2 grid) --------------------
fig = figure('Color','w','Position',[80 80 1400 1050]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% N1
ax1 = nexttile(1); plot(tPlot,f1,'b','LineWidth',1.2); hold on;
h1  = plot(tPlot(1),f1(1),'bo','MarkerFaceColor','b');
yline(meanHz,'k:','35 Hz'); title('Neuron 1 (cos, 0^\circ)'); xlabel('Time (s)'); ylabel('Hz'); ylim(yl); grid on;

% N2
ax2 = nexttile(2); plot(tPlot,f2,'m','LineWidth',1.2); hold on;
h2  = plot(tPlot(1),f2(1),'mo','MarkerFaceColor','m');
yline(meanHz,'k:','35 Hz'); title('Neuron 2 (+120^\circ)'); xlabel('Time (s)'); ylabel('Hz'); ylim(yl); grid on;

% N3
ax3 = nexttile(3); plot(tPlot,f3,'g','LineWidth',1.2); hold on;
h3  = plot(tPlot(1),f3(1),'go','MarkerFaceColor','g');
yline(meanHz,'k:','35 Hz'); title('Neuron 3 (+240^\circ)'); xlabel('Time (s)'); ylabel('Hz'); ylim(yl); grid on;

% RAW 3-D ring
ax4 = nexttile(4);
plot3(f1,f2,f3,'Color',[.7 .7 .7],'LineWidth',1); hold on;
hp = plot3(f1(1),f2(1),f3(1),'ro','MarkerFaceColor','r','MarkerSize',7);
scatter3(meanHz,meanHz,meanHz,90,'kp','filled');  % centroid
lineCentroid = plot3([meanHz f1(1)],[meanHz f2(1)],[meanHz f3(1)],'r-','LineWidth',1);
xlim(yl); ylim(yl); zlim(yl); axis vis3d; grid on; view(35,25);
xlabel('Neuron 1 (Hz)'); ylabel('Neuron 2 (Hz)'); zlabel('Neuron 3 (Hz)');
title('Raw Frequency Space: [f1(t), f2(t), f3(t)]');

% -------------------- Video writer (audio-aware) --------------------
useCVT = (exist('vision.VideoFileWriter','class')==8);
vidSilent = 'neuron_raw3D_demo.mp4';
vidFinal  = 'neuron_raw3D_with_audio.mp4';
wavFile   = 'neuron_clicks.wav';

if useCVT
    vw = vision.VideoFileWriter(vidFinal, ...
        'FileFormat','MPEG4','FrameRate',fpsVid, ...
        'AudioInputPort',true,'AudioDataType','double', ...
        'AudioCompressor','AAC','AudioSampleRate',fsAud);
else
    vw = VideoWriter(vidSilent,'MPEG-4'); vw.FrameRate = fpsVid; open(vw);
end

% -------------------- Animate (tear-free capture) --------------------
for kf = 1:nF
    tNow = (kf-1)/fpsVid;
    i    = max(1, min(length(tPlot), round(tNow*fsPlot)+1));
    % move tracers
    set(h1,'XData',tPlot(i),'YData',f1(i));
    set(h2,'XData',tPlot(i),'YData',f2(i));
    set(h3,'XData',tPlot(i),'YData',f3(i));
    set(hp,'XData',f1(i),'YData',f2(i),'ZData',f3(i));
    set(lineCentroid,'XData',[meanHz f1(i)],'YData',[meanHz f2(i)],'ZData',[meanHz f3(i)]);
    drawnow;   % ensure full render

    % robust off-screen frame (avoids diagonal tearing)
    frameRGB = print(fig,'-RGBImage','-r150');   % returns an MxNx3 uint8
    % audio slice for this frame
    a0 = (kf-1)*spf + 1; a1 = min(kf*spf, size(audio,1));
    aChunk = audio(a0:a1,:);

    if useCVT
        step(vw, frameRGB, aChunk);
    else
        writeVideo(vw, frameRGB);
    end
end

if useCVT
    release(vw);
else
    close(vw);
    audiowrite(wavFile, audio, fsAud);
    % try to mux with ffmpeg if available
    [ok,~] = system('ffmpeg -version');
    if ok==0
        cmd = sprintf('ffmpeg -y -loglevel error -i "%s" -i "%s" -c:v copy -c:a aac -shortest "%s"', ...
                       vidSilent, wavFile, vidFinal);
        system(cmd);
        fprintf('Wrote %s (muxed with audio).\n', vidFinal);
    else
        fprintf('Wrote %s (silent) and %s (audio). Install ffmpeg to mux.\n', vidSilent, wavFile);
    end
end




%Fifth try
%% Three neurons -> 2x2 animated plots + RAW 3D ring + exact-rate audio clicks (5 s)
clear; clc; rng(7);

% ===================== Parameters =====================
T        = 5;              % seconds
fpsVid   = 60;             % frames/s
fsAud    = 44100;          % audio sample rate
fsPlot   = 1000;           % plot sampling
meanHz   = 35;             % mean rate
ampHz    = 25;             % amplitude => 10..60 Hz
fmod     = 1;              % 1 Hz modulation
phi      = [0, 2*pi/3, 4*pi/3];  % 0°, 120°, 240° phase offsets
yl       = [10 60];        % y-axis limits for Hz

% Audio click kernel (short & crisp so 60 Hz is audible as fast ticks)
clickDur = 0.004;          % 4 ms
tau      = 0.0015;         % decay time constant (s)
toneHz   = [2000, 2600, 3300];  % perceptual pitch label per neuron
panLR    = [0.8 0.2; 0.5 0.5; 0.2 0.8];  % stereo panning (L,R)

% ===================== Traces (Hz) =====================
tPlot = (0:1/fsPlot:T)'; th = 2*pi*fmod*tPlot;
f1 = meanHz + ampHz*cos(th + phi(1));
f2 = meanHz + ampHz*cos(th + phi(2));
f3 = meanHz + ampHz*cos(th + phi(3));

% ===================== Exact spike times via phase accumulator =====================
tAud   = (0:1/fsAud:T-1/fsAud)';       % exactly T seconds
thA    = 2*pi*fmod*tAud;
F      = [ meanHz + ampHz*cos(thA + phi(1)), ...
           meanHz + ampHz*cos(thA + phi(2)), ...
           meanHz + ampHz*cos(thA + phi(3)) ];        % Hz at each sample

N      = size(F,1);
acc    = zeros(1,3);                    % cycles accumulator (wraps at 1)
spikes = false(N,3);

% Each sample, add F/fsAud cycles; when accumulator crosses an integer,
% emit a click. This enforces instantaneous rate exactly.
for n = 1:N
    acc = acc + F(n,:)/fsAud;           % add cycles this sample
    hits = floor(acc);                  % crossings (0 or 1 here)
    spikes(n, hits>0) = true;           % emit
    acc = acc - hits;                   % wrap
end

% ===================== Build click kernels & audio =====================
k  = round(clickDur*fsAud);
tC = (0:k-1)'/fsAud;
base = sin(pi*(tC/clickDur));           % half-sine envelope (0..pi)
env  = base .* exp(-tC/tau);            % fast decay keeps 60 Hz distinct
H    = [ env.*sin(2*pi*toneHz(1)*tC), ...
         env.*sin(2*pi*toneHz(2)*tC), ...
         env.*sin(2*pi*toneHz(3)*tC) ];

yL = zeros(N,1); yR = zeros(N,1);
for i = 1:3
    y  = conv(double(spikes(:,i)), H(:,i), 'same');
    yL = yL + panLR(i,1)*y;
    yR = yR + panLR(i,2)*y;
end
audio = [yL yR];
audio = 0.95 * audio / max(1e-9, max(abs(audio),[],'all'));  % normalize

% Tie audio to frames
spf      = round(fsAud / fpsVid);       % samples per frame (735 at 44.1k/60)
nFrames  = round(T * fpsVid);

% ===================== Figure (2x2) =====================
fig = figure('Color','w','Position',[80 80 1400 1050]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% N1
nexttile(1);
plot(tPlot,f1,'b','LineWidth',1.2); hold on;
h1 = plot(tPlot(1), f1(1), 'bo', 'MarkerFaceColor','b');
yline(meanHz,'k:','35 Hz'); ylim(yl); grid on;
title('Neuron 1 (cos, 0^\circ)'); xlabel('Time (s)'); ylabel('Hz');

% N2
nexttile(2);
plot(tPlot,f2,'m','LineWidth',1.2); hold on;
h2 = plot(tPlot(1), f2(1), 'mo', 'MarkerFaceColor','m');
yline(meanHz,'k:','35 Hz'); ylim(yl); grid on;
title('Neuron 2 (+120^\circ)'); xlabel('Time (s)'); ylabel('Hz');

% N3
nexttile(3);
plot(tPlot,f3,'g','LineWidth',1.2); hold on;
h3 = plot(tPlot(1), f3(1), 'go', 'MarkerFaceColor','g');
yline(meanHz,'k:','35 Hz'); ylim(yl); grid on;
title('Neuron 3 (+240^\circ)'); xlabel('Time (s)'); ylabel('Hz');

% RAW 3-D ring (no PCA): [f1(t), f2(t), f3(t)]
nexttile(4);
plot3(f1,f2,f3,'Color',[.7 .7 .7],'LineWidth',1); hold on;
hp = plot3(f1(1),f2(1),f3(1),'ro','MarkerFaceColor','r','MarkerSize',7);
scatter3(meanHz,meanHz,meanHz,90,'kp','filled');         % centroid
lineCentroid = plot3([meanHz f1(1)],[meanHz f2(1)],[meanHz f3(1)],'r-','LineWidth',1);
xlim(yl); ylim(yl); zlim(yl); axis vis3d; grid on; view(35,25);
xlabel('Neuron 1 (Hz)'); ylabel('Neuron 2 (Hz)'); zlabel('Neuron 3 (Hz)');
title('Raw Frequency Space: [f1(t), f2(t), f3(t)]');

% ===================== Video writer =====================
useCVT  = (exist('vision.VideoFileWriter','class')==8);
vidSilent = 'neuron_raw3D_demo.mp4';
vidFinal  = 'neuron_raw3D_with_audio.mp4';
wavFile   = 'neuron_clicks.wav';

if useCVT
    vw = vision.VideoFileWriter(vidFinal, ...
        'FileFormat','MPEG4','FrameRate',fpsVid, ...
        'AudioInputPort',true,'AudioDataType','double', ...
        'AudioCompressor','AAC','AudioSampleRate',fsAud);
else
    vw = VideoWriter(vidSilent,'MPEG-4'); vw.FrameRate = fpsVid; open(vw);
end

% ===================== Animate (tear-free capture, even dims) =====================
targetH = []; targetW = [];
for kf = 1:nFrames
    tNow = (kf-1)/fpsVid;
    i    = max(1, min(length(tPlot), round(tNow*fsPlot)+1));

    % move tracers
    set(h1,'XData',tPlot(i),'YData',f1(i));
    set(h2,'XData',tPlot(i),'YData',f2(i));
    set(h3,'XData',tPlot(i),'YData',f3(i));
    set(hp,'XData',f1(i),'YData',f2(i),'ZData',f3(i));
    set(lineCentroid,'XData',[meanHz f1(i)],'YData',[meanHz f2(i)],'ZData',[meanHz f3(i)]);
    drawnow;

    % robust off-screen raster
    frameRGB = print(fig,'-RGBImage','-r150');   % uint8 MxNx3

    % enforce constant, even dimensions for H.264
    [h,w,~] = size(frameRGB);
    if isempty(targetH)
        targetH = h - mod(h,2);
        targetW = w - mod(w,2);
    end
    frameRGB = frameRGB(1:targetH, 1:targetW, :);

    % audio slice for this frame
    a0 = (kf-1)*spf + 1; a1 = min(kf*spf, size(audio,1));
    aChunk = audio(a0:a1,:);

    if useCVT
        step(vw, frameRGB, aChunk);
    else
        writeVideo(vw, frameRGB);
    end
end

if useCVT
    release(vw);
else
    close(vw);
    audiowrite(wavFile, audio, fsAud);
    % auto-mux using ffmpeg if present
    [ok,~] = system('ffmpeg -version');
    if ok==0
        cmd = sprintf('ffmpeg -y -loglevel error -i "%s" -i "%s" -c:v copy -c:a aac -shortest "%s"', ...
                      vidSilent, wavFile, vidFinal);
        system(cmd);
        fprintf('Wrote %s (muxed with audio).\n', vidFinal);
    else
        fprintf('Wrote %s (silent) and %s (audio). Install ffmpeg to mux.\n', vidSilent, wavFile);
    end
end

% ===================== Sanity check (prints expected rate range) =====================
w = round(0.050 * fsAud);  % 50 ms window
rate_est = movsum(spikes, w, 1) * (fsAud / w);  % Hz
fprintf('N1 estimated instantaneous rate ~ %.1f–%.1f Hz\n', min(rate_est(:,1)), max(rate_est(:,1)));
fprintf('N2 estimated instantaneous rate ~ %.1f–%.1f Hz\n', min(rate_est(:,2)), max(rate_est(:,2)));
fprintf('N3 estimated instantaneous rate ~ %.1f–%.1f Hz\n', min(rate_est(:,3)), max(rate_est(:,3)));
