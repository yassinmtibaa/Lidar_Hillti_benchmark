%% quick smoke test (≤ 30 s)
bag = "data/site3_handheld_1.bag";
topic = "/hesai/pandar";
sc   = readHiltiBag(bag,topic);
pcshow(sc{5}); title("Frame 20 organised"); view(2)
fprintf("Organised frames: %d\n",numel(sc));

%% Merge all point clouds and visualize the whole scene
allPoints = [];
for k = 1:numel(sc)
    pts = sc{k}.Location;
    pts = reshape(pts, [], 3); % flatten if organized
    allPoints = [allPoints; pts];
end
allPoints = allPoints(~any(isnan(allPoints),2),:); % remove NaNs
mergedCloud = pointCloud(allPoints);
pcshow(mergedCloud);
title('Merged Point Cloud');

%% --- Smoother Optimized Visualization: More Frames, Finer Downsampling, Wider ROI ---
step = 10;
subset = sc(1:step:end);
downsampled = cell(size(subset));
parfor k = 1:numel(subset)
    downsampled{k} = pcdownsample(subset{k}, 'gridAverage', 0.5); % 0.1m grid
end
roi = [-30 30; -30 30; -3 8];
filtered = cell(size(downsampled));
parfor k = 1:numel(downsampled)
    indices = findPointsInROI(downsampled{k}, roi);
    filtered{k} = select(downsampled{k}, indices);
end
optPoints = [];
for k = 1:numel(filtered)
    pts = filtered{k}.Location;
    pts = reshape(pts, [], 3);
    optPoints = [optPoints; pts];
end
optPoints = optPoints(~any(isnan(optPoints),2),:);
optCloud = pointCloud(optPoints);
pcshow(optCloud, 'MarkerSize', 50);
title('Smoother Merged Point Cloud (More Frames, Finer Grid, Wider ROI)');

%% 1. Color by frame
figure; hold on;
cmap = jet(numel(filtered));
for k = 1:numel(filtered)
    pts = filtered{k}.Location;
    pts = reshape(pts, [], 3);
    color = repmat(cmap(k,:), size(pts,1), 1); % N×3 array for all points in this frame
    pcshow(pts, color, 'MarkerSize', 50);
end
title('Merged Point Cloud Colored by Frame');
hold off;

%% 2. Color by intensity (if available)
if isprop(optCloud, 'Intensity') && ~isempty(optCloud.Intensity)
    figure;
    pcshow(optCloud.Location, optCloud.Intensity, 'MarkerSize', 50);
    colormap('hot'); colorbar;
    title('Merged Point Cloud (Colored by Intensity)');
end

%% 3. Interactive ROI selection
figure;
pcshow(optCloud, 'MarkerSize', 50);
title('Draw ROI (rectangle) to filter points');
h = drawrectangle;
wait(h);
roiRect = h.Position;
roiBox = [roiRect(1), roiRect(1)+roiRect(3); roiRect(2), roiRect(2)+roiRect(4); -3 8];
indices = findPointsInROI(optCloud, roiBox);
roiCloud = select(optCloud, indices);
figure;
pcshow(roiCloud, 'MarkerSize', 50);
title('Points in Selected ROI');

%% 4. Animate the point cloud (frame by frame)
figure;
for k = 1:numel(sc)
    pcshow(sc{k});
    title(sprintf('Frame %d', k));
    drawnow;
    pause(0.05);
end

%% 5. Export merged cloud to PLY
pcwrite(optCloud, 'mergedCloud.ply');

%% 6. Save a screenshot
saveas(gcf, 'cloud.png');

%% 7. Remove outliers
denCloud = pcdenoise(optCloud);
figure;
pcshow(denCloud, 'MarkerSize', 50);
title('Denoised (Outlier-Removed) Cloud');

%% 8. Plane segmentation (fit largest plane)
maxDistance = 0.2;
[model, inlierIdx] = pcfitplane(optCloud, maxDistance);
planeCloud = select(optCloud, inlierIdx);
figure;
pcshow(planeCloud, 'MarkerSize', 50);
title('Largest Plane in Merged Cloud');

%% 9. Random downsampling (fixed number of points)
numSample = min(100000, optCloud.Count);
randIdx = randperm(optCloud.Count, numSample);
randCloud = select(optCloud, randIdx);
figure;
pcshow(randCloud, 'MarkerSize', 50);
title('Randomly Downsampled Cloud');

%% 10. Interactive pcplayer
player = pcplayer([-30 30], [-30 30], [-3 8]);
view(player, optCloud);

%% 11. User controls (sliders for step/grid/ROI)
% (For a full GUI, use uicontrols or App Designer. Here is a simple example:)
step = input('Enter step (frame skip, e.g. 10): ');
gridStep = input('Enter grid step (e.g. 0.1): ');
roiX = input('Enter ROI X range as [xmin xmax]: ');
roiY = input('Enter ROI Y range as [ymin ymax]: ');
roiZ = input('Enter ROI Z range as [zmin zmax]: ');
subset = sc(1:step:end);
downsampled = cell(size(subset));
for k = 1:numel(subset)
    downsampled{k} = pcdownsample(subset{k}, 'gridAverage', gridStep);
end
roi = [roiX(:)'; roiY(:)'; roiZ(:)'];
filtered = cell(size(downsampled));
for k = 1:numel(downsampled)
    indices = findPointsInROI(downsampled{k}, roi);
    filtered{k} = select(downsampled{k}, indices);
end
optPoints = [];
for k = 1:numel(filtered)
    pts = filtered{k}.Location;
    pts = reshape(pts, [], 3);
    optPoints = [optPoints; pts];
end
optPoints = optPoints(~any(isnan(optPoints),2),:);
optCloud = pointCloud(optPoints);
figure;
pcshow(optCloud, 'MarkerSize', 50);
title('User-Controlled Merged Point Cloud'); 