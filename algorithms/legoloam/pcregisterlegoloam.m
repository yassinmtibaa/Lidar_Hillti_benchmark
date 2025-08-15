function tform = pcregisterlegoloam(moving, fixed, varargin)
    % PCREGISTERLEGOLOAM  Minimal LeGO-LOAM style registration
    %   Performs ground segmentation, extracts coarse edge/plane features,
    %   and estimates SE(3) using a point-to-plane objective.

    parser = inputParser;
    parser.addParameter('InitialTransform', rigidtform3d);
    parser.addParameter('GridStep', 0.4);
    parser.parse(varargin{:});
    params = parser.Results;

    % Validate
    if isempty(moving) || isempty(fixed) || moving.Count < 10 || fixed.Count < 10
        tform = rigidtform3d; return;
    end

    % Downsample
    fixedDown  = pcdownsample(fixed, 'gridAverage', params.GridStep);
    movingDown = pcdownsample(moving,'gridAverage', params.GridStep);

    % Ground segmentation (simple RANSAC plane as a proxy)
    [fixedGround, fixedNonGround]   = localSegmentGround(fixedDown);
    [movingGround, movingNonGround] = localSegmentGround(movingDown);

    % Feature extraction proxies:
    %  - Planar features from ground (smooth)
    %  - Edge features from non-ground (sharp)
    fixedPlanar = fixedGround;  % proxy
    movingPlanar = movingGround;
    fixedEdge = fixedNonGround; % proxy
    movingEdge = movingNonGround;

    % Combine features (LeGO-LOAM mixes edge+plane; use planar here)
    fixedUse  = fixedPlanar;
    movingUse = movingPlanar;

    % Convert to arrays for residual computation
    fixedPts  = fixedUse.Location;
    movingPts = movingUse.Location;

    % Apply initial transform if provided
    if ~isempty(params.InitialTransform)
        initA = params.InitialTransform.A;
        movingPts = transformPointsForward(rigidtform3d(initA(1:3,1:3), initA(1:3,4)') , movingPts);
    end

    % Optimization (LM over point-to-plane, dummy normals as z-up fallback)
    theta = zeros(6,1);
    lambda = 0.001;
    maxIter = 10;

    % Precompute simple normals via PCA on fixed (fallback to z-up)
    if fixedUse.Count >= 10
        % Estimate normals using MATLAB function if available
        try
            normals = pcnormals(fixedUse, min(20, fixedUse.Count-1));
        catch
            normals = repmat([0 0 1], fixedUse.Count, 1);
        end
    else
        normals = repmat([0 0 1], fixedUse.Count, 1);
    end

    % Subsample to pair counts
    M = min(size(fixedPts,1), size(movingPts,1));
    if M < 10
        tform = rigidtform3d; return;
    end
    idx = randperm(M, min(500, M));
    pf = fixedPts(idx, :);
    pm = movingPts(idx, :);
    nn = normals(idx, :);

    for iter = 1:maxIter
        R = localR(theta(4), theta(5), theta(6));
        t = theta(1:3)';
        pmT = (pm*R' + t);

        residuals = sum((pmT - pf) .* nn, 2);
        % Build a simple numeric Jacobian around current theta
        epsv = 1e-4; J = zeros(numel(residuals),6);
        for j = 1:6
            d = zeros(6,1); d(j)=epsv;
            Rj = localR(theta(4)+d(4), theta(5)+d(5), theta(6)+d(6));
            tj = (theta(1:3)+d(1:3))';
            pmJ = (pm*Rj' + tj);
            rj  = sum((pmJ - pf) .* nn, 2);
            J(:,j) = (rj - residuals) / epsv;
        end

        H = J'*J; g = J'*residuals;
        Hlm = H + lambda*diag(diag(H));
        if rcond(Hlm) < 1e-12, break; end
        dtheta = - Hlm \ g;
        theta = theta + dtheta;
    end

    Rf = localR(theta(4), theta(5), theta(6));
    tf = theta(1:3)';
    tform = rigidtform3d(Rf, tf);
end

function [pcGround, pcNonGround] = localSegmentGround(pcIn)
    % Segment ground using a dominant plane model as a proxy for LeGO-LOAM ground
    if pcIn.Count < 50
        pcGround = pointCloud(zeros(0,3));
        pcNonGround = pcIn; return;
    end
    maxDistance = 0.2; % meters
    maxAngularDistance = 5; % degrees (not used directly here)
    try
        [model, inlierIdx] = pcfitplane(pcIn, maxDistance, [0 0 1], deg2rad(maxAngularDistance));
        pcGround    = select(pcIn, inlierIdx);
        pcNonGround = select(pcIn, setdiff(1:pcIn.Count, inlierIdx));
    catch
        % Fallback: z-threshold based segmentation
        z = pcIn.Location(:,3);
        thr = quantile(z, 0.2);
        inlierIdx = find(z < thr);
        pcGround    = select(pcIn, inlierIdx);
        pcNonGround = select(pcIn, setdiff(1:pcIn.Count, inlierIdx));
    end
end

function R = localR(rx, ry, rz)
    Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
    Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
    Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
    R = Rz * Ry * Rx;
end


