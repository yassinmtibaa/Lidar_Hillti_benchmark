# Lidar Hilti Benchmark

This repository provides a simple benchmarking harness for LiDAR odometry algorithms on Hilti datasets. It ships with a lightweight LOAM-style baseline and a minimal LeGO-LOAM style variant.

## Algorithms

- `algorithms/loam/` — baseline LOAM-like registration using planar residuals
- `algorithms/legoloam/` — minimal LeGO-LOAM style pipeline with ground segmentation

## Usage

Run an algorithm on a ROS bag:

```matlab
% LOAM
traj = algorithms.loam.pipeline('data/site3_handheld_1.bag', '/hesai/pandar');

% LeGO-LOAM
traj = algorithms.legoloam.pipeline('data/site3_handheld_1.bag', '/hesai/pandar');
```

Results are saved as `.tum` in `results/`.

---
# LOAM Code Checklist

## 1 · Folder & Naming

- **algorithms/loam/**
  - `pipeline.m` ← one public entry point
  - `+internal/` (package folder)
    - `edgeSurfFeatures.m`
    - `registerPair.m`
    - `buildTrajRow.m`
- **utils/**
  - `readHiltiBag.m`
  - `accumulatePose.m`
  - `saveTraj.m`

**Rule:** Put anything *only* used by LOAM inside `+internal` to keep the global namespace clean.

## 2 · File Header Template

Paste this at the top of every new `.m` file:

```matlab
%--------------------------------------------------------------------
% File        : algorithms/loam/pipeline.m
% Purpose     : End-to-end LOAM odometry on a ROS bag
% Author      : P2 <your-email>
% Created     : 2025-07-17
% Dependencies: utils/readHiltiBag, lidarToolbox R2024b
%--------------------------------------------------------------------
```

## 3 · Function Skeleton

Use this pattern for new functions:

```matlab
function traj = pipeline(bagFile, topic, cfg)
arguments
    bagFile (1,:) char
    topic   (1,:) char = "/hesai/pandar"
    cfg.GridStep   double = 0.4   % down-sample size
    cfg.InitGuess  rigidtform3d = rigidtform3d
end
%#codegen  <-- remove if not compiling

scans = readHiltiBag(bagFile, topic);
...
```
- Use `arguments` for defaults.
- Add `#codegen` if you plan to compile.
- Never leave “magic numbers” inline—surface them as `cfg`.

## 4 · Commit Message Template

```
feat(loam): implement registerPair via pcregisterloam

* algorithms/loam/+internal/registerPair.m
  - wraps pcregisterloam with custom initial guess
* pipeline.m
  - plug new helper, bump Hz 5→8
```

## 5 · Pull-request Checklist

- [ ] `demo_run_loam_site3.m` executes end-to-end without edits.
- [ ] Added/updated unit tests in `tests/loamTest.m`.
- [ ] Updated docstring & Examples section.
- [ ] Ran lint → no errors.

## 6 · Docstring Pattern

```matlab
%% registerPair
%  Align two organised pointCloud scans using edge-&-plane cost.
%
%  T = internal.registerPair(srcCloud, tgtCloud, initT, gridStep)
%
%  Inputs
%  ------
%  srcCloud  pointCloud   » current frame
%  tgtCloud  pointCloud   » previous frame
%  initT     rigidtform3d » initial estimate
%  gridStep  double       » down-sample size [m]
%
%  Output
%  ------
%  T         rigidtform3d  optimal SE(3) transform
%
%  Example
%  -------
%     T = internal.registerPair(pc2, pc1, rigidtform3d, 0.4);
%
```

## 7 · Unit-test Snippet

```matlab
classdef loamTest < matlab.unittest.TestCase
   methods (Test)
      function testRegisterPairIdentity(tc)
         pc = pointCloud(rand(1000,3));
         T  = internal.registerPair(pc, pc, rigidtform3d, 0.4);
         tc.verifyLessThan(norm(T.Translation), 1e-3);
      end
   end
end
```
Run with:
```sh
matlab -batch "runtests('tests')"
```

## 8 · Timing Hook

Add at the top of `pipeline.m`:

```matlab
tic
...
fprintf("LOAM Hz = %.1f\n", numel(scans)/toc);
```

## 9 · Result Artifact Rule

| File type | Folder             |
| --------- | ------------------ |
| *.tum     | results/           |
| *.json    | results/metrics/   |
| Figures   | fig/loam/          |

**Never commit .bag files—add them to .gitignore.**

