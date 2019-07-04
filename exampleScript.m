clc
clear
close all;

%% Import data

sessionData = importSession('FootTrackingSession');

samplePeriod = 1 / 400; % 400 Hz
[sessionData, time] = resampleSession(sessionData, samplePeriod); % resample data so that all measuremnts share the same time vector

quaternion = sessionData.(sessionData.deviceNames{1}).quaternion.vector;
acceleration = sessionData.(sessionData.deviceNames{1}).earth.vector * 9.81; % convert to m/s/s

numberOfSamples = length(time);

%% Identify stationary periods

threshold = 1; % acceleration threshold in m/s/s

% Determine as moving if acceleration greater than theshold
isMoving = abs(acceleration(:,1)) > threshold | ...
           abs(acceleration(:,2)) > threshold | ...
           abs(acceleration(:,3)) > threshold;

% Add margin to extend each period identified as moving
marginSizeInSamples = ceil(0.1 / samplePeriod); % margin = 0.1 seconds
isMovingWithMargin = isMoving;
for sampleIndex = 1 : (numberOfSamples - marginSizeInSamples)
    if(isMoving(sampleIndex) == 1)
        isMovingWithMargin(sampleIndex : (sampleIndex + marginSizeInSamples)) = 1;
    end
end
for sampleIndex = (numberOfSamples - marginSizeInSamples) : -1 : 1
    if(isMoving(sampleIndex) == 1)
        isMovingWithMargin((sampleIndex - marginSizeInSamples) : sampleIndex) = 1;
    end
end

% Stationary periods are non-moving periods
isStationary = ~isMovingWithMargin;

%% Calculate velocity

velocity = zeros(size(acceleration));
for sampleIndex = 2 : numberOfSamples
    velocity(sampleIndex, :) = velocity(sampleIndex - 1, :) + acceleration(sampleIndex, :) * samplePeriod;
    if(isStationary(sampleIndex) == 1)
        velocity(sampleIndex, :) = [0 0 0]; % force velocity to zero if stationary
    end
end

%% Remove velocity drift

stationaryStartIndexes = find([0; diff(isStationary)] == -1);
stationaryEndIndexes = find([0; diff(isStationary)] == 1);

velocityDrift = zeros(size(velocity));
for stationaryEndIndexesIndex = 1:numel(stationaryEndIndexes)

    velocityDriftAtEndOfMovement = velocity(stationaryEndIndexes(stationaryEndIndexesIndex) - 1, :);
    numberOfSamplesDuringMovement = (stationaryEndIndexes(stationaryEndIndexesIndex) - stationaryStartIndexes(stationaryEndIndexesIndex));
    velocityDriftPerSample = velocityDriftAtEndOfMovement / numberOfSamplesDuringMovement;

    ramp = (0 : (numberOfSamplesDuringMovement - 1))';
    velocityDriftDuringMovement = [ramp * velocityDriftPerSample(1), ...
                                   ramp * velocityDriftPerSample(2), ...
                                   ramp * velocityDriftPerSample(3)];

    velocityIndexes = stationaryStartIndexes(stationaryEndIndexesIndex):stationaryEndIndexes(stationaryEndIndexesIndex) - 1;
    velocity(velocityIndexes, :) = velocity(velocityIndexes, :) - velocityDriftDuringMovement;
end

%% Calculate position

position = zeros(size(velocity));
for sampleIndex = 2 : numberOfSamples
    position(sampleIndex, :) = position(sampleIndex - 1, :) + velocity(sampleIndex, :) * samplePeriod;
end

%% Plot data

figure;

subplots(1) = subplot(3, 1, 1);
hold on;
plot(time, acceleration(:, 1), 'r');
plot(time, acceleration(:, 2), 'g');
plot(time, acceleration(:, 3), 'b');
plot(time, isStationary * 10, 'k');
title('Acceleration');
xlabel('seconds)');
ylabel('m/s/s');
legend('x', 'y', 'z', 'is stationary');

subplots(2) = subplot(3, 1, 2);
hold on;
plot(time, velocity(:, 1), 'r');
plot(time, velocity(:, 2), 'g');
plot(time, velocity(:, 3), 'b');
title('Velocity');
xlabel('seconds)');
ylabel('m/s');
legend('x', 'y', 'z');

subplots(3) = subplot(3, 1, 3);
hold on;
plot(time, position(:, 1), 'r');
plot(time, position(:, 2), 'g');
plot(time, position(:, 3), 'b');
title('Position');
xlabel('seconds)');
ylabel('m');
legend('x', 'y', 'z');

linkaxes(subplots, 'x');

%% Create animation

SixDofAnimation(position, quatern2rotMat(quaternion), ...
                'SamplePlotFreq', 20, 'Trail', 'All', ...
                'Position', [9 39 1280 768], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false);
