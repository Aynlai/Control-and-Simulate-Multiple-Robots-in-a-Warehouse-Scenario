function exampleHelperVisualizeWarehouseRobots(robotPoses, robotPacketStatuses, wayPoints, logicalMap, goalLocations, loadingStations, showPaths, xLim, yLim, robotMeshes)

%% Initilize working variables
map = binaryOccupancyMap(logicalMap);
numRobots = size(robotPoses, 2);
numLoad = size(loadingStations, 1);
numGoals = size(goalLocations, 1);

%% Extract data from inputs

% Construct translation from state
xyz = [robotPoses(1:2,:); zeros(1,numRobots)]; % 2-D so z is zero

% Create quaternion from rotation state
theta = robotPoses(3,:);

%% Visualize Results
show(map);
light
hold on;
ax = gca;
plotTransforms([goalLocations, zeros(numGoals, 1)], eul2quat(zeros(numGoals, 3)), 'MeshFilePath', 'exampleWarehouseBlock.stl', 'MeshColor', 'g', 'Parent', ax);
plotTransforms([loadingStations, zeros(numLoad, 1)], eul2quat(zeros(numLoad, 3)), 'MeshFilePath', 'exampleWarehouseBlock.stl', 'MeshColor', 'b', 'Parent', ax);
text(loadingStations(:, 1), loadingStations(:, 2), ones(numLoad, 1) * 2, 'Loading', 'FontSize', 12);
text(goalLocations(:, 1), goalLocations(:, 2), ones(numGoals, 1)* 2, 'Unloading', 'FontSize', 12);
    
for robotIdx = 1:numRobots
    
    % Extract the information for the ith robot
    robotQuaternion = eul2quat([0 0 theta(robotIdx)], 'xyz');
    worldToRobotTForm = [axang2rotm([0 0 1 theta(robotIdx)]) xyz(:,robotIdx); 0 0 0 1];
    robotPathWaypoints = wayPoints(:,:,robotIdx);
    robotHasPacket = robotPacketStatuses(robotIdx);
    
    % Get the robot mesh and offset between the mesh origin and robot origin
    if ~robotHasPacket
        robotMesh = robotMeshes{1};
        meshTranslation = robotMeshes{2};
    else
        robotMesh = robotMeshes{3};
        meshTranslation = robotMeshes{4};
    end
    robotTform = worldToRobotTForm*trvec2tform(meshTranslation);
    
    % Plot the robot
    plotTransforms(robotTform(1:3,4)', robotQuaternion, 'MeshFilePath', robotMesh, 'Parent', ax);
    
    % Plot the path
    if showPaths
        plot(robotPathWaypoints(:,1), robotPathWaypoints(:,2), 'x-');
    end
end

% Select the view
xlim(xLim');
ylim(yLim');
zlim([0 2]);
    
drawnow;
hold off;

end
