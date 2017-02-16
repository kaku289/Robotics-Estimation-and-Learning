% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
% the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

% figure(1);
% imagesc(myMap); hold on;


N = length(pose);
M = length(scanAngles); % No. of rays
robotGrid = [];

for j = 1:N % for each time,

    
    robot_grid = ceil([pose(1,j);pose(2,j)]*myResol)+ myorigin;
    
%     free = sub2ind(size(myMap),robot_grid(2),robot_grid(1));
%     myMap(free) = 100;
    % Find grids hit by the rays (in the grid map coordinate)
    xocc = ranges(:,j).*cos(pose(3,j)+scanAngles) + pose(1,j);
    yocc = -ranges(:,j).*sin(pose(3,j)+scanAngles) + pose(2,j);
    
    
%     for ct=1:length(scanAngles) % for each scanAngle
%         dummy = [ranges(ct,j)*cos(pose(3,j)+scanAngles(ct)); -ranges(ct,j)*sin(pose(3,j)+scanAngles(ct))] + [pose(1,j);pose(2,j)];
%         occ(:,ct) = ceil(dummy/myResol) + myorigin;
%         
%         % get free cells in between
%         [freex, freey] = bresenham(robot_grid(1),robot_grid(2),occ(1,ct),occ(2,ct));  
%         % convert to 1d index
%         free = sub2ind(size(myMap),freey,freex);
%         % set end point value 
%         myMap(occ(2,ct),occ(1,ct)) = myMap(occ(2,ct),occ(1,ct))+lo_occ;
%         % set free cell values
%         myMap(free) = myMap(free)-lo_free;
%         
%         % Saturate the log-odd values
%         myMap(myMap>lo_max) = lo_max;
%         myMap(myMap<lo_min) = lo_min;
%     end
  

    % Find occupied-measurement cells and free-measurement cells
    occ = ceil([xocc';yocc']*myResol) + myorigin*ones(1,M);
    free = [];
    for ct=1:M
        % get cells in between
        [freex, freey] = bresenham(robot_grid(1),robot_grid(2),occ(1,ct),occ(2,ct));  
        % convert to 1d index
        free = [free;sub2ind(size(myMap),freey,freex)];
        
        myMap(occ(2,ct),occ(1,ct)) = myMap(occ(2,ct),occ(1,ct))+lo_occ;
    end

    % Update the log-odds
    myMap(free) = myMap(free)-lo_free;
  

    % Saturate the log-odd values
    myMap(myMap>lo_max) = lo_max;
    myMap(myMap<lo_min) = lo_min;
    

    % Visualize the map as needed
    robotGrid = [robotGrid robot_grid];
    imagesc(myMap); hold on;
    plot(robotGrid(1,:),robotGrid(2,:),'r.-','LineWidth',2.5); % indicate start point
    axis equal;
    lidar_global(:,1) =  (ranges(:,j).*cos(scanAngles + pose(3,j)) + pose(1,j))*myResol + myorigin(1);
    lidar_global(:,2) = (-ranges(:,j).*sin(scanAngles + pose(3,j)) + pose(2,j))*myResol + myorigin(2);
    plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 
    hold off;
    drawnow;

end

end

