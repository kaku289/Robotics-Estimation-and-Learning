% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% Used for making video
t = param.t; % Time vector
pose = param.pose; % Actual pose

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 200;                        % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.                               
Q = diag([0.0015,0.0015,0.0005]); % Segment 1                              
% Q = diag([0.025 0.025 0.03]);                               
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
% P = mvnrnd(myPose(:,1),[0.01,0.01,0.005],M)';
P = repmat(myPose(:,1), [1, M]);

w = ones(1,M)/M; % weights for each particle
correlation = zeros(1,M);

figure(10);
imagesc(map); hold on;
colormap('gray');
axis equal;
hold on;
particlesPlot = scatter(P(1,:)*myResol+myOrigin(1),P(2,:)*myResol+myOrigin(2), 6, 'MarkerEdgeColor',[0 .5 .5],...
              'MarkerFaceColor',[0 .7 .7],...
              'LineWidth',1.5);
          
figure(100);        
imagesc(map); hold on;
colormap('gray');
axis equal;
hold on;
lidar_global(:,1) =  (ranges(:,1).*cos(scanAngles + myPose(3,1)) + myPose(1,1))*myResol + myOrigin(1);
lidar_global(:,2) = (-ranges(:,1).*sin(scanAngles + myPose(3,1)) + myPose(2,1))*myResol + myOrigin(2);
lidarPlot = plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 
posPlot = plot(myPose(1,1)*param.resol+param.origin(1), ...
    myPose(2,1)*param.resol+param.origin(2), 'r.-');


% Pose compared (used for making video)
figure(1000);
subplot(3,1,1); grid;
hold on; xActual = plot(t(1), pose(1,1),'k', 'LineWidth', 2);
xCalc = plot(t(1), myPose(1,1),'r', 'LineWidth', 1);
ylabel('$x~$', 'FontSize', 26, 'Interpreter', 'latex');
title('Pose comparison in local co-ordinate frame', 'FontSize', 26, 'Interpreter', 'latex');

h1 = legend('Actual pose', 'Estimated pose' );
set(h1,'FontSize',18);

subplot(3,1,2); grid;
hold on; yActual = plot(t(1), pose(2,1),'k', 'LineWidth', 2);
yCalc = plot(t(1), myPose(2,1),'r', 'LineWidth', 2);
ylabel('$y~$', 'FontSize', 26, 'Interpreter', 'latex');

subplot(3,1,3); grid;
hold on; thetaActual = plot(t(1), pose(3,1),'k', 'LineWidth', 2);
thetaCalc = plot(t(1), myPose(3,1),'r', 'LineWidth', 2);
ylabel('$\theta~$', 'FontSize', 26, 'Interpreter', 'latex');
xlabel('$time~(s)$', 'FontSize', 20, 'Interpreter', 'latex'); 
set(findobj('type','axes'),'fontsize',18);

pause;

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    
    % 1) Propagate the particles 
    P = diag(myPose(:,j-1))*ones(3,M) +  mvnrnd([0;0;0],Q,M)';
    
    % 2) Measurement Update 
    for p = 1:M
        % closest 80% ranges
        nRanges = ceil(1.0*size(ranges,1));
%         [~, idx] = sort(ranges(:,j));
        idx = 1:nRanges;
%         robot_grid = ceil([P(1,p);P(2,p)]*myResol)+ myOrigin;
        
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate
    %   frame)
        xocc = ranges(idx,j).*cos(P(3,p)+scanAngles(idx)) + P(1,p);
        yocc = -ranges(idx,j).*sin(P(3,p)+scanAngles(idx)) + P(2,p);
        occ = ceil([xocc';yocc']*myResol) + myOrigin*ones(1,nRanges);
        del_occ =  occ(1,:)<1 | occ(2,:)<1 |  occ(1,:) > size(map,2) |  occ(2,:) > size(map,1);

        occ(:,del_occ) = [];
        
    %   2-2) For each particle, calculate the correlation scores of the particles

%         num = [occ(1,:)>size(map,2); occ(2,:)>size(map,1)];
%         sumnum = logical(sum(num,1));
%         occ(:,sumnum) = [];
        occ_index = sub2ind(size(map),occ(2,:),occ(1,:));
%         occ_index = unique(occ_index);
        
% %         free = [];
% %         for ct1=1:length(occ_index)
% %             % get cells in between
% %             [freex, freey] = bresenham(robot_grid(1),robot_grid(2),occ(1,(ct1)),occ(2,(ct1)));  
% %             % convert to 1d index
% %             if ~isempty(freex)
% %                 free = [free;sub2ind(size(map),freey,freex)];
% %             end
% %         end
% %         free = unique(free);
        occ_values = map(occ_index);
% %         free_values = map(free);
        correlation(1,p) = sum(occ_values(occ_values>=0.5)*10) + sum(occ_values(occ_values<=-0.2)*2);% + sum(free_values(free_values<0)*-3) + sum(free_values(free_values>0)*-5);% - sum(sumnum)*0.05;
    end
    %   2-3) Update the particle weights  
    correlation(correlation<0)= 0;
    w = correlation;
    w = w./sum(w);
    if sum(w<0)>0
        pause
    end
 
    %   2-4) Choose the best particle to update the pose
    [~,ind]=max(w);
    myPose(:,j) = P(:,ind);
    
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    Neff = 1/sum(w.*w);
    if Neff < 0.1*M
      edges = min([0 cumsum(w)],1); % protect against accumulated round-off
      edges(end) = 1;                 % get the upper edge exact
      % edges correspond to the c variable in Algorthm 2 in reference 1
      u1 = rand/M;
      % this works like the inverse of the empirical distribution and returns
      % the interval where the sample is to be found
      [~, idx] = histc(u1:1/M:1,edges);
      P = P(:,idx);                     % extract new particles
      w = w(:,idx);
      w = w./sum(w);
%       w = repmat(1/M, 1, M);          % now all particles have the same weight
    end
%     myPose(:,j)=sum(repmat(w,3,1).*P,2);
    
    % 4) Visualize the pose on the map as needed
    particlesPlot.XData = P(1,:)*myResol+myOrigin(1);
    particlesPlot.YData = P(2,:)*myResol+myOrigin(2);
    
    lidarPlot.XData = (ranges(:,j).*cos(scanAngles + myPose(3,j)) + myPose(1,j))*myResol + myOrigin(1);
    lidarPlot.YData = (-ranges(:,j).*sin(scanAngles + myPose(3,j)) + myPose(2,j))*myResol + myOrigin(2);
    
    dummyx = myPose(1,j)*param.resol+param.origin(1);
    dummyy = myPose(2,j)*param.resol+param.origin(2);
    posPlot.XData = [posPlot.XData dummyx];
    posPlot.YData = [posPlot.YData dummyy];
    
    figure(10);
    xlim([dummyx-40 dummyx+40])
    ylim([dummyy-40 dummyy+40])
    
    % For video
    xActual.XData = t(1:j)'; xActual.YData = [xActual.YData pose(1,j)];
    yActual.XData = t(1:j)'; yActual.YData = [yActual.YData pose(2,j)];
    thetaActual.XData = t(1:j)'; thetaActual.YData = [thetaActual.YData pose(3,j)];
    xCalc.XData = t(1:j)'; xCalc.YData = [xCalc.YData myPose(1,j)];
    yCalc.XData = t(1:j)'; yCalc.YData = [yCalc.YData myPose(2,j)];
    thetaCalc.XData = t(1:j)'; thetaCalc.YData = [thetaCalc.YData myPose(3,j)];
    
    drawnow;


end

end

