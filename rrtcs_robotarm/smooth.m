function [path_smooth] = smooth(map, path, vertices, delta)
path_smooth = path(1); 
currentIndex = 1; 
currentSmoothIndex = 2; 
while (1)
    if isSpacePNearPNewFree(map, vertices(path(currentSmoothIndex), :), vertices(path(currentIndex), :), delta)
        currentSmoothIndex = currentSmoothIndex + 1;
    else
        currentSmoothIndex = currentSmoothIndex - 1;
        path_smooth = [path_smooth, path(currentSmoothIndex)];
        currentIndex = currentSmoothIndex;
        currentSmoothIndex = currentSmoothIndex + 1;
    end
    if currentSmoothIndex>numel(path)
        path_smooth = [path_smooth, path(numel(path))];
        break
    end
end
 rrtSmoothDraw(map, path_smooth, vertices);
end

function [isBelongsFreeSpace] = isSpacePNearPNewFree(map, startPoint, endPoint, delta)
    
    v = double(endPoint - startPoint);
    
    distance = norm(v);
    
    u = v / distance;
    
    intermediatePointCount = distance / delta;
    
    currentCoordinate = double(startPoint);
    
    for ii = 1 : intermediatePointCount
        
        currentCoordinate = currentCoordinate + (delta * u);
        
        if map(int32(currentCoordinate(2)), int32(currentCoordinate(1))) == 1
            isBelongsFreeSpace = 0;
            return;
        end
        
    end
    
    isBelongsFreeSpace = 1;

end

function rrtSmoothDraw(map, path_smooth, vertices)

    imshow(int32(1 - map), []);
    title('RRT (Rapidly-Exploring Random Trees) - Smooth Path');
    % imagesc(1 - map);
    % colormap(gray);
    
    hold on;

    [~, pathCount] = size(path_smooth);
    
    for ii = 1 : pathCount - 1
        %plot(vertices(ii, 1), vertices(ii, 2), 'cyan*', 'linewidth', 1);
        plot([vertices(path_smooth(ii), 1), vertices(path_smooth(ii + 1), 1)], ...
        [vertices(path_smooth(ii), 2), vertices(path_smooth(ii + 1), 2)], ...
         'r', 'LineWidth', 2);
    end
    
end