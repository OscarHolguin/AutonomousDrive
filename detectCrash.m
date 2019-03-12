function isCrash = detectCrash(obstacles,origin,goal)
    %DETECTCRASH Summary of this function goes here
    %   Detailed explanation goes here
    
    isCrash = false;
    
    % Extract data
    x0 = origin(1,1);
    y0 = origin(1,2);
    xf = goal(1,1);
    yf = goal(1,2);
    
    % Check if new coordinate is inside a block
    if any(obstacles(:,1) < xf & xf < obstacles(:,3) & obstacles(:,2) < yf & yf < obstacles(:,4))
        isCrash = true;
        return
    end
    
    % Basic comparisons
    leftSideX_0 = x0 <= obstacles(:,1);
    leftSideX_f = xf <= obstacles(:,1);
    rightSideX_0 = x0 >= obstacles(:,3);
    rightSideX_f = xf >= obstacles(:,3);
    
    lowerSideY_0 = y0 <= obstacles(:,2);
    lowerSideY_f = yf <= obstacles(:,2);
    upperSideY_0 = y0 >= obstacles(:,4);
    upperSideY_f = yf >= obstacles(:,4);
    
    % Basic detection of no crashes
    if all(leftSideX_0) && all(leftSideX_f)
        return
    end
    
    if all(rightSideX_0) && all(rightSideX_f)
        return
    end
    
    if all(lowerSideY_0) && all(lowerSideY_f)
        return
    end
    
    if all(upperSideY_0) && all(upperSideY_f)
        return
    end
    
    % Get linear equation of the trayectory
    m = (xf - x0) / (yf - y0);
    b = y0 - m * x0;
    
    % Test the left side of the obstacle
    yTest = m * obstacles(:,1) + b;
    
    if any(obstacles(:,2) < yTest & yTest < obstacles(:,4))
        isCrash = true;
        return
    end
    
    % Test the right side of the obstacle
    yTest = m * obstacles(:,3) + b;
    
    if any(obstacles(:,2) < yTest & yTest < obstacles(:,4))
        isCrash = true;
        return
    end
    
    % Test the lower side of the obstacle
    xTest = (obstacles(:,2) - b) / m;
    if any(obstacles(:,1) < xTest & xTest < obstacles(:,3))
        isCrash = true;
        return
    end
    
    % No more tests required
end

