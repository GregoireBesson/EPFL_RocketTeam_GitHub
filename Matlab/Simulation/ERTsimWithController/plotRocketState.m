function plotRocketState(roro)

    plotRocketBody(roro)
    
    X = 1;
    Y = 2;
    D = roro.D;
    R = D/2;
    brakeLength = 20e-2;
    brakeWidth = 2e-2;
    brakeAngle = 1*2*pi/360;
    brakePos = 0.50;
    
    brakeRight = [R-brakeWidth R R R-brakeWidth; brakePos brakePos ...
                  brakePos-brakeLength brakePos-brakeLength] ;
    brakeLeft =  [ -brakeRight(X,:) ; brakeRight(Y,:)]  ;      
    
    % rotate the airbrakes
    RR = [cos(brakeAngle) -sin(brakeAngle); sin(brakeAngle) cos(brakeAngle)];
    RL = [cos(-brakeAngle) -sin(-brakeAngle); sin(-brakeAngle) cos(-brakeAngle)];
    CR = zeros(2,4);
    CR = CR+brakeRight(:,2);
    CL = zeros(2,4);
    CL = CL+brakeLeft(:,2);
    hBrakeRight = fill(brakeRight(X,:),brakeRight(Y,:),'r');
    hBrakeLeft = fill(brakeLeft(X,:),brakeLeft(Y,:),'r');
    
    pause(1)
    
    for k=1:90
    
    VR = get(hBrakeRight,'Vertices')';       % get the current set of vertices
    VR = RR*(VR - CR) + CR;                   % do the rotation relative to the top right edge of the brake
    VL = get(hBrakeLeft,'Vertices')';
    VL = RL*(VL - CL) + CL;
    set(hBrakeRight,'Vertices',VR');          % % update the vertices
    set(hBrakeLeft,'Vertices',VL');
    pause(0.03)
    
    end
    
    brakeAngle = -1*2*pi/360;
    RR = [cos(brakeAngle) -sin(brakeAngle); sin(brakeAngle) cos(brakeAngle)];
    RL = [cos(-brakeAngle) -sin(-brakeAngle); sin(-brakeAngle) cos(-brakeAngle)];
    
    for k=1:90
    
    VR = get(hBrakeRight,'Vertices')';       % get the current set of vertices
    VR = RR*(VR - CR) + CR;                   % do the rotation relative to the top right edge of the brake
    VL = get(hBrakeLeft,'Vertices')';
    VL = RL*(VL - CL) + CL;
    set(hBrakeRight,'Vertices',VR');          % % update the vertices
    set(hBrakeLeft,'Vertices',VL');
    pause(0.03)
    
    end
end