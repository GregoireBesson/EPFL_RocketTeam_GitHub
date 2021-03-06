function plotRocketBody(roro)
    
    % function which plots the fixed element of the rocket
    
    % define
    X = 1;
    Y = 2;
    
%     Length = 1.363;
%     Cone_L = 0.425;
%     Lbody = Length-Cone_L;
%     D = 0.102;
%     R = D/2;
%     Fin_height =	127e-3;
%     Fin_base_length = 152.40e-3;
%     Fin_top_length = 44.45e-3;
%     Fin_Sweep_angle = 29.794*2*pi/360;
%     Fin_offset = 20e-3;
%     Fin_thickness = 3e-3*2;
    
    Length = roro.Length;
    Cone_L = roro.Cone_L;
    Lbody = Length-Cone_L;
    D = roro.D;
    R = D/2;
    fin_h = roro.fin_h;
    fin_base = roro.fin_base;
    fin_top = roro.fin_top;
    fin_sweep = roro.fin_sweep;
    fin_offs = 50e-3;
    fin_t = roro.fin_t;
    
    % factor induced by perspective
    f = 0.75;
    
    % definition of elements
    nosecone = [-R 0 R; Lbody Length Lbody];
    body = [-R R R -R; Lbody Lbody 0 0];
    finRight = [R R (R+fin_h)*f (R+fin_h)*f; ...
                fin_offs fin_offs+fin_base ...
                fin_offs+fin_base-fin_h*sin(fin_sweep)...
                fin_offs+fin_base-fin_h*sin(fin_sweep)-fin_top];
    finLeft = [-R -R -(R+fin_h)*f -(R+fin_h)*f; ...
                fin_offs fin_offs+fin_base ...
                fin_offs+fin_base-fin_h*sin(fin_sweep)...
                fin_offs+fin_base-fin_h*sin(fin_sweep)-fin_top];
    finMiddle = [-fin_t -fin_t fin_t fin_t ; ...
                 fin_offs fin_offs+fin_base fin_offs+fin_base fin_offs];
    
    figure('Name','Rocket State','Position',[0 1000 300 1000])
    hold on
    fill(nosecone(X,:),nosecone(Y,:),'r')
    fill(body(X,:),body(Y,:),'k')
    fill(finRight(X,:),finRight(Y,:),'r')
    fill(finLeft(X,:),finLeft(Y,:),'r')
    fill(finMiddle(X,:),finMiddle(Y,:),'r')
    %xlim([-1.5*L 1.5*L]);
    %ylim([-0.5*L 1.5*L]);
    axis([-0.25*Length 0.25*Length -0.25*Length 1.25*Length])
    daspect([1 1 1])

end