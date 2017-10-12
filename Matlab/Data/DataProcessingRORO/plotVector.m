function  plotVector( v )
%% plots vector

    hold on
    o = [0 0 0 ]';
    p1 = o;                         % First Point
    p2 = v;                         % Second Point
    dp = p2-p1;                         % Difference
    figure(1)
    quiver3(p1(1),p1(2),p1(3),dp(1),dp(2),dp(3),0, 'MaxHeadSize',1);
    hold off
end

