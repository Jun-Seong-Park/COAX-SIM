function Disturbance = DisGen(Dis_max, t, mode)
% DisGen  Generate external disturbance forces and moments
%   No external parameters required.

    Force_Dis_max  = Dis_max(1);
    Moment_Dis_max = Dis_max(2);

    if mode == 0
        Force_Disturbance  = zeros(3,1);
        Moment_Disturbance = zeros(3,1);
    elseif mode == 1
        Force_Disturbance  = Force_Dis_max  * ones(3,1);
        Moment_Disturbance = Moment_Dis_max * ones(3,1);
    elseif mode == 2
        Force_Disturbance  = Force_Dis_max  * sin(t) * ones(3,1);
        Moment_Disturbance = Moment_Dis_max * sin(t) * ones(3,1);
    elseif mode == 3
        Force_Disturbance  = Force_Dis_max  * sin(t*0.1) * ones(3,1);
        Moment_Disturbance = Moment_Dis_max * sin(t*0.1) * ones(3,1);
    elseif mode == 4
        Force_Disturbance  = Force_Dis_max  * sin(t*10) * ones(3,1);
        Moment_Disturbance = Moment_Dis_max * sin(t*10) * ones(3,1);
    end

    Disturbance = [Force_Disturbance; Moment_Disturbance];
end
