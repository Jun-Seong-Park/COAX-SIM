function X_d = scenario_call(t, id)
% scenario_call  Dispatch to ScenarioN(t) by numeric ID

    X_d = feval(sprintf('Scenario%d', id), t);
end
