function U = MPCctrl(state, state_1dot, Desired, mpcobj, x_mpc)
    warning('off', 'all'); % 모든 경고 끄기
    x = [state; state_1dot];
    r = [Desired(1,1); Desired(2,1); Desired(3,1); Desired(1,2); Desired(2,2); Desired(3,2)];
    mpcverbosity('off');
    U = mpcmove(mpcobj, x_mpc, x, r);
    warning('on', 'all'); % 필요 시 경고 다시 켜기
end