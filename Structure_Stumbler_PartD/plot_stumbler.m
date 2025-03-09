global model
% Plot stumbler outcome
figure(1)
set(1, 'units', 'normalized', 'position', [0.1 0.1 0.8 0.8])
h = axes;

% Lengths
Lstance     = model.Lstance;    % [m]
Lhip        = model.Lhip;       % [m]
Lthigh      = model.Lthigh;     % [m]
Lshank      = model.Lshank;     % [m]
Lfoot       = model.Lfoot;      % [m]

% Centres of gravity with respect to proximal joint
cgStance    = model.cgStance;   % [m]
cgThigh     = model.cgThigh;    % [m]
cgShank     = model.cgShank;    % [m]
cgFoot      = model.cgFoot;     % [m]

% Part B: Brick coordinates
bx1 = model.bx1;
bx2 = model.bx2;
by1 = model.by1;
by2 = model.by2;
bz1 = model.bz1;
bz2 = model.bz2;

for jj = [3, 1]
    for kk = 1:jj:length(X_out.time)
        % Get angles from the correct index of X_out.signals.values
        gamma1 = X_out.signals.values(kk,1);
        alpha2 = X_out.signals.values(kk,2);
        beta2 = X_out.signals.values(kk,3);
        gamma2 = X_out.signals.values(kk,4);
        gamma3 = X_out.signals.values(kk,5);
        gamma4 = X_out.signals.values(kk,6);
        
        symb_Ti;
        % Mass locations (Hint; use values from Ti)
        xmass = Ti([4, 10, 16, 22]);
        ymass = Ti([5, 11, 17, 23]);
        zmass = Ti([6, 12, 18, 24]);

        % Joint locations (Hint; use values from Ti)
        xjoint = [0; Ti([1, 7, 13, 19, 25])];
        yjoint = [0; Ti([2, 8, 14, 20, 26])];
        zjoint = [0; Ti([3, 9, 15, 21, 27])];

        % Front view (z vs y)
        subplot(223)
        plot([0; Ti(3:3:end)], [0; Ti(2:3:end)], 'b-', 'linewidth', 2); hold on
        plot(zjoint, yjoint, 'bo', 'markerfacecolor', 'w');
        plot(zmass, ymass, 'ks', 'markerfacecolor', 'k');
        plot([-1 1], [0 0], 'k--'); 
        % Block in front view
        patch([bz1 bz2 bz2 bz1], [by1 by1 by2 by2], 'r', 'FaceAlpha', 0.5)
        hold off
        axis([-1 1 -0.1 1])
        title('Front view')

        % Top view (z vs x)
        subplot(221)
        plot([0; Ti(3:3:end)], [0; Ti(1:3:end)], 'b-', 'linewidth', 2); hold on
        plot(zjoint, xjoint, 'bo', 'markerfacecolor', 'w');
        plot(zmass, xmass, 'ks', 'markerfacecolor', 'k');
        % Block in top view
        patch([bz1 bz2 bz2 bz1], [bx1 bx1 bx2 bx2], 'r', 'FaceAlpha', 0.5)
        hold off
        axis([-1 1 -1 1]);
        title('Top view')

        % Right view (x vs y)
        subplot(224)
        plot([0; Ti(1:3:end)],  [0; Ti(2:3:end)], 'b-', 'linewidth', 2); hold on
        plot(xjoint, yjoint, 'bo', 'markerfacecolor', 'w');
        plot(xmass, ymass, 'ks', 'markerfacecolor', 'k');
        plot([-1 1], [0 0], 'k--'); 
        % Block in right view
        patch([bx1 bx2 bx2 bx1], [by1 by1 by2 by2], 'r', 'FaceAlpha', 0.5)
        hold off
        axis([-1 1 -0.1 1])
        text(-0.9, 0.9, ['TIME: ', num2str(X_out.time(kk)), ' s.'])
        title('Right view')

        % 3D view
        subplot(2,2,2)
        plot3([0; Ti(3:3:end)], [0; Ti(1:3:end)], [0; Ti(2:3:end)], 'b-', 'linewidth', 2); hold on
        plot3(zjoint, xjoint, yjoint, 'bo', 'markerfacecolor', 'w');
        plot3(zmass, xmass, ymass, 'ks', 'markerfacecolor', 'k'); 

        % Draw the block in 3D
        Z_block = [bx1 bx2 bx2 bx1 bx1 bx2 bx2 bx1];
        Y_block = [by1 by1 by2 by2 by1 by1 by2 by2];
        X_block = [bz1 bz1 bz1 bz1 bz2 bz2 bz2 bz2];

        fill3(X_block([1 2 3 4]), Y_block([1 2 3 4]), Z_block([1 2 3 4]), 'r', 'FaceAlpha', 0.5) % Bottom face
        fill3(X_block([5 6 7 8]), Y_block([5 6 7 8]), Z_block([5 6 7 8]), 'r', 'FaceAlpha', 0.5) % Top face
        fill3(X_block([1 2 6 5]), Y_block([1 2 6 5]), Z_block([1 2 6 5]), 'r', 'FaceAlpha', 0.5) % Side face
        fill3(X_block([2 3 7 6]), Y_block([2 3 7 6]), Z_block([2 3 7 6]), 'r', 'FaceAlpha', 0.5) % Side face
        fill3(X_block([3 4 8 7]), Y_block([3 4 8 7]), Z_block([3 4 8 7]), 'r', 'FaceAlpha', 0.5) % Side face
        fill3(X_block([4 1 5 8]), Y_block([4 1 5 8]), Z_block([4 1 5 8]), 'r', 'FaceAlpha', 0.5) % Side face

        hold off
        axis([-1 1 -1 1 0 1])
        view([1 0.5 0.5])
        xlabel('z'); ylabel('x'); zlabel('y')
        grid on
        title('3D view')

        drawnow
    end
    pause(0.5)
end
disp('END of animation')
global torque_log time_log

if isempty(torque_log)
    disp('No torque data recorded. Check if mdlDerivatives is storing data.');
else
    % Extract time and angles
    time = X_out.time; % Time from simulation
    angles = X_out.signals.values; % Joint angles

    % Create a figure for Angles & Torques
    figure;
    for i = 1:6
        subplot(3,2,i);
        yyaxis left
        plot(time, angles(:,i), 'b-', 'LineWidth', 2);
        ylabel('Angle [rad]');

        yyaxis right
        plot(time_log, torque_log(:,i), 'r--', 'LineWidth', 2);
        ylabel('Torque [Nm]');

        title(['Joint ', num2str(i), ' - Angle & Torque']);
        xlabel('Time [s]');
        legend(['Angle ', num2str(i)], ['Torque ', num2str(i)], 'Location', 'best');
        grid on;
    end
end