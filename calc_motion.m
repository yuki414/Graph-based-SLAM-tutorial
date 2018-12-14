%% 1ステップ挙動計算（ワールド座標系）
%     引数：
%         状態：x = [x座標, y座標, yaw]'
%         入力：u = [並進速度, 旋回速度]'
%         刻み：dt
%     戻り値：
%         1ステップ先の状態：x_next = [x座標, y座標, yaw]'
%     確率ロボティクスの速度制御モデル（γを除く）
%     旋回角度はx軸から左回り
function x_next = calc_motion(x,u,dt)
yaw = x(3);
v = u(1);
omega = u(2);

% dx = [-sin(yaw) + sin(yaw + omega*dt);
%     cos(yaw) - cos(yaw + omega*dt);
%     omega*dt];
% dx = dx*v/omega;

dx = [dt*cos(yaw), 0;
    dt*sin(yaw), 0;
    0,  dt]*u;

x_next = x + dx;