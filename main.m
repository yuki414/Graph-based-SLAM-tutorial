%% graph based slam
% 記述ルール：
% 忘れがちなので記載
% ①二文字以上の添え字はアンダーバー
% ②ただし，cplex用のAeqなどは例外
% ③行列要素はなるべく','で区切る
% ④','後のスペースは適宜長いときに
% ⑤コメントの行列サイズは(m,n)で

clear all
close all
clc

%% options
opt_anime = 1;

%% シミュレーションパラメータ
Ts = 2.0;   % サンプリング時間[s]
T_gslam = 20;   % graph slamのサンプリング
T_fin = 100; % 終了時刻[s]

global MAX_ITR
MAX_ITR = 20;

%% ランドマーク座標設定(RFID)
% ワールド座標系 [x, y, yaw]
RFID = [10, -2, 0;
        15, 10, 0;
        3,  15, 0;
        -5, 20, 0;
        -5, 5,  0];
    
%% 雑音分散設定
randn('state',1);   % 固定乱数生成
Qsim = diag([0.2, 1.0*2*pi/360])^2; % 観測雑音
Rsim = diag([0.1, 10*2*pi/360])^2;  % 入力雑音

%% Covariance parameter of Graph based SLAM
C_sigma1 = 0.1;
C_sigma2 = 0.1;
C_sigma3 = 1.0*2*pi/360;

global sigma
sigma = diag([C_sigma1^2, C_sigma2^2, C_sigma3^2]);
%% 1ステップ挙動計算
% x = [x座標, y座標, yaw]'
% u = [並進速度, 旋回速度]'
% 確率ロボティクスの速度制御モデル（γを除く）
% calc_motion
EYESIGHT = 30.0;    % ロボットの視界
global nx
nx = 3;
nu = 2;
x0 = zeros(nx,1);
u0 = [1; 0.1];  

x = x0;
x_true = x0;    % 真の軌道
x_dr = x0;      % dead reckoning（モデルのみによる推定）
u = u0;
% 初回の観測値
z = [];
for j = 1:size(RFID,1)  % ランドマークの数だけ行う
    % ランドマークとの距離を計算（ロボット座標系）
    dx_lm = RFID(j,1) - x_true(1);
    dy_lm = RFID(j,2) - x_true(2);
    d_RFID = sqrt(dx_lm^2 + dy_lm^2);
    phi = pi2pi(atan2(dy_lm,dx_lm)); % ワールド座標系の値（基本的に使わない）
    angle = phi - x_true(3);

    if d_RFID <= EYESIGHT
        % 視界に入ったら観測&保存
        d_RFIDn = d_RFID + Qsim(1,1)*randn;
        anglen = angle + Qsim(2,2)*randn;
        z = [z;d_RFIDn, anglen, phi, j];
    end      
end

%% データ倉庫（history）
hx_true = x_true;
hx_dr = x_dr;
hz = {z};

%% メインループ
for t = 0:Ts:T_fin - Ts
    % 挙動計算（入力雑音の扱いを変更）
    t_next = t + Ts;
    u_true = u + Rsim*randn(nu,1);              % 入力雑音付加
    x_true = calc_motion(x_true, u, Ts);   % 軌道シミュレート
    
    % dead reckoning
    x_dr = calc_motion(x_dr, u_true, Ts);    % 状態予測に近い
    
    % ランドマーク探索
%     z = zeros(1,4); % 怪しい []でも可？
    z = [];
    for j = 1:size(RFID,1)  % ランドマークの数だけ行う
        % ランドマークとの距離を計算（ロボット座標系）
        dx_lm = RFID(j,1) - x_true(1);
        dy_lm = RFID(j,2) - x_true(2);
        d_RFID = sqrt(dx_lm^2 + dy_lm^2);
        phi = pi2pi(atan2(dy_lm,dx_lm)); % ワールド座標系の値（基本的に使わない）
        angle = phi - x_true(3);
        
        if d_RFID <= EYESIGHT
            % 視界に入ったら観測&保存
            d_RFIDn = d_RFID + Qsim(1,1)*randn;
            anglen = angle + Qsim(2,2)*randn;
            z = [z;d_RFIDn, anglen, phi, j];
        end      
    end
    
    % データ格納
    hx_true = [hx_true, x_true];
    hx_dr = [hx_dr, x_dr];
    hz = [hz z];
    
    % 特定周期ごとにグラフslam計算
    if mod(t_next, T_gslam) == 0
        t_next; % 周期確認用
        x_opt = calc_gslam(hx_dr, hz);
        
        % 任意で推定の様子を確認
        if opt_anime
            figure
            hold on
            grid on
            % ランドマーク
            plot(RFID(:,1), RFID(:,2), '*k')   
            
            % 真の軌道
            plot(hx_true(1,:), hx_true(2,:), 'b')
            % dead reckoning
            plot(hx_dr(1,:), hx_dr(2,:), 'k')
            % 最適推定
            plot(x_opt(1,:), x_opt(2,:), 'r')
            axis equal
        end
    end
end