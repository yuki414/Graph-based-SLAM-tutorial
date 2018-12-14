%% graph based slamの計算
%     引数：
%         dead reckoning系列：hx_dr
%         ランドマーク観測系列：hz
%     戻り値：
%         最適推定系列：x_opt
function x_opt = calc_gslam(hx_dr, hz)
global MAX_ITR nx
disp('start graph based slam')
zlist = hz;   % deepcopyがわからなかったので仮設置
% zlist = [zlist{1}, zlist];    % おそらくhx_drとサイズを合わせているだけ+最初の2つで原点固定を行うのか?
% そうでもなさそうか？最初の1つで同じことをやってもよさそう，そもそも1つ目は時系列があっていない

x_opt = hx_dr;      % 初期化
nt = size(x_opt,2); % 要素数確認
N = nx*nt;      % ループで使うはず

% イタレーションここから
for j = 1:MAX_ITR
    % エッジコスト計算（すべての組み合わせに対するコストの総和を確認）
    edges = calc_edges(x_opt, zlist);
    
    % 情報行列Hと情報ベクトルb(3*(系列+1)おそらく0時刻のこと)
    H = zeros(N,N);
    b = zeros(N,1);
    
    % すべてのエッジから情報行列・ベクトルを生成
    for k = 1:size(edges,2)
        [H, b] = make_partHb(H, b, edges(k));
    end
    
    % 原点座標は修正する必要はないので情報を重くする，繰り返しごとに増加，最初からではだめ？
    H(1:nx, 1:nx) = H(1:nx, 1:nx) + eye(nx);
    
    % 修正値計算(時系列すべてが縦に並ぶことに注意)
    dx = -inv(H)*b;
    
    % 推定値修正，履歴の各列に修正値を追加
    for l = 1:nt
        x_opt(:,l) = x_opt(:,l) + dx(nx*(l-1)+1:nx*l,1);
    end
    
    % イタレーション終了条件
    diff = dx'*dx;  % 修正量のノルムの二乗
%     fprintf('iteration: %d, diff: %f\n', j, diff)
    if diff < 1e-5
        diff
        break
    end    
end