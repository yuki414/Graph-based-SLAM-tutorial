%% エッジに対応した情報行列・ベクトルの一部を計算
%     引数：
%         情報行列：H
%         情報ベクトル：b
%         書き込むエッジ：edge
%     戻り値：
%         情報行列：H
%         情報ベクトル：b
function [H, b] = make_partHb(H, b, edge)
global nx
% ヤコビ行列を計算して2分割
phi1 = edge.yaw1 + edge.angle1;
Ja1 = [-1, 0, edge.d1*sin(phi1);
    0,  -1, -edge.d1*cos(phi1);
    0,  0,  -1];

phi2 = edge.yaw2 + edge.angle2;
Ja2 = [1, 0, -edge.d2*sin(phi2);
    0,  1, edge.d2*cos(phi2);
    0,  0,  1];

% データ番号（離散時刻+1）を抽出
t1 = edge.t1;
t2 = edge.t2;

% 仮変数
id1end = t1*nx;
id2end = t2*nx;
id1org = id1end - nx + 1;
id2org = id2end - nx + 1;

% 情報行列Hの書き込み
H(id1org:id1end, id1org:id1end) = H(id1org:id1end, id1org:id1end) + Ja1'*edge.OMEGA*Ja1;
H(id1org:id1end, id2org:id2end) = H(id1org:id1end, id2org:id2end) + Ja1'*edge.OMEGA*Ja2;
H(id2org:id2end, id1org:id1end) = H(id2org:id2end, id1org:id1end) + Ja2'*edge.OMEGA*Ja1;
H(id2org:id2end, id2org:id2end) = H(id2org:id2end, id2org:id2end) + Ja2'*edge.OMEGA*Ja2;

% H
% pause
% 情報ベクトルbの書き込み
b(id1org:id1end, 1) = b(id1org:id1end, 1) + Ja1'*edge.OMEGA*edge.e;
b(id2org:id2end, 1) = b(id2org:id2end, 1) + Ja2'*edge.OMEGA*edge.e;
