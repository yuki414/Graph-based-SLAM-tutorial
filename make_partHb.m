%% �G�b�W�ɑΉ��������s��E�x�N�g���̈ꕔ���v�Z
%     �����F
%         ���s��FH
%         ���x�N�g���Fb
%         �������ރG�b�W�Fedge
%     �߂�l�F
%         ���s��FH
%         ���x�N�g���Fb
function [H, b] = make_partHb(H, b, edge)
global nx
% ���R�r�s����v�Z����2����
phi1 = edge.yaw1 + edge.angle1;
Ja1 = [-1, 0, edge.d1*sin(phi1);
    0,  -1, -edge.d1*cos(phi1);
    0,  0,  -1];

phi2 = edge.yaw2 + edge.angle2;
Ja2 = [1, 0, -edge.d2*sin(phi2);
    0,  1, edge.d2*cos(phi2);
    0,  0,  1];

% �f�[�^�ԍ��i���U����+1�j�𒊏o
t1 = edge.t1;
t2 = edge.t2;

% ���ϐ�
id1end = t1*nx;
id2end = t2*nx;
id1org = id1end - nx + 1;
id2org = id2end - nx + 1;

% ���s��H�̏�������
H(id1org:id1end, id1org:id1end) = H(id1org:id1end, id1org:id1end) + Ja1'*edge.OMEGA*Ja1;
H(id1org:id1end, id2org:id2end) = H(id1org:id1end, id2org:id2end) + Ja1'*edge.OMEGA*Ja2;
H(id2org:id2end, id1org:id1end) = H(id2org:id2end, id1org:id1end) + Ja2'*edge.OMEGA*Ja1;
H(id2org:id2end, id2org:id2end) = H(id2org:id2end, id2org:id2end) + Ja2'*edge.OMEGA*Ja2;

% H
% pause
% ���x�N�g��b�̏�������
b(id1org:id1end, 1) = b(id1org:id1end, 1) + Ja1'*edge.OMEGA*edge.e;
b(id2org:id2end, 1) = b(id2org:id2end, 1) + Ja2'*edge.OMEGA*edge.e;