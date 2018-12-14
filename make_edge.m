%% �G�b�W����
%     �����F
%         ���Ȑ���n��Fxlist1, xlist2
%         �����h�}�[�N�n��Fzlist1, zlist2
%         �ϑ��ԍ��Ft1, t2
%     �߂�l�F
%         �G�b�W���\����
function edge = make_edge(xlist1, xlist2, zlist1, zlist2, t1, t2)
global sigma
edge = struct;
% zlist1
% zlist2
% ���o
yaw1 = xlist1(3);
yaw2 = xlist2(3);
angle1 = zlist1(2);
angle2 = zlist2(2);

% ���[���h���W�n�̊p�x�Z�o�i�m�C�Y�������ꂽphi�j
tangle1 = pi2pi(yaw1 + angle1);
tangle2 = pi2pi(yaw2 + angle2);

% ���o
x1 = xlist1(1);
y1 = xlist1(2);
x2 = xlist2(1);
y2 = xlist2(2);
d1 = zlist1(1);
d2 = zlist2(1);

% ���[���h���W�n�̃����h�}�[�N�ʒu
tdx1 = x1 + d1*cos(tangle1);
tdy1 = y1 + d1*sin(tangle1);
tdx2 = x2 + d2*cos(tangle2);
tdy2 = y2 + d2*sin(tangle2);

% �����͗v�l�� �Ȃ�phi���g��? �΍��̂Ƃ������������
hyaw = zlist1(3) - zlist2(3) + angle1 - angle2;

% 2�̊ϑ��l�̕΍����v�Z
edge.e = [tdx2; tdy2] - [tdx1; tdy1];
% edge.e(3) = pi2pi(yaw2 - yaw1 - hyaw);
edge.e(3) = pi2pi(yaw2 - yaw1 - zlist1(3) + zlist2(3));

% ��]�s��쐬
R1 = [cos(tangle1), -sin(tangle1), 0;
    sin(tangle1), cos(tangle1), 0
    0, 0, 1];
R2 = [cos(tangle2), -sin(tangle2), 0;
    sin(tangle2), cos(tangle2), 0
    0, 0, 1];

% �Ȃ��̃Ћ����U�s��i�{����d�Ɉˑ�?�j
sig1 = sigma;
sig2 = sigma;

edge.OMEGA = inv(R1*sig1*R1' + R2*sig2*R2');
[edge.d1,edge.d2] = deal(d1,d2);
[edge.yaw1,edge.yaw2] = deal(yaw1,yaw2);
[edge.angle1,edge.angle2] = deal(angle1,angle2);
[edge.t1,edge.t2] = deal(t1,t2);


 
