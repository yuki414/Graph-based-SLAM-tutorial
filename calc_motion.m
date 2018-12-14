%% 1�X�e�b�v�����v�Z�i���[���h���W�n�j
%     �����F
%         ��ԁFx = [x���W, y���W, yaw]'
%         ���́Fu = [���i���x, ���񑬓x]'
%         ���݁Fdt
%     �߂�l�F
%         1�X�e�b�v��̏�ԁFx_next = [x���W, y���W, yaw]'
%     �m�����{�e�B�N�X�̑��x���䃂�f���i���������j
%     ����p�x��x�����獶���
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