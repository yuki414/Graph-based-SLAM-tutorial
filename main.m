%% graph based slam
% �L�q���[���F
% �Y�ꂪ���Ȃ̂ŋL��
% �@�񕶎��ȏ�̓Y�����̓A���_�[�o�[
% �A�������Ccplex�p��Aeq�Ȃǂ͗�O
% �B�s��v�f�͂Ȃ�ׂ�','�ŋ�؂�
% �C','��̃X�y�[�X�͓K�X�����Ƃ���
% �D�R�����g�̍s��T�C�Y��(m,n)��

clear all
close all
clc

%% options
opt_anime = 1;

%% �V�~�����[�V�����p�����[�^
Ts = 2.0;   % �T���v�����O����[s]
T_gslam = 20;   % graph slam�̃T���v�����O
T_fin = 100; % �I������[s]

global MAX_ITR
MAX_ITR = 20;

%% �����h�}�[�N���W�ݒ�(RFID)
% ���[���h���W�n [x, y, yaw]
RFID = [10, -2, 0;
        15, 10, 0;
        3,  15, 0;
        -5, 20, 0;
        -5, 5,  0];
    
%% �G�����U�ݒ�
randn('state',1);   % �Œ藐������
Qsim = diag([0.2, 1.0*2*pi/360])^2; % �ϑ��G��
Rsim = diag([0.1, 10*2*pi/360])^2;  % ���͎G��

%% Covariance parameter of Graph based SLAM
C_sigma1 = 0.1;
C_sigma2 = 0.1;
C_sigma3 = 1.0*2*pi/360;

global sigma
sigma = diag([C_sigma1^2, C_sigma2^2, C_sigma3^2]);
%% 1�X�e�b�v�����v�Z
% x = [x���W, y���W, yaw]'
% u = [���i���x, ���񑬓x]'
% �m�����{�e�B�N�X�̑��x���䃂�f���i���������j
% calc_motion
EYESIGHT = 30.0;    % ���{�b�g�̎��E
global nx
nx = 3;
nu = 2;
x0 = zeros(nx,1);
u0 = [1; 0.1];  

x = x0;
x_true = x0;    % �^�̋O��
x_dr = x0;      % dead reckoning�i���f���݂̂ɂ�鐄��j
u = u0;
% ����̊ϑ��l
z = [];
for j = 1:size(RFID,1)  % �����h�}�[�N�̐������s��
    % �����h�}�[�N�Ƃ̋������v�Z�i���{�b�g���W�n�j
    dx_lm = RFID(j,1) - x_true(1);
    dy_lm = RFID(j,2) - x_true(2);
    d_RFID = sqrt(dx_lm^2 + dy_lm^2);
    phi = pi2pi(atan2(dy_lm,dx_lm)); % ���[���h���W�n�̒l�i��{�I�Ɏg��Ȃ��j
    angle = phi - x_true(3);

    if d_RFID <= EYESIGHT
        % ���E�ɓ�������ϑ�&�ۑ�
        d_RFIDn = d_RFID + Qsim(1,1)*randn;
        anglen = angle + Qsim(2,2)*randn;
        z = [z;d_RFIDn, anglen, phi, j];
    end      
end

%% �f�[�^�q�Ɂihistory�j
hx_true = x_true;
hx_dr = x_dr;
hz = {z};

%% ���C�����[�v
for t = 0:Ts:T_fin - Ts
    % �����v�Z�i���͎G���̈�����ύX�j
    t_next = t + Ts;
    u_true = u + Rsim*randn(nu,1);              % ���͎G���t��
    x_true = calc_motion(x_true, u, Ts);   % �O���V�~�����[�g
    
    % dead reckoning
    x_dr = calc_motion(x_dr, u_true, Ts);    % ��ԗ\���ɋ߂�
    
    % �����h�}�[�N�T��
%     z = zeros(1,4); % ������ []�ł��H
    z = [];
    for j = 1:size(RFID,1)  % �����h�}�[�N�̐������s��
        % �����h�}�[�N�Ƃ̋������v�Z�i���{�b�g���W�n�j
        dx_lm = RFID(j,1) - x_true(1);
        dy_lm = RFID(j,2) - x_true(2);
        d_RFID = sqrt(dx_lm^2 + dy_lm^2);
        phi = pi2pi(atan2(dy_lm,dx_lm)); % ���[���h���W�n�̒l�i��{�I�Ɏg��Ȃ��j
        angle = phi - x_true(3);
        
        if d_RFID <= EYESIGHT
            % ���E�ɓ�������ϑ�&�ۑ�
            d_RFIDn = d_RFID + Qsim(1,1)*randn;
            anglen = angle + Qsim(2,2)*randn;
            z = [z;d_RFIDn, anglen, phi, j];
        end      
    end
    
    % �f�[�^�i�[
    hx_true = [hx_true, x_true];
    hx_dr = [hx_dr, x_dr];
    hz = [hz z];
    
    % ����������ƂɃO���tslam�v�Z
    if mod(t_next, T_gslam) == 0
        t_next; % �����m�F�p
        x_opt = calc_gslam(hx_dr, hz);
        
        % �C�ӂŐ���̗l�q���m�F
        if opt_anime
            figure
            hold on
            grid on
            % �����h�}�[�N
            plot(RFID(:,1), RFID(:,2), '*k')   
            
            % �^�̋O��
            plot(hx_true(1,:), hx_true(2,:), 'b')
            % dead reckoning
            plot(hx_dr(1,:), hx_dr(2,:), 'k')
            % �œK����
            plot(x_opt(1,:), x_opt(2,:), 'r')
            axis equal
        end
    end
end