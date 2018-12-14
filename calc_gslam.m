%% graph based slam�̌v�Z
%     �����F
%         dead reckoning�n��Fhx_dr
%         �����h�}�[�N�ϑ��n��Fhz
%     �߂�l�F
%         �œK����n��Fx_opt
function x_opt = calc_gslam(hx_dr, hz)
global MAX_ITR nx
disp('start graph based slam')
zlist = hz;   % deepcopy���킩��Ȃ������̂ŉ��ݒu
% zlist = [zlist{1}, zlist];    % �����炭hx_dr�ƃT�C�Y�����킹�Ă��邾��+�ŏ���2�Ō��_�Œ���s���̂�?
% �����ł��Ȃ��������H�ŏ���1�œ������Ƃ�����Ă��悳�����C��������1�ڂ͎��n�񂪂����Ă��Ȃ�

x_opt = hx_dr;      % ������
nt = size(x_opt,2); % �v�f���m�F
N = nx*nt;      % ���[�v�Ŏg���͂�

% �C�^���[�V������������
for j = 1:MAX_ITR
    % �G�b�W�R�X�g�v�Z�i���ׂĂ̑g�ݍ��킹�ɑ΂���R�X�g�̑��a���m�F�j
    edges = calc_edges(x_opt, zlist);
    
    % ���s��H�Ə��x�N�g��b(3*(�n��+1)�����炭0�����̂���)
    H = zeros(N,N);
    b = zeros(N,1);
    
    % ���ׂẴG�b�W������s��E�x�N�g���𐶐�
    for k = 1:size(edges,2)
        [H, b] = make_partHb(H, b, edges(k));
    end
    
    % ���_���W�͏C������K�v�͂Ȃ��̂ŏ����d������C�J��Ԃ����Ƃɑ����C�ŏ�����ł͂��߁H
    H(1:nx, 1:nx) = H(1:nx, 1:nx) + eye(nx);
    
    % �C���l�v�Z(���n�񂷂ׂĂ��c�ɕ��Ԃ��Ƃɒ���)
    dx = -inv(H)*b;
    
    % ����l�C���C�����̊e��ɏC���l��ǉ�
    for l = 1:nt
        x_opt(:,l) = x_opt(:,l) + dx(nx*(l-1)+1:nx*l,1);
    end
    
    % �C�^���[�V�����I������
    diff = dx'*dx;  % �C���ʂ̃m�����̓��
%     fprintf('iteration: %d, diff: %f\n', j, diff)
    if diff < 1e-5
        diff
        break
    end    
end