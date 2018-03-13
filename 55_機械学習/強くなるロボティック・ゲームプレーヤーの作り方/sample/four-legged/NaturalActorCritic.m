function [sigma, mu]=NaturalActorCritic(L, M, T, options, winx,winy)
    startSimulation (winx,winy);	% �{�̂̃E�B���h�E��\��
    MaxTorque = 100;			% �ő�g���N
    MinTorque = -100;			% �ŏ��g���N
    N = 19;				% ���f���p�����[�^���imu:18�����Csigma:1�����j
    ibstate = getBaseState();  		% ���̂̏������

    % ���􃂃f���p�����[�^�������_���ɏ�����
    mu = rand(N-1,1)-0.5;
    sigma = rand*10;

    % �f�U�C���s��Z�C��V�x�N�g��q�����
    % �A�h�o���e�[�W�֐��̃��f���p�����[�^w�̏�����
    Z = zeros(M,N);
    q = zeros(M,1);
    w = zeros(N,1);

    % ��������
    for l=1:L
        dr = 0;
        rand('state',l);

        % �W�{
        for m=1:M
	    resetSimulation();

	    for t=1:T
		% ��Ԃ̏�����
        	state = zeros(N-2,1);

		% �֐ߏ�Ԋϑ�
                jstate = getJointState();
                bstate = getBaseState();

		% ��ԃx�N�g���̍\�z
                state(1:16) = jstate;   % 8�֐߂̊p�x����ё��x
                state(17) = bstate(3);  % ����z�������̈ʒu
                state(18) = bstate(10); % ����z�������̑��x

		% �s���̑I��
		action = randn*sigma + mu'*state;
		action = min(action,MaxTorque); % �ŏ��l�m�F
		action = max(action,MinTorque); % �ő�l�m�F

		% �s���̎��s
		u = zeros(1,8);
		u(2) = action;
		u(4) = action;
		u(6) = action;
		u(8) = action;

                stepSimulation (u,0.0005);
                if(t==0 || mod(t,50)==0)
                   drawWorld;
                end

		% ���̏�Ԋϑ�
                abstate = getBaseState();

		% ��ԁC�s������ѕ�V�̃f�[�^���L�^
                states(:,t)  = state;
                actions(t)   = action;
                rewards(m,t) = abstate(1) - ibstate(1);
		dr   = dr + options.gamma^(t-1)*rewards(m,t);
            end

            for t=1:T
		% ����mu�Ɋւ�����z�̌v�Z
		der(1:N-1) = (actions(t)-mu'*states(:,t))*states(:,t)/(sigma^2);

		% �W���΍�sigma�Ɋւ�����z�̌v�Z
	      	der(N) = ((actions(t)-mu'*states(:,t))^2-sigma^2)/(sigma^3);      

		% �f�U�C���s��Z�����q�x�N�g��
		Z(m,:) = Z(m,:) + options.gamma^(t-1)*der;
		q(m) = q(m) + options.gamma^(t-1)*(rewards(m,t));
	    end
        end

	% r - V(s1)
        q = q - dr/M;

        % �ŏ����@��p���ăA�h�o���e�[�W�֐��̃��f���p�����[�^w�𐄒�
	Z(:,N) = ones(M,1);
	w = pinv(Z'*Z)*Z'*q;

	% w��p���ă��f���p�����[�^���X�V
	mu = mu + options.alpha*w(1:N-1);
	sigma = sigma + options.alpha*w(N);

        printf("%d) Max=%.2f Min=%.2f Avg=%.2f Dsum=%.2f\n",l,max(max(rewards)), min(min(rewards)), mean(mean(rewards)), dr/M);
        fflush(stdout);
    end