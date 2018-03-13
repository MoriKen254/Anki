function theta=KernelLeastSquaresPolicyIteration(L, M, T, options, win_w,win_h)
  startSimulation (win_w,win_h);    % �{�̂̃E�B���h�E��\��  
  actions = [-50, 0, 50];           % �s���̌��
  nactions = 3;                     % �s����

  % �J�l���s��K�C�x�N�g��r�̏�����
  K = zeros(M*T,M*T);
  r = zeros(M*T,1);

  % ���f���p�����[�^�̏�����
  theta = rand(M*T,1);

  % �f�[�^�s��̏������C��Ԏ���+�s������=5
  data = zeros(M*T,5);

  % ��������
  for l=1:L
    dr = 0;
    rand('state',1);

    % �W�{
    for m=1:M
      resetSimulation();

      for t=1:T
        % ��ԁipsi1, psi2, dpsi1, dpsi2�j�̊ϑ�
        state = getJointState();

        if l==1
          policy = ones(nactions,1)./nactions;
        else
          Q(1) = theta'*exp(-sum((pdata-repmat([state' actions(1)],M*T,1)).^2,2)/2/(options.var^2));
          Q(2) = theta'*exp(-sum((pdata-repmat([state' actions(2)],M*T,1)).^2,2)/2/(options.var^2));
          Q(3) = theta'*exp(-sum((pdata-repmat([state' actions(3)],M*T,1)).^2,2)/2/(options.var^2));

          % ����
          policy = zeros(nactions);
          switch options.pmode
            case 1 % greedy
              [v,a] = max(Q);
              policy(a) = 1;

            case 2 % e-greedy
              [v,a] = max(Q);
              policy = ones(nactions,1)*options.epsilon/nactions;
              policy(a) = 1 - options.epsilon+options.epsilon/nactions;
              
            case 3  % softmax           
              policy = exp(Q./options.tau)/sum(exp(Q./options.tau));
          end
        end

        % �s���I��
        ran = rand;
        if(ran < policy(1))
          action = 1;
        elseif(ran < policy(1)+policy(2))
          action = 2;
        else
          action = 3;
        end

        u(2) = actions(action);

        % �s���̎��s
        stepSimulation (u,0.01);
        if(t==0 || mod(t,10)==0)
          drawWorld;
        end

        % �f�[�^�s��̍X�V
        data(T*(m-1)+t,1) = state(1);
        data(T*(m-1)+t,2) = state(2);
        data(T*(m-1)+t,3) = state(3);
        data(T*(m-1)+t,4) = state(4);
        data(T*(m-1)+t,5) = u(2);

        % ��ԁipsi1, psi2, dpsi1, dpsi2�j�̊ϑ�
        state = getJointState();

        % M*T������V�x�N�g��r
        r(T*(m-1)+t) = -cos(state(1));

        % �������a�̌v�Z
        dr = dr + options.gamma^(t-1) * r(T*(m-1)+t);
      end
    end

    % (M*T)*(M*T)�J�[�l���s��̐���
    for mt=1:M*T-1
      K(mt,:) = exp(-sum((data(1:M*T,:)-repmat(data(mt,:),M*T,1)).^2,2)/2/(options.var^2))' - options.gamma * exp(-sum((data(1:M*T,:)-repmat(data(mt+1,:),M*T,1)).^2,2)/2/(options.var^2))';
    end

    % �ŏ����@�ɂ�鐭���]��
    theta = pinv(K)*r;

    pdata = data;

    printf("%d)Max=%.2f Avg=%.2f Dsum=%.2f numtop=%d\n",l,max(r),mean(r),dr/M,size(find(r>0.9),1));
    fflush(stdout);
  end