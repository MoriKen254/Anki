function [theta]=LeastSquaresPolicyIteration(L, M, T, B, options, win_w,win_h)
  startSimulation (win_w,win_h);    % �{�̂̃E�B���h�E��\��  
  actions = [-50, 0, 50];           % �s���̌��
  nactions = 3;  		    % �s����

  % �f�U�C���s��X�C�x�N�g��r�̏�����
  X = zeros(M*T,B*nactions);
  r = zeros(M*T,1);
  
  % ���f���p�����[�^�̏�����
  theta = zeros(B*nactions,1);

  % ��������
  for l=1:L
    dr = 0;
    rand('state',1);
    
    % �W�{
    for m=1:M
      resetSimulation();

      for t=1:T+1

        % ��ԁipsi1, psi2, dpsi1, dpsi2�j�̊ϑ�
        state = getJointState();

        % ����
        dist = sum((options.centers - repmat(state',B,1)).^2,2);

        % ���݂̏�ԂɊւ�����֐�
        phis = exp(-dist/2/(options.var^2));

        % ���݂̏�Ԃɂ����鉿�l�֐�
        Q = phis'*reshape(theta,B,nactions);

        % ����
        policy = zeros(nactions,1);
        switch options.pmode
          case 1 % greedy
            [v,a] = max(Q);
            policy(a) = 1;

          case 2 % e-greedy
            if l==1
              policy = ones(nactions,1)./nactions;
            else
              [v,a] = max(Q);
              policy = ones(nactions,1)*options.epsilon/nactions;
              policy(a) = 1 - options.epsilon+options.epsilon/nactions;
            end

          case 3  % softmax
            policy = exp(Q./options.tau)/sum(exp(Q./options.tau));
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
        stepSimulation (u,0.005);
        if(t==0 || mod(t,10)==0)
           drawWorld;
        end
        
        if t>1
          % ���݂̏�ԂɊւ�����֐��̐���Ɋւ��镽��
          aphi = zeros(B*nactions,1);
          for a=1:nactions
            aphi = aphi + getPhi(state,a,options.centers,B,options.var,nactions) * policy(a);
          end
          
          % ��O�̏�Ԃƍs���Ɋւ�����֐�
          pphi = getPhi(pstate,paction,options.centers,B,options.var,nactions);
				
          % (M*T)*B�f�U�C���s��X�CM*T�����x�N�g��r
          X(T*(m-1)+t-1,:) = (pphi - options.gamma * aphi)';
          r(T*(m-1)+t-1) = -cos(state(1));

          % �������a�̌v�Z
          dr = dr + r(T*(m-1)+t-1)*options.gamma^(t-1);
        end

        paction = action;
        pstate = state;
      end
    end

    % �����]��
    theta = pinv(X'*X)*X'*r;

    printf("%d)Max=%.2f Avg=%.2f Dsum=%.2f numtop=%d\n",l,max(r),mean(r),dr/M,size(find(r>0.9),1));
    fflush(stdout);
  end