function Q=MonteCarloPolicyIteration(L,M,options)
  nstates = 3^9;   % ��Ԑ�
  nactions = 9;    % �s����
  T = 5;           % �ő�X�e�b�v��

  % Q�֐��̏�����
  Q=zeros(nstates,nactions);
  Q=sparse(Q);

  % ��������
  for l=1:L
    visits = ones(nstates,nactions);    % (s,a)�̏o����
    results = zeros(M,1);               % �Q�[���̌���  
    rand('state',1);                    % seed�̏�����
    
    % �G�s�\�[�h
    for m=1:M
      state3 = zeros(1,9);
    
      % �X�e�b�v
      for t=1:T

        % ��Ԃ̃G���R�[�h
        state = encode(state3);
        
        % ����̐���
        policy = zeros(1,nactions);

        switch(options.pmode)
          case 1 % greedy
            [v,a] = max(Q(state,:));
            policy(a) = 1;

          case 2 % e-greedy
            [v,a] = max(Q(state,:));
            policy = ones(1,nactions)*options.epsilon/nactions;
            policy(a) = 1-options.epsilon+options.epsilon/nactions;

          case 3 % softmax
            policy=exp(Q(state,:)/options.tau)/sum(exp(Q(state,:)./options.tau));  
        end

        % �s���̑I������ю��s
        [action,reward,state3,fin] = action_train(policy,t,state3);

        % ��ԁC�s���C��V�C�o���񐔂̍X�V
        states(m,t)     = state;
        actions(m,t)    = action;
        rewards(m,t)    = reward;
        visits(state,action)    = visits(state,action) + 1;

        % �Q�[���I��
        if(fin>0)
          results(m) = fin;

          % ��������V�a�̌v�Z
          drewards(m,t) = rewards(m,t);
          for pstep=t-1:-1:1
            drewards(m,pstep) = options.gamma * drewards(m,pstep+1);
          end
          break;
        end
      end
    end

    % ��ԍs�����l�֐��̌v�Z
    Q=zeros(nstates,nactions);
    Q=sparse(Q);
    for m=1:M
      for t=1:size(states,2)
        s = states(m,t);
        a = actions(m,t); 
        if(s==0)
          break;
        end
        Q(s,a) = Q(s,a) + drewards(m,t);
      end
    end

    Q = Q./visits;

    % �����̌v�Z
    rate(l) = size(find(results==2),1)./M;

    % �W���o��
    fprintf(1,'%d) Win=%d/%d, Draw=%d/%d, Lose=%d/%d\n',l, size(find(results==2),1),M,size(find(results==3),1),M,size(find(results==1),1),M);
    % fflush(stdout);
  end
  
  %�@�O���t�̏o��
  figure(1)
  clf
%  axes('FontSize',15,'LineWidth',2.0);
  games = M:M:M*L;
  g=plot(games,rate);
  set(g,'LineWidth',2);
  g=xlabel('�Q�[����');
  set(g,'FontSize',14);
  g=ylabel('����');
  set(g,'FontSize',14);
  axis([M,M*L,0.4,1])