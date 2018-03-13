function Q=TDLambdaPolicyIteration(L,M,options)
  nstates = 3^9;   % ��Ԑ�
  nactions = 9;    % �s����
  T = 5;           % �ő�X�e�b�v��
  
  % Q�֐��̏�����
  Q=zeros(nstates,nactions);
  Q=sparse(Q);
    
  for l=1:L
    results = zeros(M,1);           % �Q�[���̌���  
    rand('state',1);                % seed�̏�����
    newQ=zeros(nstates,nactions);   % ���l�֐��̏�����
    e=zeros(nstates,nactions);      % �K�i�x�̏�����

    for m=1:M
      state3 = zeros(1,9);

      for t=1:T
        % ��ԁC��V�C�Q�[���󋵂̊ϑ�
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
      
        % 1�X�e�b�v�O�̏�ԁC�s����Q�l���X�V
        if t > 1
          % �K�i�x�̍X�V
          e = e.*options.gamma*options.lambda;
          e(pstate, paction) = e(pstate, paction) + 1;

          newQ = newQ + options.alpha * e * (reward - newQ(pstate,paction) + options.gamma * newQ(state,action));
        end

        % �Q�[���I��    
        if(fin>0)
          results(m) = fin;
          break;
        end

        % ��Ԃƍs���̋L�^
        pstate = state;
        paction = action;
      end
    end
    
    Q = newQ;

    % �����̌v�Z
    rate(l) = size(find(results==2),1)./M;
    
    % �W���o��
    fprintf(1,'%d) Win=%d/%d, Draw=%d/%d, Lose=%d/%d\n',l, size(find(results==2),1),M,size(find(results==3),1),M,size(find(results==1),1),M);
    fflush(stdout);
  end
  
  %�@�O���t�̏o��
  figure(1)
  clf
  %axes('FontSize',15,'LineWidth',2.0);
  games = M:M:M*L;
  g=plot(games,rate);
  set(g,'LineWidth',2);
  g=xlabel('�Q�[����');
  set(g,'FontSize',14);
  g=ylabel('����');
  set(g,'FontSize',14);
  axis([M,M*L,0.4,1])
