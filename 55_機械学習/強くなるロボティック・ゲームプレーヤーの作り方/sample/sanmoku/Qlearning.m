function Q=Qlearning(M,options)
  nstates = 3^9;		  % ��Ԑ�
  nactions = 9;		  % �s����
  results = zeros(M,1);   % ���s����
  eM =1000;         % �]�����s���G�s�\�[�h��

  % Q�֐��̏�����
  Q=zeros(nstates,nactions);

  for m=1:M
    rand('state',mod(m,eM))
    t = 1;
    state3 = zeros(1,9);

    while(1)
      % ��ԁC��V�C�Q�[���󋵂̊ϑ�
      state = encode(state3);

      %====================
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
      %====================

      % �s���̑I������ю��s
      [action,reward,state3,fin] = action_train(policy,t,state3);

      %====================
      % Q�֐��̍X�V�iQ�w�K�j
      % 1�X�e�b�v�O�̏�ԁC�s����Q�l���X�V
      if t > 1
        Q(pstate,paction) = Q(pstate,paction) + options.alpha*(reward - Q(pstate,paction) + options.gamma*max(Q(state,:)));
      end

      % �Q�[���I��
      if(fin>0)
        results(m) = fin;
        break;
      end

      % ��Ԃƍs���̋L�^
      pstate = state;
      paction = action;

      t = t + 1;
    end
    
    if(mod(m,eM)==0)
      fprintf(1,'%d) Win=%d/%d, Draw=%d/%d, Lose=%d/%d\n',m, size(find(results(m-eM+1:m)==2),1),eM,size(find(results(m-eM+1:m)==3),1),eM,size(find(results(m-eM+1:m)==1),1),eM);
    end

    fflush(stdout);
  end

  %�@�O���t�̏o��
  results2(results~=2)=0;
  results2(results==2)=1;
  res =reshape(results2,eM,M/eM);
  rate = sum(res)./eM;	
  figure(3)
  clf
%  axes('FontSize',15,'LineWidth',2.0);
  games = eM:eM:M;
  g=plot(games,rate);
  set(g,'LineWidth',2);
  g=xlabel('�Q�[����');
  set(g,'FontSize',14);
  g=ylabel('����');
  set(g,'FontSize',14);
  axis([eM,M,0.4,1])  
