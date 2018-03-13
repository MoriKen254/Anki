% �w�K��������𗘗p���邽�߂̃v���O����
function test(Q,options)
  nactions = 9;	 % �s����
  step = 1;      % �X�e�b�v�J�E���^�[

  while(1)
    % ��ԁC��V�C�Q�[���󋵂̊ϑ�
    [state, state3, fin]=observe_test;

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
        policy=exp(Q(state,:)/options.tau)/sum(exp(Q(state,:)/options.tau));
      end

      % �s���̑I������ю��s
      [a] = action_test(policy,step,state3);
      step = step + 1;
  end