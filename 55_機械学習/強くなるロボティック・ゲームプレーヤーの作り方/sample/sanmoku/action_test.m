% NPC�i�w�K�v���C���[�j���s����I�����邽�߂̃v���O����

function [a]=action_test(policy,step,state3)
  % �ŏ��̃X�e�b�v�ł�1�}�X�ڂ�I��
  if(step == 1)
    a=1;
  else

    % ����policy�ɏ]�������_���ɍs����I��
    while(1)
      random = rand;

      cprob = 0;
      for a=1:9      
        cprob = cprob + policy(a);
        if(random < cprob)
          break;
        end
      end

      % ���Ƀ}�X�����܂��Ă��Ȃ����ǂ������m�F
      if state3(a)==0
        break;
      end
    end
  end

  % �����Ă���Ԃ͓ǂݍ��݂����b�N����
  lock = fopen('./lock.m', 'w');

  % �s�����o��
  fp = fopen('./action.txt','w');
  fprintf(fp,'%d',a-1);
  fclose(fp);
  delete fp;

  % �������݂��I������̂Ń��b�N����
  fclose(lock);
  %munlock('./lock.m');