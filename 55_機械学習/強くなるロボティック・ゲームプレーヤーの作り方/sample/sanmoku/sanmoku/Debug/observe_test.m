% NPC�i�w�K�v���C���[�j�������̏�Ԃ�c�����邽�߂̃v���O����

function [state, state3, fin]=observe_test
  % ��ԁC��V�C�Q�[���̏󋵂��t�@�C������ǂݍ���
  fp=fopen('./state.txt','r');
  while(fp == -1)
    pause(0.01);
  end
%{
  % ���b�N�t�@�C��������Ȃ珑�����ݒ��Ȃ̂ő҂�
  while(exist('lock.m'))
    pause(0.01);
  end
%}
  % ���
  [tok,rem]=strtok(fgets(fp),':');
  state3(1) = str2num(tok);
  cnt = 2;
  while(true)
    [tok,rem]=strtok(rem,':');
    if isspace(tok(1)) == 1
        break;
    end
    state3(cnt) = str2num(tok);
    cnt = cnt + 1;
  end

  % �Q�[���̏�
  fin = str2num(fgets(fp));
  
  fclose(fp);
  %munlock('./state.txt');
  %====================

  %��Ԃ̃G���R�[�h�i3�i������10�i���j
  state = encode(state3);