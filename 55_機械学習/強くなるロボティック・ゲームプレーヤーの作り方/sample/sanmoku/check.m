% �Q�[���̏��s���`�F�b�N���邽�߂̃v���O����
% fin: 0-�Q�[�����s, 1-�l�H�m�\�v���C���[�̏���, 2-�w�K�v���C���[�̏���, 3-��������

function fin=check(s)
  pos = [1 2 3; 4 5 6; 7 8 9; 1 4 7; 2 5 8; 3 6 9; 1 5 9; 3 5 7];

  for i=1:max(size(pos))
    val = prod(s(pos(i,:)));

    % �l�H�m�\�v���C���[�̏���
    if(val==1)
      fin = 1;
      return;

    % �w�K�v���C���[�̏���
    elseif(val == 8)
      fin = 2;
      return;
    end
  end
  
  % ��������
  if(prod(s))
    fin = 3;
    return;
  end
  
  % �Q�[�����s
  fin = 0;