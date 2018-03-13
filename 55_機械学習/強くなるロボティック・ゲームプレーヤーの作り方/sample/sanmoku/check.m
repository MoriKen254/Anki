% ゲームの勝敗をチェックするためのプログラム
% fin: 0-ゲーム続行, 1-人工知能プレイヤーの勝ち, 2-学習プレイヤーの勝ち, 3-引き分け

function fin=check(s)
  pos = [1 2 3; 4 5 6; 7 8 9; 1 4 7; 2 5 8; 3 6 9; 1 5 9; 3 5 7];

  for i=1:max(size(pos))
    val = prod(s(pos(i,:)));

    % 人工知能プレイヤーの勝ち
    if(val==1)
      fin = 1;
      return;

    % 学習プレイヤーの勝ち
    elseif(val == 8)
      fin = 2;
      return;
    end
  end
  
  % 引き分け
  if(prod(s))
    fin = 3;
    return;
  end
  
  % ゲーム続行
  fin = 0;