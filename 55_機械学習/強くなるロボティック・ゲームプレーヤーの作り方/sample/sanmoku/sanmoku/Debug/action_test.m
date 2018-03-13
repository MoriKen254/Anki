% NPC（学習プレイヤー）が行動を選択するためのプログラム

function [a]=action_test(policy,step,state3)
  % 最初のステップでは1マス目を選択
  if(step == 1)
    a=1;
  else

    % 政策policyに従いランダムに行動を選択
    while(1)
      random = rand;

      cprob = 0;
      for a=1:9      
        cprob = cprob + policy(a);
        if(random < cprob)
          break;
        end
      end

      % 既にマスが埋まっていないかどうかを確認
      if state3(a)==0
        break;
      end
    end
  end

  % 書いている間は読み込みをロックする
  lock = fopen('./lock.m', 'w');

  % 行動を出力
  fp = fopen('./action.txt','w');
  if fp > 0
      fprintf(fp,'%d',a-1);
    fclose(fp);
  end

  % 書き込みが終わったのでロック解除
  if lock > 0
      fclose(lock);
  end
%  munlock('./lock.m');