% NPC（学習プレイヤー）が自分の状態を把握するためのプログラム

function [state, state3, fin]=observe_test
  % 状態，報酬，ゲームの状況をファイルから読み込む
  fp=fopen('./state.txt','r');
  while(fp == -1)
    pause(0.01);
  end
%{
  % ロックファイルがあるなら書き込み中なので待つ
  while(exist('lock.m'))
    pause(0.01);
  end
%}
  % 状態
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

  % ゲームの状況
  fin = str2num(fgets(fp));
  
  fclose(fp);
  %munlock('./state.txt');
  %====================

  %状態のエンコード（3進数から10進数）
  state = encode(state3);