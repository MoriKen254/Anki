// NPC（学習プレイヤー）と対戦して遊ぶための，GUIのソースプログラム
//-------------------------------------------------------------------------------------------
/*
    \brief  sanmoku game interface
    \author Tsubasa Fukuyama
    \date   Mar.31 2008
*/
//-------------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS
#define WIN32
#include "FL/Fl.H"
#include "FL/Fl_Window.H"
#include "FL/Fl_Box.H"
#include "FL/Fl_Button.H"
#include "FL/Fl_Menu_Bar.H"
#include "FL/Fl_Menu_Item.H"
#include "FL/Fl_ask.H"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Windows.h>

// ボタン配列
Fl_Button *b[9];

// 状態配列：0-無し，1-人間（O），2-学習プレイヤー（X）
int state[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

//先攻，1-学習プレイヤー，0-人間
int player = 1;

// ゲームの状況：0-続行，1-終了
int gameflag = 0;

Fl_Window *window = new Fl_Window(170,190,"OX Game");

//-------------------------------------------------------------------------------------------
/*
    \brief 状態をファイルstate.txtに出力．
*/
void output(){
  int k;
  FILE *fp, *lock;

  // ロックファイルの生成
  lock = fopen("lock.m", "w");
  
  // 状態ベクトルを出力
  fp = fopen("state.txt","w");
  for(k=0;k<9;k++){
    fprintf(fp,"%d:",state[k]);
  }

  // ゲームの状況を出力  
  fprintf(fp,"\n%d",gameflag);

  fclose(fp);
  fclose(lock);
  
  // ロックファイルの削除
  remove("lock.m");
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/*
    \brief 勝敗確認．
*/
int judge(int pos, int turn) {
  int i,j,k;
  char message[256] = "";

  i = (int)pos / 3;
  j = pos % 3;

  for(k=0; k<3; k++)
    if(state[i*3+k] != turn)
      break;
      
  if(k == 3) {
    for(k=0; k<3; k++)
      b[i*3+k]->color(turn==1?FL_RED:FL_BLUE);
      window->redraw();

    sprintf(message, "%s Win%s!!", turn==1?"You":"COM", turn==1?"":"s");
    fl_alert(message);
    return 1;
  }

  for(k=0; k<3; k++)
    if(state[j+k*3] != turn)
      break;
      
  if(k == 3) {
    for(k=0; k<3; k++)
      b[j+k*3]->color(turn==1?FL_RED:FL_BLUE);
    window->redraw();

    sprintf(message, "%s Win%s!!", turn==1?"You":"COM", turn==1?"":"s");
    fl_alert(message);
    return 1;
  }

  if(i==j) {
    for(k=0; k<3; k++)
      if(state[4*k] != turn)
        break;
    if(k == 3) {
      for(k=0; k<3; k++)
        b[4*k]->color(turn==1?FL_RED:FL_BLUE);
      window->redraw();

      sprintf(message, "%s Win%s!!", turn==1?"You":"COM", turn==1?"":"s");
      fl_alert(message);
      return 1;
    }
  }
  if((2-i)==j) {
    for(k=0; k<3; k++)
      if(state[2*(k+1)] != turn)
        break;
    if(k == 3) {
      for(k=0; k<3; k++)
        b[2*(k+1)]->color(turn==1?FL_RED:FL_BLUE);
      window->redraw();

      sprintf(message, "%s Win%s!!", turn==1?"You":"COM", turn==1?"":"s");
      fl_alert(message);
      return 1;
    }
  }
  return 0;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
/*
    \brief 行動をファイルaction.txtから入力．
*/
void action(void*) {
  FILE *fp, *lock;
  errno_t error;
  errno_t error2;
  char cdir[255];
  GetCurrentDirectory(255, cdir);
  // action.txtがあってもロックファイルがある間は書き込み中なので待つ
  if((error=fopen_s(&fp,"action.txt","r")) != NULL || (error2=fopen_s(&lock, "lock.m", "r")) != NULL){
    Fl::repeat_timeout(2.0,action);
  }else{

    // 行動のマスに×を置き，状態を2とする
    char str[10];
    fgets(str,10,fp);
    int num = atoi(str);
    fclose(lock);

    b[num]->label("X");
    state[num] = 2;
    player = 0;
    fclose(fp);
    remove("action.txt");
    
    if(gameflag = judge((int)num, 2)){
      output();
      return;
    }
    
  }
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/*
    \brief マス上のボタンがクリックされた時のイベント．
*/
void button_change(Fl_Widget* o, void *i) {
  Fl_Button *button = (Fl_Button *)o;
  int num;

  if(gameflag)
    return;

  // 学習プレイヤーの順番の時は，何もしないで戻す
  if(player != 0)
    return;

  // 押されたボタンをマーク（○）に変更
  if(state[(int)i] != 0)
    return;
  button->label("O");
  state[(int)i] = 1;

  // ゲーム終了の場合は，状態を出力して終わり
  if(gameflag = judge((int)i, 1)){
    output();
    return;
  }

  // ゲーム続行の場合は，次は学習プレイヤーの番
  for(num=0; num<9; num++)
    if(state[num] == 0)
      break;
  if(num == 9)
    return;

  // 状態を出力
  output();
  player = 1;

  // 行動ファイルを削除
  remove("action.txt");

  // タイマー
  Fl::add_timeout(1.0,action);
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/*
    \brief resetボタンがクリックされた時のイベント．
*/
void reset(Fl_Widget* o, void *i) {
  int k;

  // ボタンの初期化
  for(k=0; k<9; k++) {
    state[k] = 0;
    b[k]->label("");
    b[k]->color(FL_WHITE);
  }

  // フラグの初期化
  gameflag = 0;

  // 状態出力
  output();

  // タイマー開始
  Fl::add_timeout(1.0,action);
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/*
    \brief 三目並べゲームのメイン．
*/
int main(int argc, char **argv) {
  remove("action.txt");
  remove("state.txt");

  // メニューバー
  Fl_Menu_Bar *menubar = new Fl_Menu_Bar(0,0,300,20,0);
  int k;

  // ボタンの配置
  for(k=0; k<9; k++) {
    b[k] = new Fl_Button(5+55*(k%3),25+55*(int)(k/3),50,50,"");
    b[k]->labelsize(36);
    b[k]->labeltype(FL_SHADOW_LABEL);
    b[k]->callback(button_change, (void *)k);
    b[k]->color(FL_WHITE);
  }

  // メニューの追加
  menubar->add("File/Restart", "", reset, 0);
  window->end();
  window->show(argc, argv);

  // 学習プレイヤーが先行の場合
  if(player==1){
    output();
    Fl::add_timeout(1.0,action);
  }

  return Fl::run();
}