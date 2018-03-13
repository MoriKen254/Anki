// NPC�i�w�K�v���C���[�j�Ƒΐ킵�ėV�Ԃ��߂́CGUI�̃\�[�X�v���O����
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

// �{�^���z��
Fl_Button *b[9];

// ��Ԕz��F0-�����C1-�l�ԁiO�j�C2-�w�K�v���C���[�iX�j
int state[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

//��U�C1-�w�K�v���C���[�C0-�l��
int player = 1;

// �Q�[���̏󋵁F0-���s�C1-�I��
int gameflag = 0;

Fl_Window *window = new Fl_Window(170,190,"OX Game");

//-------------------------------------------------------------------------------------------
/*
    \brief ��Ԃ��t�@�C��state.txt�ɏo�́D
*/
void output(){
  int k;
  FILE *fp, *lock;

  // ���b�N�t�@�C���̐���
  lock = fopen("lock.m", "w");
  
  // ��ԃx�N�g�����o��
  fp = fopen("state.txt","w");
  for(k=0;k<9;k++){
    fprintf(fp,"%d:",state[k]);
  }

  // �Q�[���̏󋵂��o��  
  fprintf(fp,"\n%d",gameflag);

  fclose(fp);
  fclose(lock);
  
  // ���b�N�t�@�C���̍폜
  remove("lock.m");
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/*
    \brief ���s�m�F�D
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
    \brief �s�����t�@�C��action.txt������́D
*/
void action(void*) {
  FILE *fp, *lock;
  errno_t error;
  errno_t error2;
  char cdir[255];
  GetCurrentDirectory(255, cdir);
  // action.txt�������Ă����b�N�t�@�C��������Ԃ͏������ݒ��Ȃ̂ő҂�
  if((error=fopen_s(&fp,"action.txt","r")) != NULL || (error2=fopen_s(&lock, "lock.m", "r")) != NULL){
    Fl::repeat_timeout(2.0,action);
  }else{

    // �s���̃}�X�Ɂ~��u���C��Ԃ�2�Ƃ���
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
    \brief �}�X��̃{�^�����N���b�N���ꂽ���̃C�x���g�D
*/
void button_change(Fl_Widget* o, void *i) {
  Fl_Button *button = (Fl_Button *)o;
  int num;

  if(gameflag)
    return;

  // �w�K�v���C���[�̏��Ԃ̎��́C�������Ȃ��Ŗ߂�
  if(player != 0)
    return;

  // �����ꂽ�{�^�����}�[�N�i���j�ɕύX
  if(state[(int)i] != 0)
    return;
  button->label("O");
  state[(int)i] = 1;

  // �Q�[���I���̏ꍇ�́C��Ԃ��o�͂��ďI���
  if(gameflag = judge((int)i, 1)){
    output();
    return;
  }

  // �Q�[�����s�̏ꍇ�́C���͊w�K�v���C���[�̔�
  for(num=0; num<9; num++)
    if(state[num] == 0)
      break;
  if(num == 9)
    return;

  // ��Ԃ��o��
  output();
  player = 1;

  // �s���t�@�C�����폜
  remove("action.txt");

  // �^�C�}�[
  Fl::add_timeout(1.0,action);
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/*
    \brief reset�{�^�����N���b�N���ꂽ���̃C�x���g�D
*/
void reset(Fl_Widget* o, void *i) {
  int k;

  // �{�^���̏�����
  for(k=0; k<9; k++) {
    state[k] = 0;
    b[k]->label("");
    b[k]->color(FL_WHITE);
  }

  // �t���O�̏�����
  gameflag = 0;

  // ��ԏo��
  output();

  // �^�C�}�[�J�n
  Fl::add_timeout(1.0,action);
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/*
    \brief �O�ڕ��׃Q�[���̃��C���D
*/
int main(int argc, char **argv) {
  remove("action.txt");
  remove("state.txt");

  // ���j���[�o�[
  Fl_Menu_Bar *menubar = new Fl_Menu_Bar(0,0,300,20,0);
  int k;

  // �{�^���̔z�u
  for(k=0; k<9; k++) {
    b[k] = new Fl_Button(5+55*(k%3),25+55*(int)(k/3),50,50,"");
    b[k]->labelsize(36);
    b[k]->labeltype(FL_SHADOW_LABEL);
    b[k]->callback(button_change, (void *)k);
    b[k]->color(FL_WHITE);
  }

  // ���j���[�̒ǉ�
  menubar->add("File/Restart", "", reset, 0);
  window->end();
  window->show(argc, argv);

  // �w�K�v���C���[����s�̏ꍇ
  if(player==1){
    output();
    Fl::add_timeout(1.0,action);
  }

  return Fl::run();
}