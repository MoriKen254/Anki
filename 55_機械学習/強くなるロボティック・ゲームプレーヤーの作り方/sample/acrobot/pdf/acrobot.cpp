
//-------------------------------------------------------------------------------------------
/*! \file
    \brief  4-legged robot simulator - server
    \author Akihiko Yamaguchi
    \date   Mar.13 2007  */
//-------------------------------------------------------------------------------------------
#ifndef ODE_MINOR_VERSION
  #error ODE_MINOR_VERSION should be set in compile
  #error   ex. -DODE_MINOR_VERSION=10
#endif
//-------------------------------------------------------------------------------------------
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <iostream>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION
#include <octave/config.h>
#include <octave/Matrix.h>
//-------------------------------------------------------------------------------------------
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
//-------------------------------------------------------------------------------------------
#include "protocol.h"
//-------------------------------------------------------------------------------------------
#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif
// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#endif
//-------------------------------------------------------------------------------------------
using namespace std;
//-------------------------------------------------------------------------------------------
#include "robot.cpp"
//-------------------------------------------------------------------------------------------


//===========================================================================================
//! \brief ふたつのオブジェクト o1, o2 が衝突しそうならこのコールバック関数が呼ばれる
//! \note 衝突しているかいないかはこの関数で（ユーザが）判定し，衝突していれば接触点にリンクを追加する．
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
//===========================================================================================
{
  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (int i=0; i<MAX_CONTACTS; i++)
  {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.soft_cfm = 0.01;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact)))
  {
    for (int i=0; i<numc; i++)
    {
      dJointID c = dJointCreateContact (world.id(),contactgroup.id(),contact+i);
      dJointAttach (c,b1,b2);
    }
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
//! \brief start simulation - set viewpoint
static void start()
//===========================================================================================
{
  #if ODE_MINOR_VERSION>=10
    dAllocateODEDataForThread(dAllocateMaskAll);
  #endif

  static float xyz[3] = {0.75,1.3,1.0};
  static float hpr[3] = {-120.0,-16.0,0.0};

  dsSetViewpoint (xyz,hpr);
  // cerr << "Press 'R' to reset simulation\n" << endl;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! \brief キーイベントのコールバック関数
//! \param[in] cmd  入力キー
static void keyEvent (int cmd)
//===========================================================================================
{
  // if (cmd=='r'||cmd=='R')
  // {
  //   create_world();
  // }
}
//-------------------------------------------------------------------------------------------

static void getJointState (double state[JOINT_STATE_DIM])
{
  for (int j(0);j<JOINT_NUM;++j)
  {
    state[j] = joint[j].getAngle();
    state[JOINT_NUM+j] = joint[j].getAngleRate();
  }
  // cerr<<"joint1= ";for(int j(0);j<JOINT_STATE_DIM;++j)cerr<<" "<<state[j];cerr<<endl;
}
//-------------------------------------------------------------------------------------------

static void getBaseState (double state[BASE_STATE_DIM])
{
  state[0]  = body[0].getPosition()[0];  // x
  state[1]  = body[0].getPosition()[1];  // y
  state[2]  = body[0].getPosition()[2];  // z
  state[3]  = body[0].getQuaternion()[0]; // quaternion (w)
  state[4]  = body[0].getQuaternion()[1]; // quaternion (x)
  state[5]  = body[0].getQuaternion()[2]; // quaternion (y)
  state[6]  = body[0].getQuaternion()[3]; // quaternion (z)

  state[7]  = body[0].getLinearVel()[0];  // vx
  state[8]  = body[0].getLinearVel()[1];  // vy
  state[9]  = body[0].getLinearVel()[2];  // vz
  state[10] = body[0].getAngularVel()[0] ; // wx
  state[11] = body[0].getAngularVel()[1] ; // wx
  state[12] = body[0].getAngularVel()[2] ; // wx
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
static int    global_file_descriptor(-1);
static int    window_x(400), window_y(400);
// static const  dReal time_step (0.0005); // シミュレーションきざみ幅(0.5[ms])
static ColumnVector  input_torque (JOINT_NUM, 0.0);
//-------------------------------------------------------------------------------------------



void stepSimulation (const dReal &time_step)
{
  for (int j(0); j<JOINT_NUM; ++j)
    joint[j].addTorque(input_torque(j));
  // cerr<<"torque= "<<input_torque.transpose()<<endl;

  // シミュレーション
  space.collide (0,&nearCallback);
  world.step (time_step);
  // time += time_step;
  // remove all contact joints
  contactgroup.empty();
}
//-------------------------------------------------------------------------------------------


bool oct_robot_server (void)
{
  TXData data;
  while (1)
  {
    if (global_file_descriptor<0)
    {
      cerr<<"connection terminated (unexpected error)."<<data.command<<endl;
      exit(1);
    }
    read (global_file_descriptor, (char*)&data, sizeof(data));
    switch (data.command)
    {
      case  ORS_START_SIM        :
        return true;
      case  ORS_STOP_SIM         :
        return false;
      case  ORS_STEP_SIM         :
        stepSimulation (data.dvalue);
        break;
      case  ORS_RESET_SIM        :
        create_world();
        break;
      case  ORS_DRAW_WORLD       :
        return true;
      case  ORS_SET_TORQUE       :
        input_torque(data.step) = data.dvalue;
        break;
      case  ORS_SET_WINDOWSIZE   :
        if (data.step==0)  window_x=data.ivalue;
        else if (data.step==1)  window_y=data.ivalue;
        break;
      case  ORS_GET_JOINT_NUM    :
        write (global_file_descriptor, (char*)&JOINT_NUM, sizeof(JOINT_NUM));
        break;
      case  ORS_GET_JSTATE_DIM   :
        write (global_file_descriptor, (char*)&JOINT_STATE_DIM, sizeof(JOINT_STATE_DIM));
        break;
      case  ORS_GET_BSTATE_DIM   :
        write (global_file_descriptor, (char*)&BASE_STATE_DIM, sizeof(BASE_STATE_DIM));
        break;
      case  ORS_GET_JOINT_STATE  :
        getJointState (joint_state);
        // cerr<<"joint2= ";for(int j(0);j<JOINT_STATE_DIM;++j)cerr<<" "<<joint_state[j];cerr<<endl;
        write (global_file_descriptor, (char*)joint_state, sizeof(double)*JOINT_STATE_DIM);
        break;
      case  ORS_GET_BASE_STATE   :
        getBaseState (base_state);
        write (global_file_descriptor, (char*)base_state, sizeof(double)*BASE_STATE_DIM);
        break;
      default :
        cerr<<"in oct_robot_server(): invalid command "<<data.command<<endl;
        return false;
    }
  }
}
//-------------------------------------------------------------------------------------------

void setup_server (void)
  // ref. http://www.ueda.info.waseda.ac.jp/~toyama/network/example1.html
{
  int    fd1;
  struct sockaddr_un    saddr;
  struct sockaddr_un    caddr;
  int    len;

  if ((fd1 = socket (PF_UNIX, SOCK_STREAM, 0)) < 0)
  {
    perror("socket");
    exit(1);
  }

  bzero ((char *)&saddr, sizeof(saddr));
  // ソケットの名前を代入
  saddr.sun_family = AF_UNIX;
  strcpy (saddr.sun_path, SOCK_NAME);
  // ソケットにアドレスをバインド
  unlink(SOCK_NAME);
  if (bind(fd1, (struct sockaddr *)&saddr,
          sizeof(saddr.sun_family) + strlen(SOCK_NAME)) < 0){
    perror("bind");
    exit(1);
  }
  // listen をソケットに対して発行
  if (listen(fd1, 1) < 0)
  {
    perror("listen");
    exit(1);
  }

  len = sizeof(caddr);
  /*
    * accept()により、クライアントからの接続要求を受け付ける。
    * 成功すると、クライアントと接続されたソケットのディスクリプタが
    * fd2に返される。このfd2を通して通信が可能となる。
    * fd1は必要なくなるので、close()で閉じる。
    */
  if ((global_file_descriptor = accept(fd1, (struct sockaddr *)&caddr, (socklen_t*)&len)) < 0)
  {
      perror("accept");
      exit(1);
  }
  close(fd1);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*! \brief 描画(OpenGL)のコールバック関数．
    \param[in] pause 停止モードなら true (0以外)

    シミュレーションのきざみ time_step=0.0005[s] に対して描画は 50 fps 程度で十分なので，
    1 frame ごとに simStepsPerFrame=1.0/time_step/FPS=40 回ダイナミクスのシミュレーションを回す． */
static void simLoop (int pause)
//===========================================================================================
{
//   static dReal time(0.0); // シミュレーション時間
  if (!pause)
  {
    if (!oct_robot_server())  dsStop();
  }

  draw_world();
}
//-------------------------------------------------------------------------------------------

static void stopSimulation (void)
{
  close (global_file_descriptor);
  global_file_descriptor = -1;
}
//-------------------------------------------------------------------------------------------


int main (int argc, char **argv)
{
  dsFunctions fn; // OpenGL 出力用オブジェクト
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &keyEvent;
  fn.stop = &stopSimulation;
  char path_to_textures[]="textures";
  fn.path_to_textures = path_to_textures;  //! \note カレントディレクトリに textures へのリンクが必要

  #if ODE_MINOR_VERSION>=9
  # if ODE_MINOR_VERSION==9
    dInitODE();
  # elif ODE_MINOR_VERSION>=10
    dInitODE2(0);
  # endif
  #endif

  setup_server();
  if (oct_robot_server())
  {
    create_world();
    // run simulation
    if(window_x>0 && window_y>0)
      dsSimulationLoop (argc,argv,window_x,window_y,&fn);
    else
      while(oct_robot_server());
  }
  if(global_file_descriptor!=-1)
  {
    close (global_file_descriptor);
    global_file_descriptor = -1;
  }

  #if ODE_MINOR_VERSION>=9
    dCloseODE();
  #endif
  return 0;
}
//-------------------------------------------------------------------------------------------
