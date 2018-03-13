//-------------------------------------------------------------------------------------------
/*! \file
    \brief  4-legged robot simulator - client
    \author Akihiko Yamaguchi
    \date   Mar.13 2007  */
//-------------------------------------------------------------------------------------------
#include <iostream> // TODO デバッグが終了しだい削除
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
#include <octave/config.h>
#include <octave/Matrix.h>
//-------------------------------------------------------------------------------------------
#ifdef OUTPUT_OCT
#include <octave/oct.h>
#endif
//-------------------------------------------------------------------------------------------
static int  client_file_descriptor(-1);
static int  JOINT_NUM(0);
static int  JOINT_STATE_DIM (0);  // 関節状態の次元
static int  BASE_STATE_DIM (0);  // ベース状態の次元
//-------------------------------------------------------------------------------------------
static double *joint_state (NULL);
static double *base_state (NULL);
static ColumnVector jState(0), bState(0);
//-------------------------------------------------------------------------------------------
class __inner_destructor
{
  __inner_destructor(void) {};
  ~__inner_destructor(void)
    {
      if (joint_state!=NULL) {delete[] joint_state; joint_state=NULL;}
      if (base_state!=NULL)  {delete[] base_state; base_state=NULL;}
    };
};
//-------------------------------------------------------------------------------------------


inline ColumnVector get_joint_state (void);
inline ColumnVector get_base_state (void);

using namespace std;

//! check return
void chret (int ret)
{
  if (ret<0)
  {
    close (client_file_descriptor);
    client_file_descriptor = -1;
    exit(1);
  }
}

//! check the client_file_descriptor
void chfd (void)
{
  if(client_file_descriptor<0)
  {
    cerr<<"error! the connection was already terminated."<<endl;
    exit(1);
  }
}

bool setup_client (void)
  // ref. http://www.ueda.info.waseda.ac.jp/~toyama/network/example1.html
{
  struct sockaddr_un    addr;

  // ソケットを作成． UNIX ドメイン，ストリーム型
  if ((client_file_descriptor = socket (PF_UNIX, SOCK_STREAM, 0)) < 0)
  {
    perror("socket");
    exit(1);
  }

  bzero ((char *)&addr, sizeof(addr));

  // ソケットの名前を代入
  addr.sun_family = AF_UNIX;
  strcpy (addr.sun_path, SOCK_NAME);

  // サーバと接続を試みる．サーバ側で bind & listen の発行が終っている必要がある
  if (connect (client_file_descriptor, (struct sockaddr *)&addr,
        sizeof(addr.sun_family) + strlen(SOCK_NAME)) < 0)
  {
    perror("connect");
    cerr<<"-> maybe the server four-legged.exe is not running."<<endl;
    return false; //exit(1);
  }
  return true;
}

ColumnVector get_torque (void)
{
  static bool init(true);
  static const double kp=100.0, kd=2.0;
  static ColumnVector target(JOINT_NUM,0.0);
  static const double MaxTorque(100.0);  // [Nm]
  if(init)
  {
    const double q1=-0.25*M_PI, q2=0.5*M_PI;
    for(int i(0);i<8;i+=2) target(i)=q1;
    for(int i(1);i<8;i+=2) target(i)=q2;
    init = false;
  }
  ColumnVector u (JOINT_NUM,0.0);  // 制御入力（トルク）
  ColumnVector jstate (get_joint_state());  // 現在の関節状態
  for (int i(0);i<8;++i)
  {
    u(i)=kp*(target(i)-jstate(i))-kd*jstate(8+i);
    if (u(i)>MaxTorque)  u(i)=MaxTorque;
    else if (u(i)<-MaxTorque)  u(i)=-MaxTorque;
  }

  return u;
}
//-------------------------------------------------------------------------------------------

inline void start_simulation (int window_width, int window_height)
{
  chfd();
  TXData data;
  data.command = ORS_SET_WINDOWSIZE;
  data.step = 0;
  data.ivalue = window_width;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
  data.step = 1;
  data.ivalue = window_height;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
  data.command = ORS_START_SIM;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
}

inline void stop_simulation (void)
{
  chfd();
  TXData data;
  data.command = ORS_STOP_SIM;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
  close(client_file_descriptor);
  client_file_descriptor=-1;
}

inline void step_simulation (const ColumnVector &u, const double &time_step)
{
  chfd();
  TXData data;
  data.command = ORS_SET_TORQUE;
  for (int j(0); j<JOINT_NUM; ++j)
  {
    data.step = j;
    data.dvalue = u(j);
    chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
  }
  data.command = ORS_STEP_SIM;
  data.dvalue = time_step;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
}

inline void reset_simulation (void)
{
  chfd();
  TXData data;
  data.command = ORS_RESET_SIM;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
}

inline ColumnVector get_joint_state (void)
{
  chfd();
  if (JOINT_STATE_DIM<=0)  return ColumnVector(0);
  if (jState.dim1() != JOINT_STATE_DIM)  jState.resize(JOINT_STATE_DIM);
  TXData data;
  data.command = ORS_GET_JOINT_STATE;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
  chret (read (client_file_descriptor, (char*)joint_state, sizeof(double)*JOINT_STATE_DIM));
  // cerr<<"c-joint1= ";for(int j(0);j<JOINT_STATE_DIM;++j)cerr<<" "<<joint_state[j];cerr<<endl;
  for (int j(0); j<JOINT_STATE_DIM; ++j)  jState(j)=joint_state[j];
  // cerr<<"c-joint2= "<<jState.transpose()<<endl;
  return jState;
}
inline ColumnVector get_base_state (void)
{
  chfd();
  if (BASE_STATE_DIM<=0)  return ColumnVector(0);
  if (bState.dim1() != BASE_STATE_DIM)  bState.resize(BASE_STATE_DIM);
  TXData data;
  data.command = ORS_GET_BASE_STATE;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
  chret (read (client_file_descriptor, (char*)base_state, sizeof(double)*BASE_STATE_DIM));
  for (int j(0); j<BASE_STATE_DIM; ++j)  bState(j)=base_state[j];
  return bState;
}

inline void draw_world (void)
{
  chfd();
  TXData data;
  data.command = ORS_DRAW_WORLD;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
}

inline int get_joint_num (void)
{
  chfd();
  TXData data;
  data.command = ORS_GET_JOINT_NUM;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
  chret (read (client_file_descriptor, (char*)&JOINT_NUM, sizeof(JOINT_NUM)));
  cerr<<"joint-num = "<<JOINT_NUM<<endl;
  return JOINT_NUM;
}

inline int get_joint_state_dim (void)
{
  chfd();
  TXData data;
  data.command = ORS_GET_JSTATE_DIM;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
  chret (read (client_file_descriptor, (char*)&JOINT_STATE_DIM, sizeof(JOINT_STATE_DIM)));
  cerr<<"joint-state-dim = "<<JOINT_STATE_DIM<<endl;
  if (joint_state!=NULL) {delete[] joint_state; joint_state=NULL;}
  joint_state = new double[JOINT_STATE_DIM];
  return JOINT_STATE_DIM;
}

inline int get_base_state_dim (void)
{
  chfd();
  TXData data;
  data.command = ORS_GET_BSTATE_DIM;
  chret (write (client_file_descriptor, (char*)&data, sizeof(data)));
  chret (read (client_file_descriptor, (char*)&BASE_STATE_DIM, sizeof(BASE_STATE_DIM)));
  cerr<<"base-state-dim = "<<BASE_STATE_DIM<<endl;
  if (base_state!=NULL) {delete[] base_state; base_state=NULL;}
  base_state = new double[BASE_STATE_DIM];
  return BASE_STATE_DIM;
}

#ifdef OUTPUT_OCT
DEFUN_DLD (startSimulation, args, ,
  "int startSimulation(int window_width, int window_height).")
{
  if(!setup_client()) return octave_value(1);
  start_simulation(args(0).double_value(), args(1).double_value());
  get_joint_num();
  get_joint_state_dim();
  get_base_state_dim();
  return octave_value(0);
}

DEFUN_DLD (stopSimulation, args, ,
  "void stopSimulation(void).")
{
  stop_simulation();
  return octave_value();
}

DEFUN_DLD (stepSimulation, args, ,
  "void stepSimulation(const ColumnVector &u, const dReal &time_step).")
{
  ColumnVector u(args(0).vector_value());
  step_simulation (u, args(1).double_value());
  return octave_value();
}

DEFUN_DLD (resetSimulation, args, ,
  "void resetSimulation(void).")
{
  reset_simulation();
  return octave_value();
}

DEFUN_DLD (drawWorld, args, ,
  "void drawWorld(void).")
{
  draw_world();
  return octave_value();
}

DEFUN_DLD (getJointState, args, ,
  "ColumnVector getJointState(void).")
{
  ColumnVector state (get_joint_state());
  return octave_value (state);
}

DEFUN_DLD (getBaseState, args, ,
  "ColumnVector getBaseState(void).")
{
  ColumnVector state (get_base_state());
  return octave_value (state);
}
#endif


int main (int argc, char **argv)
{
  if(!setup_client()) return 1;
  start_simulation(400,400);
  get_joint_num();
  get_joint_state_dim();
  get_base_state_dim();
  while(1)
  {
    step_simulation (get_torque(), 0.001);
    draw_world();
  }

  close (client_file_descriptor);
  return 0;
}
//-------------------------------------------------------------------------------------------

