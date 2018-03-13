//-------------------------------------------------------------------------------------------
/*! \file
    \brief  robot simulator protocol
    \author Akihiko Yamaguchi
    \date   Mar.13 2007  */
//-------------------------------------------------------------------------------------------

const char *SOCK_NAME = "/tmp/octrobot-socket";   //!< �̿��˻Ȥ������åȥե�����

// ������¦�������դ��륳�ޥ��
const int ORS_START_SIM          (0);  //!< ���ߥ�졼�����򳫻Ϥ���
const int ORS_STOP_SIM           (1);  //!< ���ߥ�졼������λ����
const int ORS_STEP_SIM           (2);  //!< ���ߥ�졼������ 1 ���ƥå׿ʤ�� (dvalue=���֥��ƥå�)
const int ORS_RESET_SIM          (3);  //!< ���ߥ�졼�����ꥻ�åȤ���
const int ORS_DRAW_WORLD         (4);  //!< ����������
const int ORS_SET_TORQUE         (5);  //!< �ȥ륯���Ϥ����� (step=0,..,�����-1, dvalue)
const int ORS_SET_WINDOWSIZE     (6);  //!< ���̥��������ѹ����ǥե���� 400x400 (step=0:x, step=1:y, ivalue)�� ORS_START_SIM �������˻Ȥ�ʤ��Ȱ�̣���ʤ���
const int ORS_GET_JOINT_NUM      (7);  //!< ��������֤���(int*1)
const int ORS_GET_JSTATE_DIM     (8);  //!< ������֤μ������֤���(int*1)
const int ORS_GET_BSTATE_DIM     (9);  //!< �١������֤μ������֤���(int*1)
const int ORS_GET_JOINT_STATE   (10);  //!< ������֤���� (double*������֥٥��ȥ뼡���Υǡ������֤����)
const int ORS_GET_BASE_STATE    (11);  //!< �١������֤���� (double*�١������֥٥��ȥ뼡���Υǡ������֤����)

struct TXData  //! �̿��˻Ȥ��ǡ���
{
  int   command;
  int   step;
  union
  {
    int    ivalue;
    double dvalue;
  };
};



