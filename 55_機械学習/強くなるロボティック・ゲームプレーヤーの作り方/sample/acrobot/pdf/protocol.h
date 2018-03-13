//-------------------------------------------------------------------------------------------
/*! \file
    \brief  robot simulator protocol
    \author Akihiko Yamaguchi
    \date   Mar.13 2007  */
//-------------------------------------------------------------------------------------------

const char *SOCK_NAME = "/tmp/octrobot-socket";   //!< �ʐM�Ɏg���\�P�b�g�t�@�C��

// �T�[�o�����󂯕t����R�}���h
const int ORS_START_SIM          (0);  //!< �V�~�����[�V�������J�n����
const int ORS_STOP_SIM           (1);  //!< �V�~�����[�V�������I������
const int ORS_STEP_SIM           (2);  //!< �V�~�����[�V������ 1 �X�e�b�v�i�߂� (dvalue=���ԃX�e�b�v)
const int ORS_RESET_SIM          (3);  //!< �V�~�����[�V�������Z�b�g����
const int ORS_DRAW_WORLD         (4);  //!< ���E��`��
const int ORS_SET_TORQUE         (5);  //!< �g���N���͂�ݒ� (step=0,..,�֐ߐ�-1, dvalue)
const int ORS_SET_WINDOWSIZE     (6);  //!< ��ʃT�C�Y��ύX�D�f�t�H���g 400x400 (step=0:x, step=1:y, ivalue)�D ORS_START_SIM �����O�Ɏg��Ȃ��ƈӖ����Ȃ��D
const int ORS_GET_JOINT_NUM      (7);  //!< �֐ߐ���Ԃ��D(int*1)
const int ORS_GET_JSTATE_DIM     (8);  //!< �֐ߏ�Ԃ̎�����Ԃ��D(int*1)
const int ORS_GET_BSTATE_DIM     (9);  //!< �x�[�X��Ԃ̎�����Ԃ��D(int*1)
const int ORS_GET_JOINT_STATE   (10);  //!< �֐ߏ�Ԃ��擾 (double*�֐ߏ�ԃx�N�g�������̃f�[�^���Ԃ����)
const int ORS_GET_BASE_STATE    (11);  //!< �x�[�X��Ԃ��擾 (double*�x�[�X��ԃx�N�g�������̃f�[�^���Ԃ����)

struct TXData  //! �ʐM�Ɏg���f�[�^
{
  int   command;
  int   step;
  union
  {
    int    ivalue;
    double dvalue;
  };
};



