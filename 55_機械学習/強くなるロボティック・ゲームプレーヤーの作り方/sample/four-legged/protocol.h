//-------------------------------------------------------------------------------------------
/*! \file
    \brief  robot simulator protocol
    \author Akihiko Yamaguchi
    \date   Mar.13 2007  */
//-------------------------------------------------------------------------------------------

const char *SOCK_NAME = "/tmp/octrobot-socket";   //!< 通信に使うソケットファイル

// サーバ側が受け付けるコマンド
const int ORS_START_SIM          (0);  //!< シミュレーションを開始する
const int ORS_STOP_SIM           (1);  //!< シミュレーションを終了する
const int ORS_STEP_SIM           (2);  //!< シミュレーションを 1 ステップ進める (dvalue=時間ステップ)
const int ORS_RESET_SIM          (3);  //!< シミュレーションリセットする
const int ORS_DRAW_WORLD         (4);  //!< 世界を描画
const int ORS_SET_TORQUE         (5);  //!< トルク入力を設定 (step=0,..,関節数-1, dvalue)
const int ORS_SET_WINDOWSIZE     (6);  //!< 画面サイズを変更．デフォルト 400x400 (step=0:x, step=1:y, ivalue)． ORS_START_SIM よりも前に使わないと意味がない．
const int ORS_GET_JOINT_NUM      (7);  //!< 関節数を返す．(int*1)
const int ORS_GET_JSTATE_DIM     (8);  //!< 関節状態の次元を返す．(int*1)
const int ORS_GET_BSTATE_DIM     (9);  //!< ベース状態の次元を返す．(int*1)
const int ORS_GET_JOINT_STATE   (10);  //!< 関節状態を取得 (double*関節状態ベクトル次元のデータが返される)
const int ORS_GET_BASE_STATE    (11);  //!< ベース状態を取得 (double*ベース状態ベクトル次元のデータが返される)

struct TXData  //! 通信に使うデータ
{
  int   command;
  int   step;
  union
  {
    int    ivalue;
    double dvalue;
  };
};



