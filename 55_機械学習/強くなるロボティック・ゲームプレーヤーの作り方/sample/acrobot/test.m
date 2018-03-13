function test()
  if(startSimulation(400,400)~=0) return; endif % シミュレーション開始(ウィンドウサイズ)
  more off
  printf('press any key..\n')
  kbhit();
  tidx = 9;
  printf('press q: quit\n')
  K = [500,0,50,0; 0,500,0,50; 0,0,0,0; 0,0,0,0];
  target = [
    0,     0, 0, 0;
    0.46, -1.83, 0, 0;
    0.06, -1.07, 0, 0;
    0.12,  2.01, 0, 0;
    0.67, -0.08, 0, 0;
    0.65, -2.53, 0, 0;
   -1.76, -3.10, 0, 0;
   -0.02, -3.04, 0, 0;
    1.71, -2.55, 0, 0;
   -0.01,  1.01, 0, 0;
   -1.28,  3.00, 0, 0;
   -2.86,  2.00, 0, 0;
   -2.30,  1.10, 0, 0;
   -1.00, -0.90, 0, 0;
    0.02,  0.95, 0, 0;
    1.52,  0.1, 0, 0;
    3.13, -2.6, 0, 0;
  ];
  MaxTorque = 200.0;
  while (1)
    state = getJointState(); % acrobot の [q0,q1,dq0,dq1]' を返す
    u = K * (target(tidx,:)' - state);
    stepSimulation (u,0.005);  % トルク，時間幅
    key=kbhit(1);
    switch key
      case 'q';  stopSimulation(); printf('\n'); return;    % シミュレーションを終了
      case 'n';  tidx = tidx + 1; printf('target: %.2f %.2f\n',target(tidx,1),target(tidx,2));
    endswitch
    drawWorld();
  endwhile