# l4dc
ROS2を使ったオムニホイールの足回り制御コードです。

## launchファイル
l4dc3.launch  
l4dc4.launch  

## l4dc4_node
shirasuを使った4輪オムニの制御用コードです。joy_nodeと併用することで回転と全方向に直進の動作が可能です。  

使用コントローラー:logcool F710?(DモードMODEランプ消灯時)  

Bボタン mode_vel?に変更  
Aボタン mode_disable?に変更  

左スティック 倒した方向に移動  
LB　左回転  
RB　右回転  

## l4dc3_node
l4dc4_nodeを3輪オムニ用に改造したもの  

使用コントローラー:logcool F710?(DモードMODEランプ消灯時)  

Bボタン mode_vel?に変更  
Aボタン mode_disable?に変更  

左スティック 倒した方向に移動   
LB　左回転  
RB　右回転  
