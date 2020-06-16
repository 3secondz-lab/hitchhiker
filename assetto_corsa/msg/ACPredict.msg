# ACPredict Msg

# output : preds (torch.Tensor, shape: (1, 20), 0.1초 간격으로 2초앞까지의 예측 속력, [0][0]이 0.1초 후 예측 속력),
#          alphas_d (torch.Tensor, shape: (1, 20, len_d), 매 예측시 사용된 attention weight 값),
#          alphas_c (torch.Tensor, shape: (1, 20, len_c), 매 예측시 사용된 attention weight 값)

std_msgs/Header header
std_msgs/Float32MultiArray preds
std_msgs/Float32MultiArray alphas_d
std_msgs/Float32MultiArray alphas_c