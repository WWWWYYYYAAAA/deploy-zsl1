import torch
import torch.onnx

# 1. 加载ONNX模型
model = torch.torch.jit.load('./model/policy_4.pt')

# 2. 验证模型
torch.onnx.checker.check_model(model)
print("ONNX模型验证通过!")