import torch
import torch.onnx

# 1. 加载ONNX模型
model = torch.onnx.load("./onnx/policy_4.onnx")

# 2. 验证模型
torch.onnx.checker.check_model(model)
print("ONNX模型验证通过!")