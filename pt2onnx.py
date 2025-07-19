import torch
import torch.nn
import onnx
 
model = torch.torch.jit.load('./model/policy_4.pt')
model.eval()
 
input_names = ['input']
output_names = ['output']
 
x = torch.randn(1,270,requires_grad=True)

torch.onnx.export(model, x, './onnx/policy_4.onnx', input_names=input_names, output_names=output_names, verbose='True')