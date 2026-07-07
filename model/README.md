# 模型文件

## best-four-new.onnx
- YOLOv8 RKOPT 格式（box/cls 分离输出，9路张量）
- 输入：640×640，4类（no_smoking/smoking/fire/smoke）
- 42.5 MB

## best-four-new-i8.rknn
- INT8 RKOPT 量化模型，适配 RV1126B NPU
- 推理耗时：31ms/帧，模型体积：11.9MB
- 转换命令：`python3 convert.py best-four-new.onnx rv1126b i8 best-four-new-i8.rknn`

## 模型转换脚本（参考）
```python
from rknn.api import RKNN
rknn = RKNN(verbose=True)
rknn.config(mean_values=[[0,0,0]], std_values=[[255,255,255]], target_platform='rv1126b')
rknn.load_onnx(model='best-four-new.onnx')
rknn.build(do_quantization=True, dataset='calib_dataset.txt')
rknn.export_rknn('best-four-new-i8.rknn')
rknn.release()
```
