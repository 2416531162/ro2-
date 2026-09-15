import hashlib
import json
from pathlib import Path
from rknn.api import RKNN
import onnx

p = Path(__file__).resolve().parent
model = p.parent/'models/yolov8n.onnx'
outputs = [f'/model.22/cv{branch}.{scale}/cv{branch}.{scale}.2/Conv_output_0'
           for scale in range(3) for branch in (2, 3)]
graph = onnx.load(str(model))
converted = onnx.version_converter.convert_version(graph, 19)
onnx.checker.check_model(converted)
compatible_model = p/'yolov8n_opset19.onnx'
onnx.save(converted, str(compatible_model))
names = {name for node in graph.graph.node for name in node.output}
assert all(name in names for name in outputs)
rknn = RKNN(verbose=False)
try:
    assert rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
                       target_platform='rk3588', optimization_level=3) == 0
    assert rknn.load_onnx(model=str(compatible_model), outputs=outputs) == 0
    assert rknn.build(do_quantization=False) == 0
    target = p/'yolov8n_rk3588_fp16.rknn'
    assert rknn.export_rknn(str(target)) == 0
    metadata = dict(source_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                    rknn_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                    platform='rk3588', toolkit='2.3.2', precision='FP16',
                    input='RGB uint8 NHWC 1x640x640x3; mean=0; std=255',
                    output_heads=outputs, class_scores='logits', dfl_bins=16)
    (p/'model.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print('CONVERSION_PASS', json.dumps(metadata), flush=True)
finally:
    rknn.release()
