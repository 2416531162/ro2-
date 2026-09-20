#!/usr/bin/env python3
"""Build FP16 RK3588 pose weights from the Rockchip model-zoo ONNX export.

Run on Linux with rknn-toolkit2 and onnx installed. No INT8 calibration guessed.
"""
import argparse
from pathlib import Path


def validate_onnx(path):
    import onnx
    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    shapes = [tuple(d.dim_value for d in o.type.tensor_type.shape.dim) for o in model.graph.output]
    if (len(shapes) != 4 or set(shapes[:3]) != {(1,65,80,80),(1,65,40,40),(1,65,20,20)}
            or shapes[3] not in ((1,17,3,8400),(1,51,8400))):
        raise ValueError(f'Unsupported pose export: {shapes}')
    return shapes


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('onnx', nargs='?', type=Path, default=root/'models/yolov8n-pose.onnx')
    parser.add_argument('--output', type=Path, default=root/'models/yolov8n_pose_rk3588_fp16.rknn')
    parser.add_argument('--validate-image', type=Path, help='Run toolkit simulator on a person image before accepting the export')
    args = parser.parse_args()
    image = None
    if args.validate_image:
        import cv2
        image = cv2.imread(str(args.validate_image))
        if image is None:
            parser.error('Cannot read validation image: '+str(args.validate_image))
    print('Validated outputs:', validate_onnx(args.onnx), flush=True)
    from rknn.api import RKNN
    runtime = RKNN(verbose=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix('.tmp.rknn')
    try:
        runtime.config(mean_values=[[0,0,0]], std_values=[[255,255,255]], target_platform='rk3588')
        for label, operation in (
            ('load', lambda: runtime.load_onnx(model=str(args.onnx))),
            ('build FP16', lambda: runtime.build(do_quantization=False)),
            ('export', lambda: runtime.export_rknn(str(temporary))),
        ):
            code = operation()
            if code != 0:
                raise RuntimeError(f'RKNN {label} failed: {code}')
        if args.validate_image:
            import sys
            sys.path.insert(0, str(root))
            from pose_inference import letterbox, postprocess
            tensor, transform = letterbox(image)
            if runtime.init_runtime() != 0:
                raise RuntimeError('Toolkit simulator initialization failed')
            detections = postprocess(runtime.inference(inputs=[tensor], data_format=['nhwc']), transform)
            if not detections:
                raise RuntimeError('No person detected in validation image')
            print('Simulator people:', len(detections), 'scores:',
                  [round(d['conf'], 3) for d in detections], flush=True)
        temporary.replace(args.output)
        print('Created', args.output, flush=True)
    finally:
        runtime.release()
        temporary.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
