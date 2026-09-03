#!/usr/bin/env python3
"""Run ONNX or the actual nncase K230 simulator; compare canonical 6524 outputs.

Independent mode uses the same saved FP32 states in both models. Recurrent mode
feeds each model's own state back and resets at sequence_id boundaries. Never
treat unrelated calibration samples as one recurrent sequence.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np

INPUTS = ('input_imgs', 'big_input_imgs', 'desire', 'traffic_convention', 'initial_state')
PROBS = np.arange(990, 4955, 991)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_outputs(path):
    path = Path(path)
    if path.suffix == '.npy':
        return np.load(path, allow_pickle=False)
    import struct
    blob = path.read_bytes()
    if len(blob) < 16:
        raise ValueError('Truncated board dump header')
    magic, width, count = struct.unpack('<8sII', blob[:16])
    if magic != b'SCODMP1\0' or width != 6524 or count == 0 or len(blob) != 16 + count*width*4:
        raise ValueError('Invalid board output dump')
    return np.frombuffer(blob, '<f4', offset=16).reshape(count,width)


def stats(values):
    x = np.abs(np.asarray(values, dtype=np.float64)).ravel()
    return dict(mae=float(x.mean()), rmse=float(np.sqrt(np.mean(x*x))),
                p95=float(np.percentile(x, 95)), max=float(x.max()))


def compare(reference, candidate):
    if reference.shape != candidate.shape or reference.ndim != 2 or reference.shape[1] != 6524 or len(reference) == 0:
        raise ValueError(f'Expected matching [N,6524], got {reference.shape}, {candidate.shape}')
    if not np.isfinite(reference).all() or not np.isfinite(candidate).all():
        raise ValueError('Nonfinite model output')
    # Original ONNX emits absolute logits, the runtime model emits deltas.
    r, c = reference.copy(), candidate.copy()
    r[:, PROBS] -= r[:, PROBS[:1]]
    c[:, PROBS] -= c[:, PROBS[:1]]
    ri, ci = r[:, PROBS].argmax(1), c[:, PROBS].argmax(1)
    n = len(r)
    rp = r[:, :4955].reshape(n, 5, 991)[np.arange(n), ri, :495].reshape(n, 33, 15)
    cp = c[:, :4955].reshape(n, 5, 991)[np.arange(n), ci, :495].reshape(n, 33, 15)
    same_cp = c[:, :4955].reshape(n, 5, 991)[np.arange(n), ri, :495].reshape(n, 33, 15)
    mask = np.ones(6524, dtype=bool); mask[PROBS] = False
    margin = np.sort(r[:, PROBS], axis=1)[:, -1] - np.sort(r[:, PROBS], axis=1)[:, -2]
    result = {'samples': n, 'non_plan_logits': stats(c[:, mask]-r[:, mask]),
              'state': stats(c[:, -512:]-r[:, -512:]),
              'pose': stats(c[:, 6000:6012]-r[:, 6000:6012]),
              'lane_geometry': stats(c[:, 4955:5483]-r[:, 4955:5483]),
              'plan_logits_delta': stats(c[:, PROBS]-r[:, PROBS]),
              'plan_matches': int((ri == ci).sum()),
              'plan_match_rate': float((ri == ci).mean()),
              'candidate_top_ties': int(((c[:, PROBS] == c[:, PROBS].max(1, keepdims=True)).sum(1) > 1).sum()),
              'mismatch_reference_margin': stats(margin[ri != ci]) if (ri != ci).any() else None}
    for axis, k in [('x', 0), ('y', 1), ('z', 2)]:
        result[f'selected_path_{axis}_m'] = stats(cp[:,:,k]-rp[:,:,k])
        result[f'same_hypothesis_path_{axis}_m'] = stats(same_cp[:,:,k]-rp[:,:,k])
    # Near-term path matters separately from the far end of the 10 s horizon.
    result['selected_path_y_first_17_knots_m'] = stats(cp[:,:17,1]-rp[:,:17,1])
    for name, start, end in [('lane_means',4955,5219),('lane_logits',5483,5491),
                             ('road_edges',5491,5755),('lead',5755,5860),('desire_logits',5912,5920)]:
        result[name] = stats(c[:,start:end]-r[:,start:end])
    return result


def run(args):
    args.model=args.model.resolve(); args.data=args.data.resolve()
    args.out=args.out.resolve(); args.contract=args.contract.resolve()
    with np.load(args.data, allow_pickle=False) as archive:
        data = {name: archive[name] for name in archive.files}
    count = len(data[INPUTS[0]])
    count = min(count, args.limit) if args.limit else count
    if args.recurrent and 'sequence_id' not in data:
        raise ValueError('Recurrent evaluation requires sequence_id in the dataset')
    if args.model.suffix == '.onnx':
        import onnxruntime as ort
        options = ort.SessionOptions(); options.intra_op_num_threads = args.threads
        options.log_severity_level = 3
        engine = ort.InferenceSession(str(args.model), options, providers=['CPUExecutionProvider'])
        specs = [(x.name, x.shape, np.uint8 if x.type == 'tensor(uint8)' else np.float32) for x in engine.get_inputs()]
        def infer(feed):
            return np.concatenate([o.ravel() for o in engine.run(None, feed)])
        backend = f'onnxruntime {ort.__version__} CPU'
    else:
        import nncase
        import importlib.metadata
        import os
        import tempfile
        # The simulator creates/deletes gmodel_dump_dir relative to cwd.
        scratch = tempfile.TemporaryDirectory(prefix='supercombo-sim-')
        os.chdir(scratch.name)
        # nncase-kpu installs its subprocess simulator outside /usr/local/bin.
        package = importlib.metadata.distribution('nncase-kpu')
        for f in package.files:
            if f.name == 'nncase.simulator.k230.sc':
                os.environ['PATH'] = str(package.locate_file(f).parent) + os.pathsep + os.environ['PATH']
        engine = nncase.Simulator(); engine.load_model(args.model.read_bytes())
        import onnx
        graph = onnx.load(str(args.contract))
        graph_inputs = [x for x in graph.graph.input if x.name not in {v.name for v in graph.graph.initializer}]
        specs = [(x.name, engine.get_input_shape(i), np.uint8 if x.type.tensor_type.elem_type == 2 else np.float32)
                 for i, x in enumerate(graph_inputs)]
        if engine.inputs_size != len(specs):
            raise ValueError('kmodel/ONNX input count mismatch')
        def infer(feed):
            for i, (name, _, _) in enumerate(specs):
                engine.set_input_tensor(i, nncase.RuntimeTensor.from_numpy(feed[name]))
            engine.run()
            return np.concatenate([engine.get_output_tensor(i).to_numpy().ravel() for i in range(engine.outputs_size)])
        backend = f'nncase {importlib.metadata.version("nncase")} K230 simulator'
    rows, timings = [], []
    state = np.zeros((1,512), np.float32)
    prev_seq = None
    for i in range(count):
        feed = {name: np.ascontiguousarray(data[name][i].reshape(shape), dtype=dtype) for name, shape, dtype in specs}
        if args.recurrent:
            seq = int(data['sequence_id'][i])
            if seq != prev_seq:
                state = feed['initial_state'].copy()
            feed['initial_state'] = state
            prev_seq = seq
        t = time.monotonic(); out = infer(feed); timings.append(time.monotonic()-t)
        if out.size != 6524 or not np.isfinite(out).all():
            raise ValueError(f'Invalid output at sample {i}: shape={out.shape}')
        state = out[-512:].copy().reshape(1,512)
        rows.append(out)
        if i % 20 == 0:
            print(f'{args.model.name}: {i+1}/{count}, {timings[-1]:.3f}s', flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.save(args.out, np.stack(rows))
    info = dict(model=str(args.model), model_sha256=sha256(args.model), data=str(args.data),
                data_sha256=sha256(args.data), backend=backend, recurrent=args.recurrent,
                samples=count, mean_simulation_seconds=float(np.mean(timings)))
    args.out.with_suffix('.json').write_text(json.dumps(info, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('board-dump')
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p = commands.add_parser('run')
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--contract', type=Path, default=Path('models/onnx/supercombo_uint8.onnx'))
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--recurrent', action='store_true')
    p.add_argument('--limit', type=int)
    p.add_argument('--threads', type=int, default=4)
    p = commands.add_parser('compare')
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'board-dump':
        np.save(args.out, load_outputs(args.input))
    elif args.command == 'run':
        run(args)
    else:
        result = compare(load_outputs(args.reference), load_outputs(args.candidate))
        result['reference'] = str(args.reference); result['candidate'] = str(args.candidate)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
