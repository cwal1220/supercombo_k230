#!/usr/bin/env python3
"""Build PTQ samples or contiguous evaluation clips from K230 HEVC recordings.

Uses the production C++ fixed12 warp, decoded YUV (no RGB round-trip), both
virtual cameras, true adjacent-frame history, recorded desire edges and FP32
warmup states. Calibration is the saved route snapshot, with roll removed as in
modeld; compressed video and a fixed snapshot are not exact online ISP inputs.
Select disjoint routes in separate manifest files for calibration and evaluation.
"""
import argparse
import json
import struct
import subprocess
from pathlib import Path

import numpy as np
import onnxruntime as ort

INPUTS = ('input_imgs','big_input_imgs','desire','traffic_convention','initial_state')


def frame_index(path):
    b = path.read_bytes()
    magic, version, hs, rs, w, h, fps, start, _ = struct.unpack_from('<8sIIIIIIQQ', b)
    if magic != b'K230IDX1' or hs != 48 or rs != 40 or (len(b)-hs) % rs:
        raise ValueError(f'Invalid frame index: {path}')
    records = np.frombuffer(b, dtype=np.dtype([('frame_id','<u8'),('timestamp','<u8'),
              ('encode_index','<u8'),('offset','<u8'),('size','<u4'),('flags','<u4')]), offset=hs)
    return w, h, fps, records


def controls(route):
    """v2..v5 share the ControlState prefix through desire at byte offset 96."""
    rows = []
    paths = sorted(route.glob('events/*.bin')) or list(route.glob('events.bin'))
    for path in paths:
        b = path.read_bytes()
        if b[:8] != b'K230LOG1': raise ValueError(f'Invalid event file: {path}')
        version, pos = struct.unpack_from('<II', b, 8)
        if version not in (2,3,4,5): raise ValueError(f'Unknown event version {version}')
        while pos+16 <= len(b):
            t, kind, flags, size = struct.unpack_from('<QHHI', b, pos); pos += 16
            if pos+size > len(b): raise ValueError(f'Truncated event: {path}')
            if kind == 4 and size >= 100:
                d = struct.unpack_from('<I', b, pos+96)[0]
                speed = struct.unpack_from('<f', b, pos+56)[0]
                if d > 7 or not np.isfinite(speed): raise ValueError(f'Invalid ControlState prefix: {path}')
                rows.append((t,d,speed))
            pos += size
    return np.array(rows, dtype=[('timestamp','<u8'),('desire','<u4'),('speed_kph','<f4')])


def decode_warp(video, start_frame, count, dims, rpy, warp_binary):
    w,h = dims
    # Decode from the segment start so non-keyframe seeks cannot change indexing.
    cmd = ['ffmpeg','-v','error','-threads','2','-i',str(video),'-vf',
           f'trim=start_frame={start_frame}:end_frame={start_frame+count}',
           '-fps_mode','passthrough','-pix_fmt','nv12','-f','rawvideo','pipe:1']
    ff = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    warped = subprocess.run([str(warp_binary),str(w),str(h),*[str(x) for x in rpy]],
                            stdin=ff.stdout, capture_output=True)
    ff.stdout.close()
    stderr = ff.stderr.read(); status = ff.wait()
    if status or warped.returncode:
        raise RuntimeError(f'Decode/warp failed: {stderr.decode()} {warped.stderr.decode()}')
    arr = np.frombuffer(warped.stdout, np.uint8)
    if arr.size != count*2*6*128*256:
        raise ValueError(f'Expected {count} frames, got {arr.size//(2*6*128*256)}: {video}')
    return arr.reshape(count,2,6,128,256)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--warp-binary', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--warmup', type=int, default=40)
    p.add_argument('--frames', type=int, default=40)
    p.add_argument('--stride', type=int, default=1, help='Keep every Nth sample after real consecutive warmup')
    args = p.parse_args()
    if min(args.warmup,args.frames,args.stride) <= 0: raise ValueError('counts must be positive')
    manifest = json.loads(args.manifest.read_text())
    options = ort.SessionOptions(); options.intra_op_num_threads=4; options.log_severity_level=3
    session = ort.InferenceSession(str(args.model),options,providers=['CPUExecutionProvider'])
    collected = {name:[] for name in INPUTS}; metadata=[]; ids=[]; refs=[]
    cached_controls={}
    for sid, clip in enumerate(manifest):
        video = Path(clip['video']).resolve(); route=video.parent.parent.parent
        w,h,fps,idx=frame_index(video.with_name('frames.bin'))
        if (w,h) != (1280,720): raise ValueError(f'Unsupported camera geometry: {w}x{h}')
        start=int(clip['start_frame']); count=args.warmup+args.frames
        if start < 0 or start+count > len(idx): raise ValueError(f'Clip beyond index: {video}')
        calib=json.loads((route/'params/calibration.json').read_text())
        rpy=[0.0,*calib['rpy_rad'][1:]]
        warped=decode_warp(video,start,count,(w,h),rpy,args.warp_binary.resolve())
        if str(route) not in cached_controls: cached_controls[str(route)]=controls(route)
        ctrl=cached_controls[str(route)]
        state=np.zeros((1,512),np.float32); previous=np.zeros_like(warped[0]);prev_desire=0
        for j,current in enumerate(warped):
            timestamp=int(idx[start+j]['timestamp'])
            ci=int(np.searchsorted(ctrl['timestamp'],timestamp,side='right'))-1
            fresh=ci>=0 and timestamp-int(ctrl[ci]['timestamp'])<=500_000_000
            desire=int(ctrl[ci]['desire']) if fresh else 0
            pulse=np.zeros((1,8),np.float32)
            if desire and desire!=prev_desire: pulse[0,desire]=1
            feed={'input_imgs':np.concatenate((previous[0],current[0]))[None].astype(np.float32),
                  'big_input_imgs':np.concatenate((previous[1],current[1]))[None].astype(np.float32),
                  'desire':pulse,'traffic_convention':np.array([[1,0]],np.float32),'initial_state':state}
            raw=np.concatenate([x.ravel() for x in session.run(None,feed)])
            if not np.isfinite(raw).all(): raise ValueError(f'Nonfinite reference: {video} frame {j}')
            if j>=args.warmup and (j-args.warmup)%args.stride==0:
                for key in INPUTS:
                    collected[key].append(feed[key][0].astype(np.uint8 if key.endswith('imgs') else np.float32))
                ids.append(sid); refs.append(raw)
                metadata.append(dict(sequence_id=sid,video=str(video),route=str(route),frame=start+j,
                                     frame_id=int(idx[start+j]['frame_id']),
                                     capture_gap_ms=(timestamp-int(idx[start+j-1]['timestamp']))/1e6 if start+j else None,
                                     timestamp_ns=timestamp,speed_kph=float(ctrl[ci]['speed_kph']) if fresh else None,
                                     desire=desire,desire_pulse=int(pulse.argmax()) if pulse.any() else 0,
                                     rpy_rad=rpy,mean_luma=float(current[0,:4].mean()),
                                     std_luma=float(current[0,:4].std())))
            previous=current; prev_desire=desire; state=raw[-512:][None].copy()
        print(f'clip {sid+1}/{len(manifest)} {video}: {len(metadata)} samples',flush=True)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    arrays={key:np.stack(vals) for key,vals in collected.items()}
    # Only mark actual contiguous clips as recurrent sequences.
    if args.stride == 1: arrays['sequence_id']=np.array(ids,np.int32)
    np.savez_compressed(args.out,**arrays)
    np.save(args.out.with_suffix('.reference.npy'),np.stack(refs))
    args.out.with_suffix('.metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    args.out.with_suffix('.provenance.json').write_text(json.dumps(dict(
        manifest=str(args.manifest),source_model=str(args.model),warmup=args.warmup,
        frames=args.frames,stride=args.stride,samples=len(metadata),
        preprocessing='production scalar fixed12, decoded NV12, route snapshot pitch/yaw, roll=0',
        limitation='HEVC is lossy; online calibration may differ from route snapshot'),indent=2)+'\n')


if __name__ == '__main__': main()
