"""Checks that prevent invalid calibration and misleading quantization reports."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from compile_supercombo_nncase import npz_calibration
from evaluate_quantization import compare, load_outputs, PROBS


class QuantizationToolsTest(unittest.TestCase):
    def test_common_logit_offset_does_not_count_as_error(self):
        rng = np.random.default_rng(1)
        r = rng.normal(size=(2,6524)).astype(np.float32)
        c = r.copy(); c[:,PROBS] += 320
        m = compare(r,c)
        self.assertEqual(m['plan_matches'],2)
        self.assertLess(m['plan_logits_delta']['max'], 4e-5)
        self.assertEqual(m['non_plan_logits']['max'],0)

    def test_plan_selection_changes_physical_path(self):
        r=np.zeros((1,6524),np.float32); r[0,PROBS]=[1,0,-1,-2,-3]
        r[0,991+1:991+495:15]=2
        c=r.copy(); c[0,PROBS]=[0,1,-1,-2,-3]
        m=compare(r,c)
        self.assertEqual(m['plan_matches'],0)
        self.assertEqual(m['selected_path_y_m']['mae'],2)
        self.assertEqual(m['same_hypothesis_path_y_m']['max'],0)

    def test_reject_nonfinite_output(self):
        r=np.zeros((1,6524),np.float32);c=r.copy();c[0,3]=np.nan
        with self.assertRaises(ValueError):compare(r,c)

    def test_calibration_rejects_lossy_cast_and_wrong_counts(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'input.npz'
            specs=[('image',(1,2),np.dtype(np.uint8))]
            for x in [np.array([[0.5,1]]),np.array([[0,256]]),np.array([[0,np.nan]])]:
                np.savez(p,image=x)
                with self.assertRaises(ValueError):npz_calibration(p,specs,1)
            np.savez(p,image=np.array([[0,255]],np.float32))
            self.assertEqual(npz_calibration(p,specs,1)[0][0].shape,(1,2))
            with self.assertRaises(ValueError):npz_calibration(p,specs,2)

    def test_reject_truncated_board_output(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'out.bin';p.write_bytes(b'SCODMP1\0')
            with self.assertRaises(ValueError):load_outputs(p)


if __name__ == '__main__':unittest.main()
