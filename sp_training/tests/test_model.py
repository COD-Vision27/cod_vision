import unittest
from pathlib import Path
import numpy as np
import torch
from armor_training.model import build_model
from armor_training.loss import armor_loss
from armor_training.train import metric_counts,summarize

BASE=Path(__file__).resolve().parents[1]/"assets/base_model/yolov5.xml"


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): torch.set_num_threads(2)
    def test_exact_graph_batch_gradient_and_empty_frame(self):
        m=build_model(BASE); self.assertEqual(sum(p.numel() for p in m.parameters()),1232796)
        before=next(m.parameters()).detach().clone(); optimizer=torch.optim.SGD(m.parameters(),lr=1e-5)
        output=m(torch.rand(2,3,640,640)); self.assertEqual(tuple(output.shape),(2,25200,22))
        targets=[torch.tensor([[0,3,100,100,100,140,180,140,180,100]],dtype=torch.float32),torch.empty((0,10))]
        loss,stats=armor_loss(output,targets); self.assertTrue(torch.isfinite(loss));loss.backward()
        self.assertTrue(all(p.grad is not None and torch.isfinite(p.grad).all() for p in m.parameters()))
        optimizer.step(); self.assertFalse(torch.equal(before,next(m.parameters()).detach()))
    def test_metrics_count_correct_and_wrong_class(self):
        rows=torch.zeros(1,25200,22); rows[:,:,8]=-50
        rows[0,0,:8]=torch.tensor([100,100,100,140,180,140,180,100]); rows[0,0,8]=10; rows[0,0,9]=10; rows[0,0,16]=10
        truth=[torch.tensor([[0,3,100,100,100,140,180,140,180,100]],dtype=torch.float32)]
        correct=summarize(metric_counts(rows,truth)); self.assertEqual(correct["precision"],1); self.assertEqual(correct["recall"],1)
        rows[0,0,17]=20; wrong=summarize(metric_counts(rows,truth)); self.assertEqual(wrong["recall"],0)
    def test_original_openvino_parity(self):
        from armor_training.export import verify_base
        result=verify_base(BASE); self.assertLess(max(x["max_abs_error"] for x in result["fp32_parity"]),.005)


if __name__=="__main__": unittest.main()
