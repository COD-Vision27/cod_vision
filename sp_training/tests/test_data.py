import json
import shutil
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from armor_training.contract import label_line,parse_label,split_for,decode_output,write_sample
from armor_training.dataset import validate_dataset,prepare,import_frames
from armor_training.synthetic import generate,load_asset

ROOT=Path(__file__).resolve().parents[1]


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.obj={"color_id":0,"number_id":3,"points":[[.2,.2],[.2,.4],[.5,.4],[.5,.2]]}
    def test_label_and_cpp_point_reorder(self):
        self.assertEqual(len(label_line(self.obj).split()),13)
        self.assertEqual(parse_label(label_line(self.obj)),self.obj)
        raw=np.zeros((1,22),np.float32); raw[0,:8]=(np.asarray(self.obj["points"])*640).flatten()
        raw[0,8]=10; raw[0,9]=2; raw[0,16]=3
        out=decode_output(raw,scale=2,offset=(10,20))[0]
        expected=np.asarray(self.obj["points"])[[0,3,2,1]]*320+[10,20]
        np.testing.assert_allclose(out["points"],expected,atol=1e-5)
        self.assertEqual((out["color_id"],out["number_id"]),(0,3))
    def test_reject_malformed_annotations(self):
        for points in ([[.2,.2],[.5,.2],[.5,.4],[.2,.4]],[[.2,.2],[.2,.4],[.5,.4],[float('nan'),.2]]):
            with self.assertRaises(ValueError): label_line({**self.obj,"points":points})
        with self.assertRaises(ValueError): label_line({**self.obj,"visible":[True,False,True,True]})
        with self.assertRaises(ValueError): parse_label("27 .2 .3 .1 .1 0 0 0 0 0 0 0 0")
    def test_verified_asset_needs_reference(self):
        with tempfile.TemporaryDirectory() as d:
            a=json.loads((ROOT/"configs/placeholder.json").read_text()); a["status"]="verified"
            path=Path(d)/"asset.json"; path.write_text(json.dumps(a))
            with self.assertRaisesRegex(ValueError,"reference texture or mesh"): load_asset(path)
    def test_fixed_texture_has_one_class(self):
        with tempfile.TemporaryDirectory() as d:
            a=json.loads((ROOT/"configs/placeholder.json").read_text()); a["texture"]="sprite.png"
            cv2.imwrite(str(Path(d)/"sprite.png"),np.zeros((32,32,4),np.uint8))
            path=Path(d)/"asset.json"; path.write_text(json.dumps(a))
            with self.assertRaisesRegex(ValueError,"exactly one"): load_asset(path)
            a["allowed_colors"]=[0];a["allowed_numbers"]=[3];a["status"]="verified";path.write_text(json.dumps(a))
            self.assertEqual(load_asset(path)["allowed_numbers"],[3])
    def test_scene_split_is_stable(self):
        self.assertEqual(split_for("sequence-001"),split_for("sequence-001"))
        self.assertEqual({split_for(f"scene-{i}") for i in range(100)},{"train","val","test"})
    def test_generate_validate_move_and_corruption(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)/"data"; generate(root,ROOT/"configs/placeholder.json",count=32,width=320,height=240)
            with self.assertRaisesRegex(ValueError,"placeholder"): validate_dataset(root)
            records,report=validate_dataset(root,True); self.assertEqual(report["samples"],32)
            prepare(root,True); moved=Path(d)/"moved"; shutil.move(str(root),moved)
            self.assertEqual(validate_dataset(moved,True)[1],report)
            with self.assertRaises(FileExistsError): generate(moved,ROOT/"configs/placeholder.json",count=1,width=320,height=240)
            label=moved/records[0]["label"]; label.write_text("0 0 0\n")
            with self.assertRaises(ValueError): validate_dataset(moved,True)
    def test_same_frame_import_guard(self):
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/"src"; src.mkdir(); cv2.imwrite(str(src/"rgb.png"),np.zeros((64,64,3),np.uint8))
            record={"schema":"sp-armor-v5-v1","sample_id":"frame1","scene_id":"clip1","frame_id":1,"image_frame_id":1,"annotation_frame_id":2,
                    "image":"rgb.png","objects":[self.obj],"provenance":{"asset_id":"test","asset_status":"placeholder"}}
            (src/"frame.json").write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError,"frame mismatch"): import_frames(src,Path(d)/"target")
            record["annotation_frame_id"]=1; (src/"frame.json").write_text(json.dumps(record))
            self.assertEqual(import_frames(src,Path(d)/"target"),{"imported":1})
    def test_orphans_and_path_escape_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)/"data"; write_sample(root,"one","scene",np.zeros((64,64,3),np.uint8),[],{"asset_id":"test","asset_status":"placeholder"})
            record=next((root/"meta").glob("*/*.json")); value=json.loads(record.read_text()); value["image"]="../../escape.png"; record.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,"escapes"): validate_dataset(root,True)


if __name__=="__main__": unittest.main()
