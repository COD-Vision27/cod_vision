#!/usr/bin/env python3
"""Create a self-contained server bundle; excludes local environments and generated runs."""
import argparse
import hashlib
import json
import tarfile
from pathlib import Path


def main():
    p=argparse.ArgumentParser(); p.add_argument("--output",required=True); p.add_argument("--data"); p.add_argument("--checkpoint"); p.add_argument("--allow-placeholder",action="store_true"); a=p.parse_args()
    root=Path(__file__).resolve().parent; destination=Path(a.output).resolve()
    if destination.exists(): raise FileExistsError(destination)
    ignored={".venv",".tools","__pycache__",".git","data","runs","exports"}; files=[]
    for source in sorted(root.rglob("*")):
        relative=source.relative_to(root)
        if any(part in ignored or part.endswith(".egg-info") for part in relative.parts) or source.suffix in (".pyc",".tar",".gz"): continue
        if source.is_symlink(): raise ValueError(f"bundle rejects symlinks: {source}")
        if source.is_file(): files.append((source,"sp_training/"+relative.as_posix()))
    if a.data:
        from armor_training.dataset import validate_dataset
        dataset=Path(a.data).resolve(); validate_dataset(dataset,a.allow_placeholder)
        if destination.is_relative_to(dataset): raise ValueError("bundle output must be outside dataset")
        for source in sorted(dataset.rglob("*")):
            if source.is_symlink(): raise ValueError("dataset symlinks are not portable")
            if source.is_file(): files.append((source,"sp_training/data/armor/"+source.relative_to(dataset).as_posix()))
    if a.checkpoint:
        source=Path(a.checkpoint).resolve()
        if source.is_symlink() or not source.is_file(): raise ValueError("checkpoint must be a regular file")
        files.append((source,"sp_training/runs/transferred/last.pt"))
    destination.parent.mkdir(parents=True,exist_ok=True)
    manifest={name:hashlib.sha256(path.read_bytes()).hexdigest() for path,name in files}
    with tarfile.open(destination,"w:gz") as archive:
        for source,name in files: archive.add(source,arcname=name,recursive=False)
        import io
        content=(json.dumps(manifest,indent=2)+"\n").encode(); info=tarfile.TarInfo("SHA256SUMS.json"); info.size=len(content); archive.addfile(info,io.BytesIO(content))
    print(json.dumps({"bundle":str(destination),"files":len(files),"bytes":destination.stat().st_size},indent=2))


if __name__=="__main__": main()
