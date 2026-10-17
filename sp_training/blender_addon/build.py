"""Assemble the development add-on archive without importing Blender or running rendering."""
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED


def main():
    root=Path(__file__).resolve().parent
    target=root/"sp_armor_labels.building.zip"
    with ZipFile(target,"w",ZIP_DEFLATED) as archive:
        for source in sorted((root/"sp_armor_labels").glob("*.py")):
            archive.write(source,"sp_armor_labels/"+source.name)
    print(target)


if __name__=="__main__":main()
