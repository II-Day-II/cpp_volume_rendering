from pathlib import Path
import subprocess

ImageFolder = Path("img")

AllImages = list(ImageFolder.glob("*.png"))
FirstFile = AllImages[0]
CompareFiles = AllImages[1:5]

SSIM = list()
for CmpFile in CompareFiles:
    OutFile = FirstFile.stem + "_" + CmpFile.stem + ".png"
    # REQUIRES IMAGEMAGICK.ORG
    res = subprocess.run(["magick", "compare", "-metric", "SSIM", str(FirstFile),  str(CmpFile), OutFile], capture_output=True, text=True)
    print(res.stderr)
    SSIM.append(res.stderr)

#~ print(SSIM)