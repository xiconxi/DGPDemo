import glob
import os
import concurrent.futures

def weight_cvt(file):
    print(file)
    img_name = os.path.basename(file).replace(".bmp","")
    os.system("rm -rf "+img_name)
    os.system("mkdir -p "+img_name)
    os.system("./games102_hw8 "+file+" "+img_name+"/img > "+img_name+"/log.txt")
    os.system("ffmpeg -i "+img_name+"/img%d.svg -r 5 videos/"+img_name+".mp4 -y")

if __name__ == "__main__":
    os.system("mkdir -p videos")
    with concurrent.futures.ProcessPoolExecutor() as executor:
        for file in executor.map(weight_cvt, list(glob.glob("../data/bmp/*.bmp"))):
            pass

