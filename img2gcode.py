from loguru import logger as log
import click

import cv2
import numpy as np
from matplotlib import pyplot as plt

import random
import ntpath
import os
import subprocess
from shutil import copyfile

from gray2gcode import gray2gcode

@click.command()
@click.option("--file", prompt="image in?", help="svg to process")
@click.option("--folder", default="img_output", help="folder to output into")
@click.option("--colours", default=4, help="number of colours (yarn) ")
@click.option("--width", default=4000, help="maximum bed width (x)")
@click.option("--height", default=4000, help="maximum bed height (y)")
@click.option("--depth", default=4.5, help="maximum depth (z)")
@click.option("--spacing", default=0.25, help="space between tufting lines (inches)")
@click.option("--seed", default=0, help="random seed")
def run(
    folder,
    file,
    colours,
    width,
    height,
    depth,
    spacing,
    seed,
):
    random.seed(seed)
    imconvert = "convert"
    if os.name == "nt":
        imconvert = "imconvert"

    if folder != ".":
        try:
            os.mkdir(folder)
        except:
            pass

    foldername = os.path.join(folder, ntpath.basename(file) + ".img2gcode")
    try:
        os.mkdir(foldername)
    except:
        pass

    copyfile(file, os.path.join(foldername, ntpath.basename(file)))

    log.info(f"working in {foldername}")
    log.debug(foldername)
    os.chdir(foldername)
    file = ntpath.basename(file)

    cmd = f"{imconvert} {file} +dither -colors {colours} -resize {width}x{height} reduce-colour-{colours}.png"
    log.debug(cmd)
    subprocess.run(cmd.split())

    img = cv2.imread(f"reduce-colour-{colours}.png")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([gray],[0],None,[256],[0,256])
    colors = np.where(hist>5000)

    img_number = 0
    for color in colors[0]:
        log.debug(f"working on color: {color}")
        split_image = img.copy()
        split_image[np.where(gray != color)] = 0
        cv2.imwrite(str(img_number)+".png",split_image)
        
        cmd = f"{imconvert} {img_number}.png -channel RGB -white-threshold 1% c-{img_number}.png"
        subprocess.run(cmd.split())

        fname = f"{os. getcwd()}/c-{img_number}.png"
        outputfile = gray2gcode(fname, depth, spacing)

        log.info(f"gcode: /nc_output/{outputfile}")
        img_number+=1

    plt.hist(gray.ravel(),256,[0,256])
    plt.savefig('plt')

    log.debug("done.")

if __name__ == "__main__":
    run()
