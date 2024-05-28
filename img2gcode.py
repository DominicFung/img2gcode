from loguru import logger as log
import click

import cv2
import numpy as np
from matplotlib import pyplot as plt

import math
import random
import hashlib
import ntpath
import os
import subprocess
import sys
from shutil import copyfile

from util import processAutotraceSVG, processSVG, animateProcess

@click.command()
@click.option("--file", prompt="image in?", help="svg to process")
@click.option("--folder", default=".", help="folder to output into")
@click.option("--colours", default=4, help="number of colours (yarn) ")
@click.option("--animate/--no-animate", default=False)
@click.option("--overwrite/--no-overwrite", default=True)
@click.option("--skeleton/--no-skeleton", default=False)
@click.option("--autotrace/--no-autotrace", default=False)
@click.option("--minimize/--no-minimize", default=True)
@click.option("--minx", default=650, help="minimum x")
@click.option("--maxx", default=1775, help="maximum x")
@click.option("--miny", default=-1000, help="minimum y")
@click.option("--maxy", default=1000, help="maximum y")
# @click.option("--minx", default=500, help="minimum x")
# @click.option("--maxx", default=1800, help="maximum x")
# @click.option("--miny", default=-1400, help="minimum y")
# @click.option("--maxy", default=1200, help="maximum y")
@click.option("--junctiondist", default=0, help="junction distance")
@click.option("--seed", default=0, help="random seed")
@click.option("--minpath", default=0, help="min path length")
@click.option("--merge", default=0, help="mege points closer than")
@click.option("--prune", default=0, help="amount of pruning of small things")
@click.option("--simplify", default=0, help="simplify level", type=float)
@click.option("--threshold", default=60, help="percent threshold (0-100)")
def run2(
    folder,
    autotrace,
    prune,
    skeleton,
    file,
    colours,
    simplify,
    overwrite,
    animate,
    minx,
    maxx,
    miny,
    maxy,
    threshold,
    minpath,
    merge,
    minimize,
    junctiondist,
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
    os.chdir(foldername)
    file = ntpath.basename(file)

    width = maxy - miny
    height = maxx - minx
    new_new_paths_flat = []
    bounds = [minx, maxx, miny, maxy]

    cmd = f"{imconvert} {file} +dither -colors {colours} reduce-colour-{colours}.png"
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
        
        img_number+=1

    plt.hist(gray.ravel(),256,[0,256])
    plt.savefig('plt')

    

    # cmd = f"{imconvert} {file} -resize {width}x{height} -background White -gravity center -extent {width}x{height} -threshold {threshold}%% -rotate 90 thresholded.png"



@click.command()
@click.option("--file", prompt="image in?", help="svg to process")
@click.option("--folder", default=".", help="folder to output into")
@click.option("--animate/--no-animate", default=False)
@click.option("--overwrite/--no-overwrite", default=True)
@click.option("--skeleton/--no-skeleton", default=False)
@click.option("--autotrace/--no-autotrace", default=False)
@click.option("--minimize/--no-minimize", default=True)
@click.option("--minx", default=650, help="minimum x")
@click.option("--maxx", default=1775, help="maximum x")
@click.option("--miny", default=-1000, help="minimum y")
@click.option("--maxy", default=1000, help="maximum y")
# @click.option("--minx", default=500, help="minimum x")
# @click.option("--maxx", default=1800, help="maximum x")
# @click.option("--miny", default=-1400, help="minimum y")
# @click.option("--maxy", default=1200, help="maximum y")
@click.option("--junctiondist", default=0, help="junction distance")
@click.option("--seed", default=0, help="random seed")
@click.option("--minpath", default=0, help="min path length")
@click.option("--merge", default=0, help="mege points closer than")
@click.option("--prune", default=0, help="amount of pruning of small things")
@click.option("--simplify", default=0, help="simplify level", type=float)
@click.option("--threshold", default=60, help="percent threshold (0-100)")
def run(
    folder,
    autotrace,
    prune,
    skeleton,
    file,
    simplify,
    overwrite,
    animate,
    minx,
    maxx,
    miny,
    maxy,
    threshold,
    minpath,
    merge,
    minimize,
    junctiondist,
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
    os.chdir(foldername)
    file = ntpath.basename(file)

    width = maxy - miny
    height = maxx - minx
    new_new_paths_flat = []
    bounds = [minx, maxx, miny, maxy]
    if autotrace:
        log.debug("autotrace!")
        cmd = f"{imconvert} {file} -resize {width}x{height} -background White -gravity center -extent {width}x{height} -threshold {threshold}%% -rotate 90 thresholded.png"
        log.debug(cmd)
        subprocess.run(cmd.split())
        cmd = f"{imconvert} thresholded.png 1.tga"
        log.debug(cmd)
        subprocess.run(cmd.split())
        cmd = (
            f"autotrace -output-file potrace.svg --output-format svg --centerline 1.tga"
        )
        log.debug(cmd)
        subprocess.run(cmd.split())
        new_new_paths_flat = processAutotraceSVG(
            "potrace.svg",
            "final.svg",
            drawing_area=bounds,
            simplifylevel=simplify,
            minPathLength=minpath,
            mergeSize=merge,
            minimizeMoves=minimize,
            junction_distance=junctiondist,
        )
    elif not os.path.exists("potrace.svg") or overwrite:
        if skeleton:
            cmd = f"{imconvert} {file} -resize {width}x{height} -background White -gravity center -extent {width}x{height} -threshold {threshold}%% thresholded.png"
            log.debug(cmd)
            subprocess.run(cmd.split())

            cmd = f"{imconvert} thresholded.png -negate -morphology Thinning:-1 Skeleton skeleton.png"
            log.debug(cmd)
            subprocess.run(cmd.split())

            cmd = f"{imconvert} skeleton.png -negate skeleton_negate.png"
            log.debug(cmd)
            subprocess.run(cmd.split())

            cmd = f"{imconvert} skeleton_negate.png -rotate 90 skeleton_border.png"
            log.debug(cmd)
            subprocess.run(cmd.split())

            cmd = f"{imconvert} skeleton_border.png -flip skeleton_border_flip.bmp"
            log.debug(cmd)
            subprocess.run(cmd.split())

            cmd = f"potrace -b svg -o potrace.svg skeleton_border_flip.bmp"
            log.debug(cmd)
            subprocess.run(cmd.split())
        else:
            cmd = f"{imconvert} {file} -resize {width}x{height} -background White -gravity center -extent {width}x{height} -threshold {threshold}%% thresholded.png"
            log.debug(cmd)
            subprocess.run(cmd.split())

            cmd = f"{imconvert} thresholded.png -rotate 90 -flip  thresholded.bmp"
            log.debug(cmd)
            subprocess.run(cmd.split())

            cmd = f"potrace -b svg -o potrace.svg -n thresholded.bmp"
            log.debug(cmd)
            subprocess.run(cmd.split())
            os.remove("thresholded.bmp")

        new_new_paths_flat = processSVG(
            "potrace.svg",
            "final.svg",
            simplifylevel=simplify,
            pruneLittle=prune,
            drawing_area=[minx, maxx, miny, maxy],
        )

    cmd = f"{imconvert} final.svg -rotate 270 final.png"
    log.debug(cmd)
    subprocess.run(cmd.split())

    animatefile = ""
    if animate:
        animatefile = "1.gif"
        animateProcess(new_new_paths_flat, bounds, animatefile)
        cmd = f"{imconvert} 1.gif -rotate 270 animation.gif"
        log.debug(cmd)
        subprocess.run(cmd.split())
    # os.remove("1.gif")


if __name__ == "__main__":
    run2()
