# Tufting CNC: GCode Generator

This script, `img2gcode.py` generates gcode for any CNC machine modified to tuft rugs! 

<p align="center">
<img src=".github/sun.jpg" height=300>
</p>

You can specify the dimensions of the underlying plotter. For the example below, generated from the image above, the dimensions for [the Line-us drawing arm](https://github.com/Line-us/Line-us-Programming/blob/master/Documentation/LineUsDrawingArea.pdf)).

<p align="center">
<img src=".github/output.gif" height=300>
</p>

## Install

You need the following requirements:

- python (3.6+)
- potrace
- ffmpeg
- imagemagick
- autotrace


### Windows 

On Windows you can install with [scoop](https://scoop.sh/). Note that you must use `python37` to install Python because one of the libraries (simplification) on Windows is incompatible with Python3.8.

	scoop install potrace ffmpeg imagemagick python37

Once installed, you can install the required Python packages with pip:

	python37 -m pip install click loguru numpy simplification svgpathtools svgwrite tqdm pillow svg.path

You also need to download `autotrace` from [here](https://github.com/scottvr/autotrace-win64-binaries/tree/master/bin) and put it in your path.

### Linux 

On Linux you can install with apt:
	
	sudo apt install potrace ffmpeg imagemagick python3

Once installed, you can install the required Python packages with pip:

	python3 -m pip install click loguru numpy simplification svgpathtools svgwrite tqdm pillow svg.path

You also need to build `autotrace` since newer distributions of Ubuntu don't include it:


```bash
sudo apt update
sudo apt install intltool imagemagick libmagickcore-dev pstoedit libpstoedit-dev autopoint

git clone https://github.com/autotrace/autotrace.git
cd autotrace

./autogen.sh
LD_LIBRARY_PATH=/usr/local/lib ./configure --prefix=/usr
make
sudo make install
```

## Run

You can just download and run the script directly:

	wget https://raw.githubusercontent.com/schollz/img2gcode/master/img2gcode.py
	python3 img2gcode.py --file image.jpg --animate --simplify 2 --threshold 80

Try changing `--simplify` from 1 to 10 to decrease the number of lines. You can also increase `--threshold` if you aren't getting the whole picture. You can also add an option `--centerline` to get a better skeleton.

After it runs it should create a folder `image.jpg.drawbot`. In that folder there are a number of files:

- `image.gc`: contains the final GCode coordinates
- `final.svg` / `final.png`: contains the final image after simplification and transforming
- `animation.gif`: contains the animation showing the drawing process
- `potrace.svg`: shows the untransformed SVG after skeleton
- `skeleton*`: shows the skeleton transformations from the image (if `--centerline` was added)
- `thresholded.png`: shows how the image looks after initial thresholding

## Notes

### Creating text with imagemagick

(Keep less <= 16 characters)

```bash
convert -size 2000x1125 xc:white white.png
convert white.png -fill black -pointsize 100 -gravity northwest -annotate +50+50 "Some cool message\non the left side\n\n:)\n" -annotate +1050+300 "Some cool message\non the right side" test.png
python3 img2gcode.py --simplify 5 --threshold 80 --file test.png --animate --centerline
python3 img2gcode.py --autotrace --file test.png  --animate --threshold 80 --simplify 1
```

Using autotrace:

```bash
convert -size 2000x1125 xc:white white.png
convert white.png -fill black -pointsize 100 -gravity northwest -annotate +50+50 "Some cool message\non the left side\n\n:)\n" -annotate +1050+300 "Some cool message\non the right side" test.png
convert test.png -rotate 90 test2.tga
autotrace -output-file test2.svg --output-format svg --centerline test2.tga
cat test2.svg | grep "svg\|000000\|xml" > test3.svg
python3 img2gcode.py --autotrace --file test3.svg
```

Using autotrace

```
convert c2.jpg -resize 2000x1125 -background White -gravity center -extent 2000x1125 -threshold 90% -rotate 90 test2.tga
autotrace -output-file test2.svg --output-format svg --centerline test2.tga
```

## License

MIT