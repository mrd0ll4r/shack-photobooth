# shack-photobooth

Photo booth built from a Raspberry Pi + camera module and a cheap 58mm thermal printer.

## Features

It can:
- Wait for various button presses
- Indicate "busy" and a countdown via 6 LEDs mounted around the camera
- Take multiple photos at different lighting conditions
- Convert one of those to grayscale, adding various overlays and watermarks, then print it
- Process some of them for high-resolution output
- Make those available via syncthing/.../

## Requirements

Software:
- Python >= 3.5 (which is the latest on Raspbian at the time of writing).
- Some system libraries (see below)
- `pipenv` for dependency management
- CUPS setup for cheap 58mm thermal printers (see below)

Hardware:
- Raspberry Pi
- Camera module (enable this up using `raspi-config`)
- We use a bunch of random bulbs and tubes for lighting, a few LEDs, and buttons; your setup may vary.

## Install

We print via CUPS, this [filter](https://github.com/klirichek/zj-58) worked for us.
Our printer is connected via.. USB serial? Unclear, but it shows up at `/dev/usb/lp0`.

System dependencies:
```
sudo apt install libjpeg-dev zlib1g-dev imagemagick
```

Python dependencies:
```
pipenv install
```

## Run

If everything worked:
```
pipenv run python3 Hcam.py
```

There's also a systemd service file in [dist/HCam.service](dist/HCam.service).