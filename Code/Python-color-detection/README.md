# Color Detection with Python and OpenCV

This project is a Python-based color detection tool that uses OpenCV to detect colors in an image.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [How It Works](#how-it-works)
- [License](#license)

## Features

- Detects the dominant colors in an image.
- Provides the color name and RGB values for the detected colors.
- Works with various image formats.

## Prerequisites

Before using this color detection tool, make sure you have the following prerequisites installed:

- Python (3.x recommended)
- OpenCV (Open Source Computer Vision Library)

You can install OpenCV using pip:

`pip install opencv-python`

## Instalations

* 1. Clone this repository to your local machine:
`git clone https://github.com/sathvik995/Python-color-detection.git`
* 2. Navigate to the project directory: 
`cd color-detection`

## Usage
* 1. Open a command prompt or terminal.
* 2.Navigate to the project directory:
   `cd /path/to/your/color-detection-directory`
* 3.Run the color detection script with your image file as an argument:
   `python color_detection.py -i path_to_your_image.jpg`


## How It Works
The color detection tool uses OpenCV to read the input image and allows you to select colors by double-clicking on the image. The tool then displays the color name and RGB values for the selected colors. It uses a CSV file containing color names and corresponding RGB values to identify the closest matching color for the selected pixel.

## License
This project is licensed under the MIT License. See the LICENSE file for details.






















