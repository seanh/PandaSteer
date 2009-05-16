# perlin.py -- example/utility functions for Panda3D's Perlin2 class.

# Panda3d imports
from pandac.PandaModules import Vec2
from pandac.PandaModules import PerlinNoise, PerlinNoise2, PerlinNoise3
from pandac.PandaModules import PNMImage,Filename
# Python imports
import math

def greenNoise(imgSize=(32,32),scale=0.25):
    """Return a PNMImage of the given size containing Perlin noise at the given
    scale in the green colour values of the image pixels."""

    # Initialuse the PNMImage object
    img = PNMImage(*imgSize)

    # Initalise PerlinNoise2 object
    noise = PerlinNoise2()
    noise.setScale(scale)

    # Fill in the pixels of img, setting the red and blue values of each pixel
    # to 0 and the green value to Perlin noise.
    for x in xrange(imgSize[0]):
        for y in xrange(imgSize[1]):
            img.setRed(x,y,0)
            img.setBlue(x,y,0)
            pos = Vec2(1.0/32*x, 1.0/32*y)
            img.setGreen(x,y,(noise(pos)+1.0)/2.0)

    return img

def grassTexture(imgSize=(256,256)):
    """Return a green, 'grassy' texture (PNMImage) of the given image size,
    produced using 2D Perlin noise."""

    # Initialuse the PNMImage object
    img = PNMImage(*imgSize)

    # Initalise 4 PerlinNoise2 objects to produce noise at different scales
    noise1 = PerlinNoise2()
    noise1.setScale(2.0)
    noise2 = PerlinNoise2()
    noise2.setScale(5.0)
    noise3 = PerlinNoise2()
    noise3.setScale(0.25)
    noise4 = PerlinNoise2()
    noise4.setScale(0.125)

    # For each pixel in the image, set the red and blue values of the pixel to
    # constant values, and set the green value of the pixel using all 4
    # PerlinNoise2 objects.
    red = 0.125 # Colour values in PNMImage are doubles in the range [0.0,1.0]
    blue = 0.0625
    for x in xrange(imgSize[0]):
        for y in xrange(imgSize[1]):
            img.setRed(x,y,red)
            img.setBlue(x,y,blue)
            pos = Vec2(1.0/32*x, 1.0/32*y)
            img.setGreen(x,y,(0.5 + noise1(pos)*0.0625 + noise2(pos)*0.0625 +
                              noise3(pos)*0.125 + noise4(pos)*0.0625))

    return img

if __name__ == "__main__":

    # Create a grass texture and save it to file    
    grass = grassTexture()        
    grass.write(Filename('grass.png'))
    
    # Create some noise meant to be used for placing trees on terrain, and save
    # it to file
    tree = greenNoise(imgSize=(32,32),scale=0.25)
    tree.write(Filename('trees.png'))
