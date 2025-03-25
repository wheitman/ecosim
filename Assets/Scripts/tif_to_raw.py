from PIL import Image
import numpy
from matplotlib import pyplot as plt

im = Image.open('Assets\Scenes\schenley_farms\srtm.tif')
imarray = numpy.array(im)

plt.imshow(imarray, cmap='gray')
plt.show()

imarray.astype('int16').tofile("image.raw")