import numpy as np
from six.moves import urllib
import cv2

def url_to_image(url):
    resp = urllib.request.urlopen(url)
    print('got url')
    image = np.asarray(bytearray(resp.read()), dtype = "uint8")
    print('got byte')
    image = cv2.imdecode(image,cv2.IMREAD_COLOR)

    return image

image = url_to_image('http://172.20.10.9/capture')
cv2.imshow("Image",image)
cv2.waitKey(0)
cv2.destroyAllWindows()
