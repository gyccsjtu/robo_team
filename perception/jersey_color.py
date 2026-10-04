"""Same-image shirt-color veto; no target positions or classifier relabeling."""
import math


CLASS_COLOR = {1: 'green', 3: 'white'}


def supported(color, fractions):
    """Insufficient crop data leaves the original detector in charge."""
    if fractions is None:
        return True
    expected = fractions[color]
    minimum = .08 if color == 'white' else .12
    other = max(value for name, value in fractions.items() if name != color)
    return expected >= minimum and not (other >= .20 and other >= 1.5*expected)


def torso_fractions(image, rectangle):
    import cv2
    if len(rectangle) != 4 or not all(math.isfinite(float(v)) for v in rectangle):
        return None
    x1,y1,x2,y2 = (float(v) for v in rectangle)
    width,height = x2-x1,y2-y1
    if width <= 0 or height <= 0:
        return None
    xa,xb = max(0,int(x1+.25*width)),min(image.shape[1],int(x2-.25*width))
    ya,yb = max(0,int(y1+.20*height)),min(image.shape[0],int(y1+.60*height))
    if xb <= xa or yb <= ya or (xb-xa)*(yb-ya) < 12:
        return None
    hue,saturation,value = cv2.split(cv2.cvtColor(image[ya:yb,xa:xb],cv2.COLOR_BGR2HSV))
    masks = {
        'red': ((hue<12)|(hue>168))&(saturation>80)&(value>45),
        'green': (hue>=35)&(hue<=90)&(saturation>65)&(value>40),
        'blue': (hue>90)&(hue<140)&(saturation>65)&(value>40),
        'white': (saturation<65)&(value>160),
        'brown': (hue>=8)&(hue<35)&(saturation>65)&(value>35)&(value<190),
    }
    return {name:float(mask.mean()) for name,mask in masks.items()}


def filter_boxes(image, boxes):
    output = []
    for box in boxes:
        color = CLASS_COLOR.get(int(box.cls))
        if color is None or supported(color,torso_fractions(image,box.xyxy[0])):
            output.append(box)
    return output
