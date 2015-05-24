import colorsys

def get_heat_colour(value, maxValue):
    minHue = 0
    maxHue = 0.7
    hue = maxHue - ((value / float(maxValue)) * (maxHue - minHue))
    return colorsys.hsv_to_rgb(hue, 1, 1)

def main():
    maxv = 9
    for i in range(0,maxv + 1):
        colour = get_heat_colour(i, maxv)

        print(i)
        print(colour[0] * 255, colour[1] * 255, colour[2] * 255)
        
        

if __name__ == '__main__':
    main()
