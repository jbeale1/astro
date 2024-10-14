# track center of sun in set of images
# 13-Oct-2024 J.Beale

# Standard imports
import cv2
import numpy as np
import os
import math

#inDir = r"C:\Users\beale\Desktop\SharpCap Captures\2024-10-13\Sun\pipp_20241013_124607\12_34_32"
inDir = r"C:\Users\beale\Desktop\SharpCap Captures\2024-10-13\Sun3\pipp_20241013_222641\13_57_06"

outFile = r"C:\Users\beale\Documents\astro\SunPos3.csv"

header = "n,count,cx,cy,diam,area" # column headers for CSV file out
EOL="\n"


def dist(p1, p2):
    dist = math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) )
    return dist

# =============================================
# main program starts here

fout = open(outFile, "w")
fout.write(header+EOL)

#for pnum in range(916):
for pnum in range(999):
#for pnum in range(1):
    idx = pnum+1
    # 12_34_32_f001.png
    #f1 = ("12_34_32_f%03d.png" % idx)
    f1 = ("13_57_06_f%03d.png" % idx)
    fname1 = os.path.join(inDir, f1)

    imRaw = cv2.imread(fname1, cv2.IMREAD_GRAYSCALE)
    if (imRaw is None):
        print("Unable to load %s" % fname1)
        break
    # im_gray = cv2.resize(imRaw, (0,0), fx=0.5, fy=0.5, interpolation = cv2.INTER_AREA)     
    im_gray = imRaw

    im = cv2.cvtColor(im_gray, cv2.COLOR_GRAY2BGR)
    thresh = cv2.threshold(im_gray, 60, 255, cv2.THRESH_BINARY)[1]  

    """
            # Detect circles
    blurred = cv2.GaussianBlur(im_gray, (5, 5), 0)
    blurBig = cv2.resize(imRaw, (0,0), fx=2, fy=2)
    circles = cv2.HoughCircles(blurBig, cv2.HOUGH_GRADIENT, 1, 200,
                            param1=50, param2=30, minRadius=760, maxRadius=770) # was 50,30,350,400  then 50,80,x1,x2
    if circles is not None:
        circles = np.float32(np.around(circles))
        for i in circles[0, :]:
            center = (i[0]/2, i[1]/2)
            radius = i[2]
            #diameter = 2 * radius
            diameter = radius # because we scaled up 2x
            # cv2.circle(im, center, radius, (0, 255, 0), 2)
            print("Circle:", diameter, center)
    """

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    s = ("%d, %d " % (idx, len(contours)))
    print(s,end="")
    fout.write(s)
    for contour in contours:        
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cX = (M['m10'] / M['m00'])
            cY = (M['m01'] / M['m00'])
            # cv2.circle(im, (int(cX), int(cY)), 10, (0, 0, 255), 2)

        p1 = (cX, cY)
        x, y, w, h = cv2.boundingRect(contour)
        # cv2.rectangle(im, (x, y), (x + w, y + h), (255, 0, 0), 2)
        xc = x+w/2
        yc = y+h/2
        area = cv2.contourArea(contour)

        # Create a point at (x=10, y=20)
        # point = cv2.Point(10, 20)
        rSum = 0
        points = 0
        for point in contour:
            p2 = point[0]
            rSum += dist(p1,p2)
            points += 1
            # print("%.3f" % dist(p1,p2), end=", ")
        radius = rSum / points
        diam = radius*2
        s = (", %.3f,%.3f,%.3f, %d" % (cX,cY,diam,area))
        print(s,end="")
        fout.write(s)
        #cv2.drawContours(im, contours, -1, (0, 255, 0), 2)
 
    print()
    fout.write(EOL)

    #cv2.imshow('Contours', im)
    #cv2.waitKey(0)
    # cv2.destroyAllWindows()
fout.close()
