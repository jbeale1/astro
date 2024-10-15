# track center of sun and other stats, in set of images
# 14-Oct-2024 J.Beale

# Standard imports
import cv2
import numpy as np
import os
import math

#inDir = r"C:\Users\beale\Desktop\SharpCap Captures\2024-10-13\Sun\pipp_20241013_124607\12_34_32"
#inDir = r"C:\Users\beale\Desktop\SharpCap Captures\2024-10-13\Sun3\pipp_20241013_222641\13_57_06"
inDir = r"C:\Users\beale\Desktop\SharpCap Captures\2024-10-14\Sun4\pipp_20241014_181643\12_41_57"

outFile = r"C:\Users\beale\Documents\astro\SunPos4.csv"

header = "n,count,cx,cy,diam,dStd,area,meanVal,stdVal,mV2,sV2,rUD,rRL" # column headers for CSV file out
EOL="\n"


def dist(p1, p2):
    dist = math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) )
    return dist

# return upper and lower half of contour
def UL_contour(cnt):
    uhC = []
    lhC = []
    x, y, w, h = cv2.boundingRect(cnt)
    midY = (y + h//2)
    for point in cnt:
        if point[0][1] < midY:  # Keep points above the middle of the bounding box
            uhC.append(point)
        else:
            lhC.append(point)

    uhC = np.array(uhC)
    lhC = np.array(lhC)
    return (uhC, lhC)

# return left and right half of contour
def RL_contour(cnt):
    rhC = []
    lhC = []
    x, y, w, h = cv2.boundingRect(cnt)    
    midX = (x + w//2)
    for point in cnt:
        if point[0][0] < midX:  # Keep points above the middle of the bounding box
            lhC.append(point)
        else:
            rhC.append(point)

    rhC = np.array(rhC)
    lhC = np.array(lhC)
    return (rhC, lhC)


# =============================================
# main program starts here

fout = open(outFile, "w")
fout.write(header+EOL)

#for pnum in range(916):
for pnum in range(810):
#for pnum in range(1):
    idx = pnum+1
    # 12_34_32_f001.png
    #f1 = ("12_34_32_f%03d.png" % idx)
    #f1 = ("13_57_06_f%03d.png" % idx)
    f1 = ("12_41_57_f%03d.png" % idx)
    
    fname1 = os.path.join(inDir, f1)

    imRaw = cv2.imread(fname1, cv2.IMREAD_GRAYSCALE)
    if (imRaw is None):
        print("Unable to load %s" % fname1)
        break
    imSmall = cv2.resize(imRaw, (0,0), fx=0.2, fy=0.2, interpolation = cv2.INTER_AREA)
    imSmallBlur = cv2.blur(imSmall, (9, 9))
    maxVal = imSmallBlur.max()
    imSmall1 = cv2.convertScaleAbs(imSmall, alpha=(255.0/maxVal), beta=0)    
    #cv2.imshow('Rescale', imSmall1)
    #cv2.waitKey(1)


    imGray = cv2.convertScaleAbs(imRaw, alpha=(255.0/maxVal), beta=0)    

    im = cv2.cvtColor(imGray, cv2.COLOR_GRAY2BGR)
    thresh = cv2.threshold(imGray, 110, 255, cv2.THRESH_BINARY)[1]  

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    s = ("%d, %d " % (idx, len(contours)))  # number of detected objects
    print(s,end="")
    fout.write(s)

    for contour in contours:        
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cX = (M['m10'] / M['m00'])
            cY = (M['m01'] / M['m00'])
            # cv2.circle(im, (int(cX), int(cY)), 10, (0, 0, 255), 2)
        else:
            continue
        p1 = (cX, cY)
        x, y, w, h = cv2.boundingRect(contour)
        # cv2.rectangle(im, (x, y), (x + w, y + h), (255, 0, 0), 2)
        xc = x+w/2
        yc = y+h/2
        area = cv2.contourArea(contour)
        if (area < 450000): # not what we're looking for
        #if (area < 100000): # not what we're looking for
            continue

        #perimeter = cv2.arcLength(contour, True)
        #print(perimeter)

        rSum = 0
        points = 0
        dList = []
        for point in contour:
            p2 = point[0]
            r1 = dist(p1,p2)
            dList.append(r1)
        radii = np.array(dList, dtype=np.float32)            
        diam = 2 * radii.mean()
        dStd = 2 * radii.std()

        mask = np.zeros(imGray.shape, np.uint8)
        cv2.drawContours(mask, [contour], 0, 255, -1)
        meanVal,stdVal = cv2.meanStdDev(imGray, mask=mask) # Calculate mean pixel value inside the contour

        scale_factor = 0.8
        scaled_contour = [np.int32(((point-p1) * scale_factor)+p1) for point in contour]
        scaled_contour = np.array(scaled_contour)

        mask = np.zeros(imGray.shape, np.uint8)
        cv2.drawContours(mask, [scaled_contour], 0, 255, -1)
        mV2,sV2 = cv2.meanStdDev(imGray, mask=mask) # Calculate mean pixel value inside the contour

        uh,dh = UL_contour(scaled_contour)  # get upper and lower half contours

        mask1 = np.zeros(imGray.shape, np.uint8)
        cv2.drawContours(mask1, [uh], 0, 255, -1)
        mU = cv2.mean(imGray, mask=mask1)[0] # Calculate mean pixel value inside the contour

        mask2 = np.zeros(imGray.shape, np.uint8)
        cv2.drawContours(mask2, [dh], 0, 255, -1)
        mD = cv2.mean(imGray, mask=mask2)[0] # Calculate mean pixel value inside the contour
        ratioUL = mU/mD  # upper/lower ratio of brightness

        rh,lh = RL_contour(scaled_contour)  # get right and left-half contours

        #p0 = cv2.arcLength(scaled_contour, True)
        #p1 = cv2.arcLength(rh, True)
        #p2 = cv2.arcLength(lh, True)
        #print(p0,p1,p2)

        mask3 = np.zeros(imGray.shape, np.uint8)
        cv2.drawContours(mask3, [rh], 0, 255, -1)
        mR = cv2.mean(imGray, mask=mask3)[0] # Calculate mean pixel value inside the contour

        mask4 = np.zeros(imGray.shape, np.uint8)
        cv2.drawContours(mask4, [lh], 0, 255, -1)
        mL = cv2.mean(imGray, mask=mask4)[0] # Calculate mean pixel value inside the contour
        ratioRL = mR/mL  # right/left ratio of brightness


        s = (", %.3f,%.3f,%.3f,%.4f, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f" % 
             (cX,cY,diam,dStd, area, meanVal, stdVal, mV2, sV2, ratioUL, ratioRL))
        print(s,end="")
        fout.write(s)
        #cv2.drawContours(im, contours, -1, (0, 255, 0), 2)


    print()
    fout.write(EOL)

    #cv2.imshow('Contours', im)
    #cv2.waitKey(0)
    # cv2.destroyAllWindows()
fout.close()
