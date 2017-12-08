#TO RUN: Change path (line 35) to your own video path
#Recommended: change s values in lowg and upg (lines 48 and 49) for specific video because different cameras and lighting affect value of the grey road
#Change constants (lines 37-45) if adjustment is needed


#Improvements possible:
# 1. Using a quadratic function (2nd degree polynomial) for curved roads (I tried it, and it worked great sometimes, but worked very badly at other times)
# 2. Detect cars, because they mess up the road detection
# 3. Update for multi-lane roads
# 4. Use a perspective transform to make the analysis occur on a birds-eye view.
# 4cont. This would make it easier to detect the road and lines because they would be vertical.
# 4cont. I think you would need to know the make of the camera, and set it up based on
# 4cont. how the camera warps images
# 5. Detect when a road is a side road or part of main road

import cv2
import numpy as np

def cropFrame(frame):
    (h, w) = frame.shape[:2]
    xrange = (70.0 / 360.0) * w  # 70 degrees124
    yrange = (50.0 / 180.0) * h  # 50 degrees
    xstart = w / 2.0 - xrange / 2.0
    ystart = h / 2.0 - yrange / 2.0

    return frame[ystart:ystart + yrange, xstart:xstart + xrange]  # cropped frame

def getVideo(path):
    return cv2.VideoCapture(path)




def main():
    path = "/Users/joshuayoung/Docs/GIT/RideOn/RoadFinder/Video/MotorcycleRide720.mp4"  #path

    #Constants
    dYInterceptMax = 5 #max change in y intercept between frames
    dSlopeMax = .05 #max change in line slope between frames
    dIntersectionXMax = 10 #max change in intersection in x direction
    dIntersectionYMax = 10 #max change in intersection in y direction
    slopeMinLeft  = .2 #min value of left line slope
    slopeMinRight = .3 #min value of right line slope
    slopeMax = .75 #max value of slope for both lines
    intersectionMinY = -10 #min y value of intersection

    # RECOMMEND CHANGING S (MIDDLE) VALUE BASED ON VIDEO
    lowg = np.array([0, 0 * 2.55, 0], dtype="uint8")
    upg = np.array([180, (0 + 30) * 2.55, 90 * 2.55], dtype="uint8")


    video = getVideo(path)

    lastIntersection = None
    lastMask = None
    frameCounter = 0
    lastFrameCounter = frameCounter
    last0Left = None
    lastSlopeLeft = None
    last0Right = None
    lastSlopeRight = None


    #play video
    while(video.isOpened()):


        ret, frame = video.read()
        frameCounter+=1
        cropped = cropFrame(frame)
        frame = cropped
        (height, width) = frame.shape[:2]
        frame[0:height/2-10, 0:] = (0,0,0)
        frame = frame[height/2-10:, 0:] #Make analysis only look at bottom half-10 of frame (where the road is)
        (h, w) = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # hsv = [hue, saturation, value]



        smask = cv2.inRange(hsv, lowg, upg)
        line = cv2.bitwise_and(frame, frame, mask=smask)

        g = cv2.cvtColor(line, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(g, (7, 7), 0)
        g= cv2.addWeighted(g, 1.5, blur, -.5, 0)
        (cnts, _) = cv2.findContours(g.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #find contours
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:1] #gets largest contour

        for c in cnts:
            xdataleft = []
            ydataleft = []
            xdataright = []
            ydataright = []

            peri = cv2.arcLength(c, True) #perimeter of the contour
            c = cv2.approxPolyDP(c, 0.001 * peri, True) #deletes very close points on contour

            pt = c
            last1 = 0
            last2 = 0
            k1 = 1
            k2 = 1
            for i in range(0,len(c)-2): #split points into left and right side of road
                if pt[i][0][1] < .8*h:
                    #No baseline points
                    if pt[i][0][0] < .3*w and pt[i][0][0] > 10:
                        if (pt[i][0][1]-pt[last1][0][1]>0 and pt[i][0][0]-pt[last1][0][0]<0) or k1==1:
                            #checks if point is in correct direction
                            #some points in wrong still accepted randomly (when k1==1)
                            #to account for randomness

                            xdataleft.append(pt[i][0][0])
                            ydataleft.append(pt[i][0][1])
                            last1 = i
                            k1=i%2

                    elif pt[i][0][0] > .67*w and pt[i][0][0] < w-5:
                        if (pt[i][0][1] - pt[last2][0][1] < 0 and pt[i][0][0] - pt[last2][0][0] < 0) or k2 == 1:
                            xdataright.append(pt[i][0][0])
                            ydataright.append(pt[i][0][1])

                            last2 = i
                            k2 = i % 2

            if (len(xdataleft)<5 or len(xdataright)<5):
                break

            zleft = np.polyfit(xdataleft, ydataleft, 1)
            fleft = np.poly1d(zleft) #get linear fit of left contour data
            zright = np.polyfit(xdataright, ydataright, 1)
            fright = np.poly1d(zright) #get linear fit of right contour data

            intersection = np.roots(fright - fleft) #find intersection of left and right lines
            intersectiony = np.multiply(fleft[1], intersection) + fleft[0]

            #if first frame
            if lastIntersection == None:
                lastIntersection=intersection
                lastIntersectiony = intersectiony
                last0Left= fleft[0]
                last0Right =fright[0]
                lastSlopeLeft = fleft[1]
                lastSlopeRight = fright[1]


            #statements to check if contour fit is bad, reuses last good one
            if (abs(fleft[1]) < slopeMinLeft or abs(fright[1]) < slopeMinRight or abs(fleft[1])> slopeMax or abs(fright[1]) > slopeMax) and frameCounter != 1: #if slope is too horizontal
                intersection = lastIntersection
                mask1 = lastMask
            elif intersectiony<intersectionMinY: #if intersection is too high
                intersection = lastIntersection
                mask1 = lastMask
            elif (intersection-lastIntersection>dIntersectionXMax or intersectiony-lastIntersection>dIntersectionYMax)\
                    and frameCounter-lastFrameCounter<5:
                #if intersection is too far from last intersection
                intersection=lastIntersection
                mask1 = lastMask
            elif (fleft[0]-last0Left>dYInterceptMax or fright[0]-last0Right>dYInterceptMax or abs(fleft[1]-lastSlopeLeft)>dSlopeMax or
                          abs(fright[1]-lastSlopeRight)>dSlopeMax) and frameCounter-lastFrameCounter<5:
                #check for large change in intercepts and slope
                intersection = lastIntersection
                mask1 = lastMask
            else: #find new mask if fit is good

                # get set of points for plotting line
                x1 = list(range(0,intersection,2))
                ptsLeft = list(zip(x1, np.multiply(fleft[1], x1) + fleft[0]))
                x2 = list(range(intersection, 250, 2))
                ptsRight = list(zip(x2, np.multiply(fright[1], x2) + fright[0]))

                mask1 = np.zeros(frame.shape, np.uint8)

                #attach bottom left and right corners to lists and merge together
                p = [(0, h)]
                p = np.concatenate((p, ptsLeft), 0)
                p = np.append(p,ptsRight,0)
                p = np.append(p,[(w, h)],0)


                cv2.fillPoly(mask1, pts=np.int32([p]), color=(255, 255, 255)) #creates mask

                #updates data
                lastIntersection = intersection
                lastIntersectiony = intersectiony
                lastMask = mask1
                lastFrameCounter = frameCounter
                last0Left = fleft[0]
                last0Right = fright[0]
                lastSlopeLeft = fleft[1]
                lastSlopeRight = fright[1]

        mask = np.zeros(cropped.shape, np.uint8) #set up mask to be size of original frame
        mask[height/2-10:, 0:] = mask1

        result = cv2.bitwise_and(cropped, mask) #black out non-road
        cv2.imshow('lines', result) #show result

        y = cv2.waitKey(1)
        if y == ord('p'): #p to pause
            cv2.waitKey(0)
        if y == ord('q'): #q to quit
            break

    video.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()