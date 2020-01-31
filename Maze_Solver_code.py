import cv2
import numpy as np
import serial
import time

points=[]
c=[]
p=0

##pen=serial.Serial('com9',9600) 
##
##def send(x,y):                     
##    
##    s=str(x)+' '+str(y)+'\n'
##    pen.flush()
##    pen.write(s.encode())
##    time.sleep(1)
##
##send(-11.7,22)
##time.sleep(5)

frame = cv2.imread('maze.jpg')   #see to that the image is rotated manually such that the maze opening and closing is in top to bottom direction and not left to right
frame=cv2.resize(frame,(600,600),interpolation=cv2.INTER_AREA)
cv2.imshow("MAZE",frame)

def nothing(x):
    pass

cv2.namedWindow("Tracking")

cv2.createTrackbar("LH", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LS", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 0, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

while True:
    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")

    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")

    l_b = np.array([l_h, l_s, l_v])
    u_b = np.array([u_h, u_s, u_v])
    
    mask = cv2.inRange(hsv, l_b, u_b)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    
    cv2.imshow("res", res)
    
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()


bgr = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
ret, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV) 
kernel = np.ones((5, 5), np.uint8)
erosion = cv2.erode(thresh, kernel, iterations=1) 
contours, _ = cv2.findContours(erosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

imc=cv2.drawContours(res,contours,-1,(0,255,0),5)
cv2.imshow("dd",imc)
for i in range(len(contours) - 1):
        (x,y),radius = cv2.minEnclosingCircle(contours[i+1])
        x=int(x)
        y=int(y)
        c.append([x,y])

#Now we have to order the points in the order top left(tl),top right(tr),bottom left(bl),bottom right(br)
#the below algorithm will do the ordering of points

#the coordinates with max sum is br
#the coordinates with min sum is tl
csum = [0,0,0,0]
for i in range(len(c)):
    csum[i]=c[i][0]+c[i][1]
BR=max(csum)
br=csum.index(BR)
TL=min(csum)
tl=csum.index(TL)

#now we store the x and y coordinates in two different lists ccx and ccy
#now we make br and tl positions 0 and we will find tr and bl by the logic that if x coordinate is greater then it is tr and if y coordinate is greater then it is bl
ccx=[0,0,0,0]
for i in range(len(c)):
    ccx[i]=c[i][0]
ccy=[0,0,0,0]
for i in range(len(c)):
    ccy[i]=c[i][1]    
ccx[br]=0
ccx[tl]=0
ccy[br]=0
ccy[tl]=0
TR=max(ccx)
tr=ccx.index(TR)
BL=max(ccy)
bl=ccy.index(BL)

#now we have index of tl,tr,bl,br
#we will order it according to our interest(similar to that in the crop variable
co=[0,0,0,0]
co[0]=c[tl]
co[1]=c[tr]
co[2]=c[bl]
co[3]=c[br]

#now lets do perspective transformation with the points that we got
co = np.float32(co)
crop = np.float32([[0,0],[600,0],[0,600],[600,600]])#tl,tr,bl,br order
M = cv2.getPerspectiveTransform(co, crop) 
img = cv2.warpPerspective(frame, M, (600, 600))#this variable has the cropped image



gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
kerne = np.ones((2, 2), np.uint8)
dilate = cv2.dilate(thresh, kerne, iterations=3)
contours, _ = cv2.findContours(dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

drawing1 = np.zeros(thresh.shape, np.uint8)
drawing2 = np.zeros(thresh.shape, np.uint8)

#code to find the two halves using first 2 maximum area method
surf=[]
for i in range(len(contours)-1):
    cnt = contours[i]
    area = cv2.contourArea(cnt)
    surf.append(area)

#to find the index of the maximum
num1=max(surf)
q=surf.index(num1)
surf[q]=-1
num2=max(surf)
r=surf.index(num2)

#now we have the index of 2 contours with the 2 halves of maze now we use 
draw1=cv2.drawContours(drawing2, contours, q, (255, 255, 255), -1)
draw2=cv2.drawContours(drawing1, contours, r, (255, 255, 255), -1)
cv2.imshow("imc1",draw1)
cv2.imshow("imc2",draw2)
dilate_mask = np.ones((19, 19), np.uint8)

dilate1 = cv2.dilate(drawing1, dilate_mask, iterations=4)
dilate2 = cv2.dilate(drawing2, dilate_mask, iterations=4)
path = cv2.bitwise_and(dilate1, dilate2)

cv2.imshow("path",path)
kernel = np.ones((2, 2), np.uint8)
erosion = cv2.erode(path, kernel, iterations=10)
cnts, _ = cv2.findContours(erosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


l = len(cnts[0])

l /= 2
l = int(l)
for i in range(0, l,20): 
    a = cnts[0][i]
    (x, y) = (a[0][0], a[0][1])
    points.append((x,y))

temp = np.zeros((800,800,3), np.uint8)

for i in range(len(points)):            
    (x,y)=points[i]
    cv2.circle(thresh, (x, y), 1, (255, 255, 255), -1) 
    cv2.imshow("result",thresh)
    if(p==0):
        time.sleep(5)
    p+=1
    x=(x+20)
    y=600-(y-15)
##    if(x==359 and y==545):
##        send(x,y)
##        time.sleep(5)
##        continue
##    send(x,y)
    


