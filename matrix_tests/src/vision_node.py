#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
#from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from serp.msg import Matrix
bridge = CvBridge()
import numpy as np

#from PIL import Image 
#import glob
#import time

##### START DEFINIG FUNCTIONS #####

# Undistortion
def undistort(img):
    # Undistortion Parameters - that came from the calibration function 
    DIM = (1920, 1080)
    K = np.array([[950.0280635162867, 0.0, 942.1890938140375], [0.0, 951.2475443289691, 624.0548417573921], [0.0, 0.0, 1.0]])
    D = np.array([[-0.01846168620928796], [0.001797964190577665], [0.0068078755063244525], [-0.006138159488132415]])
    h,w = img.shape[:2]
    map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv.CV_16SC2)
    undistorted_img = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

    #cv.imshow("undis", undistorted_img)
    #cv.waitKey(0) # waits until a key is pressed

    #plotImage(undistorted_img, 14)
    #cv.imwrite('undis_' + img_path, undistorted_img)
    #cv.waitKey(0)
    #cv.destroyAllWindows()
    return undistorted_img

# True if all corners (Up Left, Up Right, Bottom Left, Bottom Right) found - they give us the border of our sheet 
def cornersInArucos(arucos):
  aux = 0
  corners = [28, 29, 30, 31]
  for i in range(len(arucos)):
    if arucos[i] in corners:
      aux += 1
  if (aux == 4):
    return True
  return False

# Return image in correct orientation and perspective
def correctPerspective(img,ids,corners,width,height): 
  for i, corner in zip(ids, corners):
    if i==28: #Top Left ArUco
      #print("Top Left ArUco found!")
      x1 = corner[0][0][0]
      y1 = corner[0][0][1]
    elif i==30: #Top Right ArUco
      #print("Top Right ArUco found!")
      x2 = corner[0][1][0]
      y2 = corner[0][1][1]
    elif i==31: #Bottom Right ArUco
      #print("Bottom Right ArUco found!")
      x3 = corner[0][2][0]
      y3 = corner[0][2][1]
    elif i==29: #Bottom Left ArUco
      #print("Bottom Left ArUco found!")
      x4 = corner[0][3][0]
      y4 = corner[0][3][1]
  input = np.float32([[x1,y1], [x2,y2], [x3,y3], [x4,y4]])
  output = np.float32([[0,0], [width-1,0], [width-1,height-1], [0,height-1]])
  matrix = cv.getPerspectiveTransform(input,output)
  return cv.warpPerspective(img, matrix, (width,height), cv.INTER_LINEAR)

# Return -1 if ArUco in wrong orientation is found
def checkArucoOrientation(ids,corners):
  for i, corner in zip(ids, corners):
    if (corner[0][0][0] < corner[0][1][0]) and (corner[0][0][1]<corner[0][3][1]):
      continue
    print('ERROR: ArUco', int(i), '- WRONG ORIENTATION!')
    return -1
  return 1

#Functions to give border limits of ArUcos
def getXLeft(corner):
  return int(corner[0][0][0]-8)
def getYTop(corner):
  return int(corner[0][0][1]-12)
def getXRight(corner):
  return int(corner[0][2][0]+8)
def getYBot(corner):
  return int(corner[0][2][1]+8)

def findThresh(hist, limiar):
  pos1 = pos2 = 0
  firstPeak = endFirst = secondPeak = False
  for i in range(len(hist)):
    if (hist[i][0] > limiar) and (firstPeak == False):
      firstPeak = True
    elif (hist[i][0] < limiar) and (firstPeak == True) and (endFirst == False):
      endFirst = True
      pos1 = i
    elif (hist[i][0] > limiar) and (endFirst == True):
      pos2 = i
      break
  return min(int((pos1+pos2)/2), 255)

# Function that returna ArUco type
def arucoType(aruco):
  if (aruco in [2,7,8]) or ((aruco>=14)and(aruco<=23)):
    return 'DOUBLE' # 1 INPUT 1 OUTPUT
  if (aruco in [0,1,13,25,26]) or ((aruco>=3)and(aruco<=6)):
    return 'TRIPLE' # 2 INPUT 1 OUTPUT
  if ((aruco>=9)and(aruco<=12)):
    return 'SENSOR' # 1 OUTPUT 
  if aruco==27:
    return 'MUX' # 2 INPUT 1 YELLOW ENTRY (BOT) 1 OUTPUT
  if aruco==32:
    return 'ELSE_IF' # 2 INPUT 1 YELLOW ENTRY (TOP) 1 OUTPUT
  if aruco==33:
    return 'EXTENSOR' # 1 INPUT 2 OUTPUT
  if aruco==34:
    return 'TE' # 2 INPUT 
  if aruco==35:
    return 'TD' # 2 OUTPUT
  return 'ERROR_ARUCO_TYPE'

# Funtion to search lines near ArUcos
def findLine(lines, xc, yc, w, h):
  for x in range(int(xc-w/2), int(xc+w/2)):
    for y in range(int(yc-h/2), int(yc+h/2)):
      pixel = lines.item(y, x)
      if pixel != 0:
        return pixel
  return -1

# Funcoes apenas para traduzir matriz
# Function to print ArUcos IN/OUTputs
def printArUcosINOUT(vector):
  for id in range(0, len(vector)):
    if (arucoType(vector[id][1])=='DOUBLE'):
      print(int(vector[id][0]), ': ArUco', int(vector[id][1]))
      print('INPUT ->', findConnectedAruco(id, vector[id][2], vector))
      print('OUTPUT ->', findConnectedAruco(id, vector[id][5], vector))
      print()
    elif (arucoType(vector[id][1])=='TRIPLE'):
      print(int(vector[id][0]), ': ArUco', int(vector[id][1]))
      print('INPUT1 ->', findConnectedAruco(id, vector[id][2], vector))
      if vector[id][1]==3:
        print('COND ->', findConnectedAruco(id, vector[id][3], vector))
      else:
        print('INPUT2 ->', findConnectedAruco(id, vector[id][3], vector))
      print('OUTPUT ->', findConnectedAruco(id, vector[id][5], vector))
      print()
    elif (arucoType(vector[id][1])=='SENSOR'):
      print(int(vector[id][0]), ': ArUco', int(vector[id][1]))
      print('OUTPUT ->', findConnectedAruco(id, vector[id][5], vector))
      print()
    elif (arucoType(vector[id][1])=='MUX') or (arucoType(vector[id][1])=='ELSE_IF'):
      print(int(vector[id][0]), ': ArUco', int(vector[id][1]))
      print('INPUT1 ->', findConnectedAruco(id, vector[id][2], vector))
      print('INPUT2 ->', findConnectedAruco(id, vector[id][3], vector))
      print('COND ->', findConnectedAruco(id, vector[id][4], vector))
      print('OUTPUT ->', findConnectedAruco(id, vector[id][5], vector))
      print()
    elif (arucoType(vector[id][1])=='EXTENSOR'):
      print(int(vector[id][0]), ': ArUco', int(vector[id][1]))
      print('INPUT ->', findConnectedAruco(id, vector[id][2], vector))
      print('OUTPUT1 ->', findConnectedAruco(id, vector[id][4], vector))
      print('OUTPUT2 ->', findConnectedAruco(id, vector[id][5], vector))
      print()
    elif (arucoType(vector[id][1])=='TE'):
      print(int(vector[id][0]), ': ArUco', int(vector[id][1]))
      print('INPUT1 ->', findConnectedAruco(id, vector[id][2], vector))
      print('INPUT2 ->', findConnectedAruco(id, vector[id][3], vector))
      print()
    elif (arucoType(vector[id][1])=='TD'):
      print(int(vector[id][0]), ': ArUco', int(vector[id][1]))
      print('OUTPUT1 ->', findConnectedAruco(id, vector[id][4], vector))
      print('OUTPUT2 ->', findConnectedAruco(id, vector[id][5], vector))
      print()
    

#Fucntion to find connected ArUco
def findConnectedAruco(id, line, vector): # id of ArUco, entry of aruco_lines, 
  if line == -1:
    return 'None'
  for i in range(0,len(vector)):
    if i==id:
      continue
    for j in range(2,len(vector[i])):
      if vector[i][j]==line:
        string = str(int(vector[i][0])) + ': ArUco ' + str(int(vector[i][1]))
        return string
  return 'None'

# Functions to return the 1 of the 4 white corners of an ArUco
def returnArUcoCorner(corner, fourCorners):
  if(fourCorners == 'top_left'):
    return corner[0][0][0], corner[0][0][1] # x_top_left, y_top_left
  elif(fourCorners == 'top_right'):
    return corner[0][1][0], corner[0][1][1] # x_top_right, y_top_right
  elif(fourCorners == 'bot_right'):
    return corner[0][2][0], corner[0][2][1] # x_bot_right, y_bot_right
  elif(fourCorners == 'bot_left'):
    return corner[0][3][0], corner[0][3][1] # x_bot_left, y_bot_left

# Function for organizing the Logic Team Output
#Output requested by the Logic Team ( Top Output | Bot Output | Top Input | Bot Input | Yellow Entry )
def logicOutput(arucos, connections_matrix): # connections - first aruco_line, 
  # find out the identifiers of each connection
  #print(connections_matrix)
  connections = np.unique(connections_matrix)
  #print(connections)
  #print(connections[1:])
  # find out number of connections
  n_connections = len(np.unique(connections_matrix))-1
  #print(n_connections)
  n_arucos = len(arucos[:,1])
  #print(arucos)
  #print(n_arucos)
  n_entries = 5
  # create output matrix with the detected connections (from which ArUco to which ArUco)
  output_Logic = -np.ones((n_connections, n_entries)) # ( Begin Top | Begin Bot | End Top | End Bot | Yellow Entry )
  index = 0
  #print(connections_matrix)
  for connection in connections[1:]:
   for i in range(0, n_arucos):
      for j in range(0, 4):
        #print(str(connection) + ' ' + str(connections_matrix[i][j]))
        if connection == connections_matrix[i][j]:
          # 1 INPUT 1 OUTPUT
          if arucoType(arucos[i][1]) == 'DOUBLE':
            if j == 0: # ArUco Input - End of line
              output_Logic[index][2] = arucos[i][0]
            elif j == 3: # ArUco Output - Begin of line
              output_Logic[index][0] = arucos[i][0]
          # 2 INPUTS 1 OUTPUT
          elif arucoType(arucos[i][1]) == 'TRIPLE':
            if j == 0: # ArUco Top Input - End of line
              output_Logic[index][2] = arucos[i][0]
            elif j == 1: # ArUco Bot Input - End of line
              output_Logic[index][3] = arucos[i][0]
            elif j == 3: # ArUco Output - Begin of line
              output_Logic[index][0] == arucos[i][0]
          # 1 OUTPUT    
          elif arucoType(arucos[i][1]) == 'SENSOR':
            if j == 3: # ArUco Output - Begin of line
              output_Logic[index][0] = arucos[i][0]
          # 2 INPUTS 1 YELLOW 1 OUTPUT
          elif arucoType(arucos[i][1]) == 'MUX' or arucoType(arucos[i][1]) == 'ELSE_IF':
            if j == 0: # ArUco Top Input - End of line
              output_Logic[index][2] = arucos[i][0]
            elif j == 1: # ArUco Bot Input - End of line
              output_Logic[index][3] = arucos[i][0]
            elif j == 2: # ArUco Yellow - End of line
              output_Logic[index][4] = arucos[i][0]
            elif j == 3: # ArUco Output - Begin of line
              output_Logic[index][0] = arucos[i][0]
          # 1 INPUT 2 OUTPUTS
          elif arucoType(arucos[i][1]) == 'EXTENSOR':
            if j == 0: # ArUco Top Input - End of line
              output_Logic[index][2] = arucos[i][0]
            elif j == 2: # ArUco Top Output - Begin of line
              output_Logic[index][0] = arucos[i][0]
            elif j == 3: # ArUco Bot Output - Begin of line
              output_Logic[index][1] = arucos[i][0]
          # 2 INPUTS
          elif arucoType(arucos[i][1]) == 'TE':
            if j == 0: # ArUco Top Input - End of line
              output_Logic[index][2] = arucos[i][0]
            elif j == 1: # ArUco Bot Input - End of line
              output_Logic[index][3] = arucos[i][0]
          # 2 OUTPUTS
          elif arucoType(arucos[i][1]) == 'TD':
            if j == 3: # ArUco Bot Output - Begin of line
              output_Logic[index][1] = arucos[i][0]
            elif j == 2: # ArUco Top Output - Begin of line
              output_Logic[index][0] = arucos[i][0]
   index = index + 1

  return output_Logic 

#Functions to add lines start and end pos
def setStartLine(x, y, line, pos):
  if pos==-1:
    return
  line[pos-1][0]=x
  line[pos-1][1]=y
  return
def setEndLine(x, y, line, pos):
  if pos==-1:
    return
  line[pos-1][2]=x
  line[pos-1][3]=y
  return

#Function to assign start and end of a line to an ArUco
def assignLines2ArUcos(ids, corners, num_labels, labels_im):

  # Vector for ArUcos 
  aruco_lines = -np.ones((len(ids),4))

  # Vector for start and final pos of each line: x_start | y_start | x_end | xy_end
  lines = -np.ones((num_labels,4))     # Original: lines = -np.ones((len(ids),4))

  id=0
  for i, corner in zip(ids, corners):
    if arucoType(i)=='DOUBLE':
      #Definir posicao input
      xin = getXLeft(corner)-2
      yin = (getYTop(corner)+getYBot(corner))/2
      #Definir posicao output
      xout = getXRight(corner)+2
      yout = (getYTop(corner)+getYBot(corner))/2   # aqui nao e melhor colocar o y do top_left ???
      #Get line
      aruco_lines[id][0] = findLine(labels_im, xin, yin, 4, 10)
      aruco_lines[id][3] = findLine(labels_im, xout, yout, 4, 10)
      #Fill lines vector
      setEndLine(xin, yin, lines, int(aruco_lines[id][0]))
      setStartLine(xout, yout, lines, int(aruco_lines[id][3]))
    elif arucoType(i)=='TRIPLE':
      #Definir posicao input
      xin1 = getXLeft(corner)-2
      yin1 = corner[0][0][1]+5
      xin2 = getXLeft(corner)-2
      yin2 = corner[0][3][1]-5
      #Definir posicao output
      xout = getXRight(corner)+2
      yout = (getYTop(corner)+getYBot(corner))/2
      #Get line
      aruco_lines[id][0] = findLine(labels_im, xin1, yin1, 4, 10)
      aruco_lines[id][1] = findLine(labels_im, xin2, yin2, 4, 10)
      aruco_lines[id][3] = findLine(labels_im, xout, yout, 4, 10)
      #Fill lines vector
      setEndLine(xin1, yin1, lines, int(aruco_lines[id][0]))
      setEndLine(xin2, yin2, lines, int(aruco_lines[id][1]))
      setStartLine(xout, yout, lines, int(aruco_lines[id][3]))
    elif arucoType(i)=='SENSOR':
      #Definir posicao output
      xout = getXRight(corner)+2
      yout = (getYTop(corner)+getYBot(corner))/2
      #Get line
      aruco_lines[id][3] = findLine(labels_im, xout, yout, 4, 10)
      #Fill lines vector
      setStartLine(xout, yout, lines, int(aruco_lines[id][3]))
    elif arucoType(i)=='MUX':
      #Definir posicao input
      xin1 = getXLeft(corner)-2
      yin1 = corner[0][0][1]+5
      xin2 = getXLeft(corner)-2
      yin2 = corner[0][3][1]-5
      #Definir posicao condicao
      xcond = (getXLeft(corner)+getXRight(corner))/2
      ycond = getYBot(corner)
      #Definir posicao output
      xout = getXRight(corner)+2
      yout = (getYTop(corner)+getYBot(corner))/2
      #Get line
      aruco_lines[id][0] = findLine(labels_im, xin1, yin1, 4, 10)
      aruco_lines[id][1] = findLine(labels_im, xin2, yin2, 4, 10)
      aruco_lines[id][2] = findLine(labels_im, xcond, ycond, 4, 10)
      aruco_lines[id][3] = findLine(labels_im, xout, yout, 4, 10)
      #Fill lines vector
      setEndLine(xin1, yin1, lines, int(aruco_lines[id][0]))
      setEndLine(xin2, yin2, lines, int(aruco_lines[id][1]))
      setEndLine(xcond, ycond, lines, int(aruco_lines[id][2]))
      setStartLine(xout, yout, lines, int(aruco_lines[id][3]))
    elif arucoType(i)=='ELSE_IF':
      #Definir posicao input
      xin1 = getXLeft(corner)-2
      yin1 = corner[0][0][1]+5
      xin2 = getXLeft(corner)-2
      yin2 = corner[0][3][1]-5
      #Definir posicao condicao
      xcond = (getXLeft(corner)+getXRight(corner))/2
      ycond = getYTop(corner)
      #Definir posicao output
      xout = getXRight(corner)+2
      yout = (getYTop(corner)+getYBot(corner))/2
      #Get line
      aruco_lines[id][0] = findLine(labels_im, xin1, yin1, 4, 10)
      aruco_lines[id][1] = findLine(labels_im, xin2, yin2, 4, 10)
      aruco_lines[id][2] = findLine(labels_im, xcond, ycond, 4, 10)
      aruco_lines[id][3] = findLine(labels_im, xout, yout, 4, 10)
      #Fill lines vector
      setEndLine(xin1, yin1, lines, int(aruco_lines[id][0]))
      setEndLine(xin2, yin2, lines, int(aruco_lines[id][1]))
      setEndLine(xcond, ycond, lines, int(aruco_lines[id][2]))
      setStartLine(xout, yout, lines, int(aruco_lines[id][3]))
    elif arucoType(i)=='EXTENSOR':
      #Definir posicao input
      xin = getXLeft(corner)-2
      yin = (getYTop(corner)+getYBot(corner))/2
      #Definir posicao output
      xout1 = getXRight(corner)+2
      yout1 = corner[0][1][1]+5
      xout2 = getXRight(corner)+2
      yout2 = corner[0][2][1]-5
      #Get line
      aruco_lines[id][0] = findLine(labels_im, xin, yin, 4, 10)
      aruco_lines[id][2] = findLine(labels_im, xout1, yout1, 4, 10)
      aruco_lines[id][3] = findLine(labels_im, xout2, yout2, 4, 10)
      #Fill lines vector
      setEndLine(xin, yin, lines, int(aruco_lines[id][0]))
      setStartLine(xout1, yout1, lines, int(aruco_lines[id][2]))
      setStartLine(xout2, yout2, lines, int(aruco_lines[id][3]))
    elif arucoType(i)=='TE':
      #Definir posicao input
      xin1 = getXLeft(corner)-2
      yin1 = corner[0][0][1]+5
      xin2 = getXLeft(corner)-2
      yin2 = corner[0][3][1]-5
      #Get line
      aruco_lines[id][0] = findLine(labels_im, xin1, yin1, 4, 10)
      aruco_lines[id][1] = findLine(labels_im, xin2, yin2, 4, 10)
      #Fill lines vector
      setEndLine(xin1, yin1, lines, int(aruco_lines[id][0]))
      setEndLine(xin2, yin2, lines, int(aruco_lines[id][1]))
    elif arucoType(i)=='TD':
      #Definir posicao output
      xout1 = getXRight(corner)+2
      yout1 = corner[0][1][1]+5
      xout2 = getXRight(corner)+2
      yout2 = corner[0][2][1]-5
      #Get line
      aruco_lines[id][2] = findLine(labels_im, xout1, yout1, 4, 10)
      aruco_lines[id][3] = findLine(labels_im, xout2, yout2, 4, 10)
      #Fill lines vector
      setStartLine(xout1, yout1, lines, int(aruco_lines[id][2]))
      setStartLine(xout2, yout2, lines, int(aruco_lines[id][3]))
    id=id+1

  return aruco_lines, lines


# Global function to interpret images
def interpretImageCaptured(image, fisheye):

  if fisheye:
    # Remove distortion
    image = undistort(image)

  # Convert to RGB
  img = cv.cvtColor(image, cv.COLOR_BGR2RGB)

  # ArUco Parameters for Detection
  ARUCO_PARAMETERS = cv.aruco.DetectorParameters_create()
  ARUCO_DICT = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_250)

  # Detect ArUcos
  corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

  # Check 4 corners
  if (cornersInArucos(ids) == False):
    return -1, None, None, None

  # Perspective Correction
  width = 577  #1154
  height = 408  #816
  img = correctPerspective(img, ids, corners, width, height) 

  # Remove Corner ArUcos
  cv.rectangle(img, (0, 0), (55, 55), (255, 255, 255), -1)
  cv.rectangle(img, (width-55, 0), (width, 55), (255, 255, 255), -1)
  cv.rectangle(img, (0, height-55), (55, height), (255, 255, 255), -1)
  cv.rectangle(img, (width-55, height-55), (width, height), (255, 255, 255), -1)

  # Clone Image
  img_lines = img.copy()

  # Detect Arucos in New Perspective
  corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

  # Check ArUco orientation
  if checkArucoOrientation(ids, corners) == -1:
      return -3, None, None, None

  aruco_corner = [28, 29, 30, 31]

  # Check ArUcos
  if ids is not None:
      #print('Detected ArUcos:', len(ids))
      for i, corner in zip(ids, corners):
        if i in aruco_corner:
          continue
        cv.rectangle(img_lines, (getXLeft(corner), getYTop(corner)), (getXRight(corner), getYBot(corner)), (255, 255, 255), -1)
      img = cv.aruco.drawDetectedMarkers(image=img, corners=corners, borderColor=(0, 255, 0))
      #img = cv.aruco.drawDetectedMarkers(image=img, corners=corners, ids=ids, borderColor=(0, 255, 0))
  else:
      #print("NO ArUcos DETECTED")
      return -2, None, None, None

  # Plot
  #plotImage(img,14)

  # Convert to grey
  grey = cv.cvtColor(img_lines, cv.COLOR_RGB2GRAY)

  # Removing noise
  grey = cv.fastNlMeansDenoising(grey, None, 7, 7, 21)

  # Calculate histogram
  dst = cv.calcHist(grey, [0], None, [250], [0,250])
  #plt.plot(dst)

  # Binarization
  threshold = findThresh(dst, 3)
  #print('Threshold: ', threshold)
  _,thresh = cv.threshold(grey,threshold,255,cv.THRESH_BINARY_INV)

  # Close Morphologic Operation
  kernel = np.ones((9, 9), np.uint8)
  thresh = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel)

  # Show
  #cv.imshow("thresh", thresh)
  #cv.waitKey(0) # waits until a key is pressed

  # Separate lines
  num_labels, labels_im = cv.connectedComponents(thresh)

  # Assign lines to ArUcos
  aruco_lines, lines = assignLines2ArUcos(ids, corners, num_labels, labels_im)

  aux = np.zeros((len(ids),2))
  for i in range(0,len(ids)):
    aux[i][0]=i+1
    aux[i][1]=ids[i]

  # Translation to Logic Team Output
  output_Logic = logicOutput(aux, aruco_lines) 

  #Vector: ID | ID_ARUCO | INPUTS/OUTPUTS(LINES)
  aruco_lines = np.concatenate((aux, aruco_lines), axis=1)

  # Show
  cv.imshow("IMGGG", img)
  cv.waitKey(0) # waits until a key is pressed

  # Draw lines in image
  for i in range(0,len(lines)):
    if (-1 in lines[i]):
      continue
    start_point = (int(lines[i][0]), int(lines[i][1]))
    end_point = (int(lines[i][2]), int(lines[i][3]))
    img = cv.line(img, start_point, end_point, (50, 100, 255), 4)

  img = cv.cvtColor(img, cv.COLOR_RGB2BGR)

  return 1, aruco_lines[:,1], output_Logic, img



##### END DEFINIG FUNCTIONS #####

# ------------------------------#

#######   START ROS MAIN  #######

# Publishers (defined here to be global)
pub_error = rospy.Publisher("/errors", Int16, queue_size=1)
pub_logic1 = rospy.Publisher("/matrix", Matrix , queue_size=1)
#pub_logic2 = rospy.Publisher("/serp/logic2", '????', 1)
#pub_img = rospy.Publisher("/serp/analysed_img", Image, 1)

def callBack(data):
    # Prints on terminal
    rospy.loginfo('Image received 1...')
    # Decodes received image

    try:
      image = bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)

    #if cv_image is None:
    #  rospy.loginfo('Blank')

    #cv.imshow('sample image', cv_image)
    #cv.waitKey(0) # waits until a key is pressed
    
    # Analyses image
    ret, logic1, logic2, img = interpretImageCaptured(image, fisheye=True)
    # Check errors
    if (ret == -1):
        rospy.loginfo('ERROR: NOT ALL CORNERS DETECTED')
        pub_error.publish(-1)
    elif (ret == -2):
        rospy.loginfo('ERROR: NO ArUcos DETECTED')
        pub_error.publish(-2)
    elif (ret == -3):
        rospy.loginfo('ERROR: WRONG ORIENTATION ArUcos DETECTED')
        pub_error.publish(-3)
    # If no errors send matrix to logic
    else:
        rospy.loginfo('NOT ERROR! incrivel se entrar aqui!!')
        cv.imshow("final_img", img)
        cv.waitKey(0) # waits until a key is pressed
        mat = Matrix()
        mat.manual_mode = False
        mat.matrix1 = logic1
        mat.matrix2 = logic2.flatten()
        #rospy.loginfo(logic1) # SEND LOGIC
        pub_logic1.publish(mat)
        
        #pub_img.publih(img)

def main():
    # Initialize the node
    rospy.init_node('vision_node', anonymous=True)

    # Subscriber
    rospy.Subscriber("/image", Image, callBack)

    # Publishers
        # Defined above
    # Keep awake
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    
