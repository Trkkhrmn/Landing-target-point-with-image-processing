import cv2
import numpy as np
import math
import time

def search_lock(target, algo, precision):
    cap = cv2.VideoCapture(0)
    cap.set(3,432)
    cap.set(4,432)

    if algo == "orb":
        orb = cv2.ORB_create()
        indexparameters= dict(algorithm = 6,
                     table_number = 12,
                     key_size = 20,
                     multi_probe_level = 2)
        searchparameters = dict(checks=30)
        flann = cv2.FlannBasedMatcher(indexparameters, searchparameters)
    elif algo == "surf":
        surf = cv2.xfeatures2d.SURF_create() 
        indexparameters1 = dict(algorithm= 0 , trees = 5) 
        searchparameters1 = dict()
        flann = cv2.FlannBasedMatcher(indexparameters1, searchparameters1)
    else:
        print("Algorithm not supported")
        return

    lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    color = np.random.randint(0,255,(100,3))

    while cap.isOpened():
        ret , frame = cap.read()

        if not ret:
            break

        medianBlur = cv2.medianBlur(frame,5)
        grayFrame = cv2.cvtColor(medianBlur,cv2.COLOR_BGR2GRAY)

        if algo == "orb":
            keypoints, descriptors = orb.detectAndCompute(target, None)
            keypoints_grayFrame, descriptors_grayFrame = orb.detectAndCompute(grayFrame, None)
        elif algo == "surf":
            keypoints, descriptors = surf.detectAndCompute(target, None) 
            keypoints_grayFrame, descriptors_grayFrame = surf.detectAndCompute(grayFrame, None)

        matches_flann = flann.knnMatch(descriptors, descriptors_grayFrame, k=2)
        
        goodMatches_flann = []
        for m in matches_flann:
            if len(m) > 0 and m[0].distance < precision * m[-1].distance:
                goodMatches_flann.append(m[0])

        result_flann = cv2.drawMatches(target, keypoints, grayFrame, keypoints_grayFrame, goodMatches_flann, grayFrame)

        if len(goodMatches_flann) > 7:
            queryPoints = np.float32([keypoints[i.queryIdx].pt for i in goodMatches_flann]).reshape(-1, 1, 2)   
            trainPoints = np.float32([keypoints_grayFrame[j.trainIdx].pt for j in goodMatches_flann]).reshape(-1, 1, 2) 
            targetPoints = trainPoints
            mask = np.zeros_like(frame)

            while len(goodMatches_flann) > 7:
                ret,frame2 = cap.read()
                newGrayFrame = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

                flowMatches, st, err = cv2.calcOpticalFlowPyrLK(grayFrame, newGrayFrame, targetPoints, None, **lk_params)

                try:
                    good_new = flowMatches[st==1]
                    good_old = targetPoints[st==1]
                except:
                    break

                for i,(new,old) in enumerate(zip(good_new,good_old)):
                    a,b = new.ravel()
                    c,d = old.ravel()
                    mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
                    frame3 = cv2.circle(frame2,(a,b),5,color[i].tolist(),-1)
                    
                    if not ((150 < a < 195) and (115 < b < 155)):  
                        if a < 130:
                            print("RIGHT!")
                        elif a > 220:
                            print("LEFT!")
                        elif b < 114:
                            print("BACK!")
                        elif b > 190:
                            print("FRONT!")
                    else:
                        cv2.putText(frame3,'Initializing Landing...',(80,80), cv2.FONT_HERSHEY_SIMPLEX, .5,(30,40,50),2,cv2.LINE_AA)
                        print("Initiate Landing...")
                    
                img = cv2.add(frame3, mask)
                img = cv2.rectangle(img, (100,95), (250,175),(255,0, 0), 2)
                img = cv2.rectangle(img, (120,115), (230,155),(0,255, 0), 2)    
                cv2.imshow('frame',img)
                grayFrame = newGrayFrame.copy()
                targetPoints = good_new.reshape(-1,1,2)
                if cv2.waitKey(1) == 27:
                    break
        else:
            cv2.imshow("Result", result_flann)

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
    cap.release()

target1 = cv2.imread("C:/Users/KAHRAMAN/Desktop/drone/VEFA_logo.png", cv2.IMREAD_GRAYSCALE)
target2 = cv2.imread("C:/Users/KAHRAMAN/Desktop/drone/YTU_logo.jpg", cv2.IMREAD_GRAYSCALE)
target3 = cv2.imread("C:/Users/KAHRAMAN/Desktop/drone/LAND_logo.png", cv2.IMREAD_GRAYSCALE)
home = cv2.imread("C:/Users/KAHRAMAN/Desktop/drone/TR_flag.png", cv2.IMREAD_GRAYSCALE)

search_lock(target1, "orb", 0.5)
