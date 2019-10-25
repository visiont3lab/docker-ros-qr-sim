from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
 
 
def draw(points,im, qr_type, qr_data):
    # If the points do not form a quad, find convex hull
    if len(points) > 4 : 
        hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
        hull = list(map(tuple, np.squeeze(hull)))
    else : 
        hull = points;
    
    # Number of points in the convex hull
    n = len(hull)

    # Draw the convext hull
    for j in range(0,n):
        cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)
    
    cv2.putText(im, str(qr_type) + ": " + str(qr_data), hull[0], 0, 0.8, (0, 255, 0), 2)

    return im

def run(img):
    
    # Find barcodes and QR codes
    decodedObjects = pyzbar.decode(img)
    
    for decodedObject in decodedObjects:
        print('Type : ', decodedObject.type)
        print('Data : ', decodedObject.data,'\n')

        points = decodedObject.polygon
        img = draw(points, img, decodedObject.type, decodedObject.data) 

    return img;
    
# Main 
if __name__ == '__main__':
 

    cap = cv2.VideoCapture(0)
    cv2.namedWindow('Results',cv2.WINDOW_NORMAL)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
       
        # Read image
        #frame = cv2.imread('images/1571151760.jpg')
        
        # Crop image commands
        #center_y = int(frame.shape[0]/2)
        #center_x = int(frame.shape[1]/2)
        #y = center_y - 400
        #x = center_x - 400
        #h = 800
        #w = 800
        #frame = frame[y:y+h, x:x+w]

        frame = run(frame)

        # Display results 
        cv2.imshow("Results", frame);

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
                
