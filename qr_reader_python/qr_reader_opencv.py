import cv2
import numpy as np
import sys
import time

# Display barcode and QR code location
def draw(im, bbox):
    n = len(bbox)
    for j in range(n):
        cv2.line(im, tuple(bbox[j][0]), tuple(bbox[ (j+1) % n][0]), (255,255,0), 3)
 
    # Display results

# Main 
if __name__ == '__main__':
 
    qrDecoder = cv2.QRCodeDetector()

    cap = cv2.VideoCapture(0)
    cv2.namedWindow('Results',cv2.WINDOW_NORMAL)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        #center_y = int(frame.shape[0]/2)
        #center_x = int(frame.shape[1]/2)
        #y = center_y - 150
        #x = center_x - 150
        #h = 300
        #w = 600
    
        #frame = frame[y:y+h, x:x+w]
        #frame = cv2.imread("images/qr_example.jpg")  

        # Detect and decode the qrcode
        retval_detection, bbox	= qrDecoder.detect(frame)
        
        #data,bbox,rectifiedImage = qrDecoder.detectAndDecode(frame)

        if (retval_detection):
            retval_decoding, straight_qrcode =	qrDecoder.decode(frame, bbox)
            draw(frame, bbox)
            if (retval_decoding):
                print("Decoded Data : {}".format(retval_decoding))

            #rectifiedImage = np.uint8(straight_qrcode);
        else:
            print("QR Code not detected")

        cv2.imshow("Results", frame) 

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cv2.destroyAllWindows()
 

