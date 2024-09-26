from eyeball.utils.USBcam import USBCamera
import cv2

def main():
    cam = USBCamera(0)

    while True:
        img = cam.read()
        cv2.imshow('image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    cam.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    