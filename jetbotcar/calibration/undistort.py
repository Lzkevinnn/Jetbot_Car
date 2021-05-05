# You should replace these 3 lines with the output in calibration step
import numpy as np
import cv2
import sys

DIM=(640, 480)
K=np.array([[303.50302110360155, 0.0, 309.3304248853367], [0.0, 301.7866827641378, 250.41685282205265], [0.0, 0.0, 1.0]])
D=np.array([[0.04531290460366392], [-1.330731154170817], [4.949802075414228], [-5.589086135185314]])
def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)