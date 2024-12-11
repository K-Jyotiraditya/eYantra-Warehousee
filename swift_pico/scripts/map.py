
import cv2
import numpy as np
import cv2.aruco as aruco
import argparse

class Arena:

    def __init__(self, image_path):
        self.width = 1000
        self.height = 1000
        self.image_path = image_path
        self.detected_markers = []
        self.obstacles = 0
        self.total_area = 0

    def identification(self):
        # Read the image

        parser = argparse.ArgumentParser(description='Aruco Marker Detection')
        parser.add_argument('--image', required=True , help= '/home/jyotiraditya/pico_ws/src/swift_pico/scripts/task1c_image.jpg')
        args =parser.parse_args()

        frame = cv2.imread(args.image)
        # frame = cv2.imread(self.image_path)
        print(cv2.__version__)
        ###################################
        # Identify the Aruco ID's in the given image

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        print("Corners=",corners)
        print("IDS=",ids)
        print("Rejected=", rejected)

        # Variables to store marker edges
        self.marker_one = []
        self.marker_two = []
        self.marker_three = []
        self.marker_four = []


        if ids is not None:
            self.detected_markers = ids.flatten().tolist()
           
            print("Detected_marker=",)
            
            # Flatten the detected corners
            
            
            if len(corners) > 0:
                # Flatten the corners and store them in respective marker variables
                if self.detected_markers[0] in self.detected_markers:
                    self.marker_one = corners[self.detected_markers.index(self.detected_markers[0])].flatten().tolist()
                if self.detected_markers[1] in self.detected_markers:
                    self.marker_two = corners[self.detected_markers.index(self.detected_markers[1])].flatten().tolist()
                if self.detected_markers[2] in self.detected_markers:
                    self.marker_three = corners[self.detected_markers.index(self.detected_markers[2])].flatten().tolist()
                if self.detected_markers[3] in self.detected_markers:
                    self.marker_four = corners[self.detected_markers.index(self.detected_markers[3])].flatten().tolist()



            # print(f"Aruco markers detected: {self.detected_markers}")
            # print(f"Marker one Corners detected: {self.marker_one }")
            # print(f"Marker two Corners detected: {self.marker_two}")
            # print(f"Marker three Corners detected: {self.marker_three }")
            # print(f"Marker four Corners detected: {self.marker_four}")
        
        else:
            print("No Aruco markers detected")
            return

        ###################################
        # Apply Perspective Transform

        # Define the destination points for perspective transformation
        dst_points = np.array([[0, 0], [self.width, 0] , [0, self.height], [self.width, self.height]], dtype="float32")

        # Ensure we have exactly four markers to define the transformation
        if len(corners) == 4:
            # Extract the four corners from the detected Aruco markers
            src_points = np.array([[self.marker_four[0],self.marker_four[1]] , [self.marker_three[2],self.marker_three[3]] ,
                        [self.marker_two[6],self.marker_two[7]] , [self.marker_one[4],self.marker_one[5]]], dtype="float32")

            print("Sec_points=",src_points)
            print("dst_points=",dst_points)

            # Compute the perspective transform matrix
            matrix = cv2.getPerspectiveTransform(src_points, dst_points)

            # Apply the perspective transformation
            transformed_image = cv2.warpPerspective(frame, matrix, (self.width, self.height))
            # cropped_image = transformed_image[0:self.height , 0:self.width]
            cropped_image = transformed_image[2:self.height-2 , 2:self.width-2]

            # Show the transformed image (optional)
            # cv2.imshow('Transformed Image', transformed_image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        else:
            print("Insufficient or excess Aruco markers for perspective transform")
            return

        ###################################
        # Use the transformed image to find obstacles and their area

        # Convert the transformed image to grayscale
        gray_transformed = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

        # Show the transformed image (optional)
        # cv2.imshow('Grey Transformed Image', gray_transformed)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # Apply a binary threshold to find obstacles (adjust threshold value as needed)

        ret, binary = cv2.threshold(gray_transformed, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        binary_invert = cv2.bitwise_not(binary)

        # Find contours (obstacles) in the thresholded image
        contours, hierarchy = cv2.findContours(binary_invert, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)


        print("contours=",contours)

        # Count the number of obstacles
        self.obstacles = len(contours)
        
        # Calculate the total area covered by obstacles
        self.total_area = sum([cv2.contourArea(cnt) for cnt in contours])

        print("Aruco ID's=",self.detected_markers)
        print("obstacles=",self.obstacles)
        print("Total_area=",self.total_area)


        # Draw the contours on the transformed image (optional)
        obstacle_image = cropped_image.copy()
        cv2.drawContours(obstacle_image, contours, -1, (0, 255, 0), 2)

        # Show the image with obstacles (optional)
        # cv2.imshow('Obstacles Detected', obstacle_image)
        # cv2.imshow('Obstacles Detected', binary_invert)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        ###################################

    def text_file(self):
        with open("obstacles.txt", "w") as file:
            file.write(f"Aruco ID: {self.detected_markers}\n")
            file.write(f"Obstacles: {self.obstacles}\n")
            file.write(f"Area: {self.total_area}\n")


if __name__ == '__main__':
    image_path = 'task1c_image.jpg'
    arena = Arena(image_path)
    arena.identification()
    arena.text_file()
