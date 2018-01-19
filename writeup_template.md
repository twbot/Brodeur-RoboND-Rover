# Rover Search and Return - Udacity

Here is a [video](https://youtu.be/D5wGoIJFGIo) of the Rover navigation

Simulator Settings:
	- Screen Resolution: 640 X 480
    - Graphics Quality: Good


[//]: # (Image References)

[image1]: ./calibration_images/example_grid1.jpg
[image2]: ./calibration_images/angle_example.jpg
[image3]: ./calibration_images/transform_ex.png
[image4]: ./calibration_images/thresholding.png
[image5]: ./calibration_images/coordinate.png

## Notebook Analysis

Within this Jupyter Notebook are all the functions providing the necessary data that will allow the rover to autonomously navigate its environment.

#### Sample Data

In the simulator, you can toggle a grid onto the the ground for calibration. The image below is an example. We will use this as a reference for the perspective transform function, allowing us to choose source points, as well as adjust the offset value from the bottom, to account for image data directly in front of the rover.

![Grid Image][image1]

### Perspective Transform

This function allows us to tranform the image from front point-of-view to a top-down point-of-view, allowing us to calculate the average angle based on available path pixels, once color thresholding is applied, which is the next step.

![Perspect Transform][image3]

### Color Thresholding

We will apply three seperate color thresholding functions to account for the navigable terrain, obstacles, and the goals. You could condense this into one function, and pass in the rgb threshold, I just prefer this method as it's cleaner and one can know exactly which threshold you're applying via the naming conventions. Although I only use the goal and navigable terrain thresholding for the rover, I plan on coming back and using obstacle thresholding to closely follow the wall, as it is apparently the most efficient navigation method.

Using the RGB image splitting function, I looked at the intensity values and approximated the values that would be needed in order for the robot to "see" its goal points and its obstacles.

The resultant image is an image that only displays the information we want from that particular function. In the case of the goal_thresholding function, it shows a pixel intesity of 1 at every threshold point.

![Path Threshold][image4]


### Coordinate Transformation

In order for the environment to be correctly observed, we "flipped the pixels" to rover-centric coordinates. It first grabs all non-zero pixels from the resultant images of the color thresholding functions, and flipped to seperate axis, while also applying some filters (subtractng to flip rover x-axis, dividing x axis for steering, etc.)

![Path Threshold][image5]
