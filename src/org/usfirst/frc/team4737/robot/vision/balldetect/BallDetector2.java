package org.usfirst.frc.team4737.robot.vision.balldetect;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

import java.util.*;

import static org.opencv.imgproc.Imgproc.*;
import static org.opencv.core.Core.*;

/**
 * @author Brian Semrau
 * @version 1/15/2016
 */
public class BallDetector2 {

    private Mat debugImage;

    public BallDetector2() {

    }

    public Circle[] findBoulders(Mat sourceImageRGB) {

        // Resize the image
        int scale = 4;
        Mat resized = new Mat();
        resize(sourceImageRGB, resized, new Size(sourceImageRGB.width() / scale, sourceImageRGB.height() / scale));

        // Convert to grayscale
        Mat gray = new Mat();
        cvtColor(resized, gray, COLOR_RGB2GRAY);

        // Blur the image for canny edge detection
        int blurSize = 4;
        Mat blurred = new Mat();
        blur(gray, blurred, new Size(blurSize, blurSize));

        // Perform edge detection
        Mat canny = new Mat();
        Canny(blurred, canny, 49, 50);

        // Find contours of edges
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(canny, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

        // Calculate fitting circles on each contour
        List<Circle> circleList = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {

            MatOfPoint contour = contours.get(i);
            Point[] points = contour.toArray();

            int pointDist = 25;
            for (int j = 0; j < points.length - pointDist * 3; j++) {
                // Calculate circle
                Point a = points[j];
                Point b = points[j + pointDist];
                Point c = points[j + pointDist * 2];

                Circle circle = new Circle(a, b, c);

                // Filter circle by radius
                if (circle.r > 10 && circle.r < 30) {
                    circle(resized, circle.center(), (int) circle.r, new Scalar(255, 0, 0));
                    circleList.add(circle);
                }
            }

        }


        // Threshold based on saturation
//        Mat hsv = new Mat();
//        cvtColor(resized, hsv, COLOR_RGB2HSV);
//        List<Mat> hsvSplits = new ArrayList<>();
//        split(hsv, hsvSplits);

        debugImage = canny.clone();
        return null; // TODO replace
    }

    public Mat getDebugImg() {
        return debugImage;
    }

}
