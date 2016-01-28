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

            int pointDist = 10;
            for (int j = 0; j < points.length - pointDist * 3; j++) {
                // Calculate circle
                Point a = points[j];
                Point b = points[j + pointDist];
                Point c = points[j + pointDist * 2];

                Circle circle = new Circle(a, b, c);

                // Filter circle by radius
                if (circle.r > 10 && circle.r < 60) {
//                    circle(resized, circle.center(), (int) circle.r, new Scalar(255, 0, 0));
                    circleList.add(circle);
                }
            }

        }

        // Draw all the data points
        for (Circle circle : circleList) {
            rectangle(resized, new Point(circle.x, circle.y), new Point(circle.x, circle.y), new Scalar(0, 0, 255));
        }

        // Remove outlying circles
        if (circleList.size() > 0) {
            double outlierDist = 30;
            { // Remove X outliers
                circleList.sort((o1, o2) -> o1.x > o2.x ? 1 : o1.x == o2.x ? 0 : -1);
                double median = circleList.get(circleList.size() / 2).x;
                List<Circle> toRemove = new ArrayList<>();
                for (Circle circle : circleList) {
                    if (circle.x < median - outlierDist || circle.x > median + outlierDist)
                        toRemove.add(circle);
                }
                circleList.removeAll(toRemove);
            }
            { // Remove Y outliers
                circleList.sort((o1, o2) -> o1.y > o2.y ? 1 : o1.y == o2.y ? 0 : -1);
                double median = circleList.get(circleList.size() / 2).y;
                List<Circle> toRemove = new ArrayList<>();
                for (Circle circle : circleList) {
                    if (circle.y < median - outlierDist || circle.y > median + outlierDist)
                        toRemove.add(circle);
                }
                circleList.removeAll(toRemove);
            }
//            double outlierRadius = 50;
//            { // Remove R outliers
//                circleList.sort((o1, o2) -> o1.r > o2.r ? 1 : o1.r == o2.r ? 0 : -1);
//                double median = circleList.get(circleList.size() / 2).r;
//                List<Circle> toRemove = new ArrayList<>();
//                for (Circle circle : circleList) {
//                    if (circle.y < median - outlierRadius || circle.r > median + outlierRadius)
//                        toRemove.add(circle);
//                }
//                circleList.removeAll(toRemove);
//            }

            // Find average of meaningful circleList
            double x = 0;
            double y = 0;
            double r = 0;
            {
                for (Circle circle : circleList) {
                    x += circle.x;
                    y += circle.y;
                    r += circle.r;
                }
                x /= circleList.size();
                y /= circleList.size();
                r /= circleList.size();
            }

            // Calculate spread of data
            double deviationSqrSum = 0;
            for (Circle circle : circleList) {
                double dx = (x - circle.x);
                double dy = (y - circle.y);
                double distSqr = dx * dx + dy * dy;
                deviationSqrSum += distSqr;
            }
            deviationSqrSum /= (circleList.size() * circleList.size());
            System.out.println(deviationSqrSum);

            // Draw the found ball
            drawMarker(resized, new Point(x, y), new Scalar(0, 0, 255));
            boolean isBallValid = deviationSqrSum < 2 && circleList.size() > 10;

            // Draw data points
            for (Circle circle : circleList) {
                rectangle(resized, new Point(circle.x, circle.y), new Point(circle.x, circle.y), new Scalar(255, 0, 0));
            }

            if (isBallValid) {
                // Draw circleList that survived filtering
//                for (Circle circle : circleList) {
//                    circle(resized, new Point(circle.x, circle.y), (int) r, new Scalar(0, 255 - deviationSqrSum / 5, 0));
//                }

                circle(resized, new Point(x, y), (int) r, new Scalar(0, 255, 0));
            }
        }

        // Threshold based on saturation
        Mat hsv = new Mat();
        cvtColor(resized, hsv, COLOR_RGB2HSV);
        List<Mat> hsvSplits = new ArrayList<>();
        split(hsv, hsvSplits);

        debugImage = resized.clone();
        return null; // TODO replace
    }

//    private List<Circle> removeOutliers

    public Mat getDebugImg() {
        return debugImage;
    }

}
