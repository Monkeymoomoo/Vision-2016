package org.usfirst.frc.team4737.robot.vision;

import org.opencv.core.*;

import java.util.*;

import static org.opencv.imgproc.Imgproc.*;

/**
 * @author Brian Semrau
 * @version 1/15/2016
 */
public class BallDetector {

    public BallDetector() {

    }

    public Mat detectBall(Mat image, boolean drawDebug) {
        // Get S component of HSV converted image
        Mat rgb = new Mat();
        cvtColor(image, rgb, COLOR_YUV2RGB);
        Mat hsv = new Mat();
        cvtColor(rgb, hsv, COLOR_RGB2HSV);
        List<Mat> hsvSplits = new ArrayList<>();
        Core.split(hsv, hsvSplits);

        // Generate gray filter
        Mat grayFilter = new Mat();
        threshold(hsvSplits.get(1), grayFilter, 150, 255, THRESH_BINARY_INV);
//        cvtColor(grayFilter, grayFilter,);

        // Extract the gray (value) component
        List<Mat> srcSplits = new ArrayList<>();
        Core.split(image, srcSplits);

        Mat gray = srcSplits.get(0);

        //penispienispenis
        // - Jared Grimes, 2016

        // Apply filter
//        Core.bitwise_and(gray, grayFilter, gray);

        // Blur the image
        blur(gray, gray, new Size(13, 13));

        // Do Canny edge detection
        Mat canny = new Mat();
        Canny(gray, canny, 50, 5000, 7, true);

        // Find the contours of all the edges
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(canny, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        List<double[]> circles = new ArrayList<>();

        for (int i = 0; i < contours.size(); i++) {
            // Convert the contour to points
            Point[] points = contours.get(i).toArray();

            MatOfPoint2f contour = new MatOfPoint2f(points);
//            double epsilon = 0.01 * arcLength(contour, false);
//            approxPolyDP(contour, contour, epsilon, false);

//            points = contour.toArray();

            if (points.length >= 30 && arcLength(contour, false) > 10) {

//                drawContours(image, contours, i, new Scalar(0, 255, 0), 1);

                Scalar color = new Scalar(Math.random() * 255, Math.random() * 255, Math.random() * 255);

                int pointDist = 20;
                for (int j = 0; j < points.length - pointDist * 3; j++) {
                    Point a = points[j];
                    Point b = points[j + pointDist];
                    Point c = points[j + pointDist * 2];
                    double[] circle = circleFromPoints(a, b, c);

                    // Filter out tiny/large circles
                    if (circle[2] < 30 || circle[2] > 100) {
                        continue;
                    }

                    circles.add(circle);

                    if (drawDebug) {
                        Point center = new Point(circle[0], circle[1]);
//                        circle(image, center, (int) circle[2], new Scalar(128, 0, 0));
                        rectangle(image, center, center, color, 2);
                    }
                }

            } else {
//                drawContours(image, contours, i, new Scalar(255, 0, 255), 1);
            }

        }

        // Remove outlying circles
        if (circles.size() > 0) {
            double outlierDist = 50;
            { // Remove X outliers
                circles.sort((o1, o2) -> o1[0] > o2[0] ? 1 : o1[0] == o2[0] ? 0 : -1);
                double median = circles.get(circles.size() / 2)[0];
                List<double[]> toRemove = new ArrayList<>();
                for (double[] circle : circles) {
                    if (circle[0] < median - outlierDist || circle[0] > median + outlierDist)
                        toRemove.add(circle);
                }
                circles.removeAll(toRemove);
            }
            { // Remove Y outliers
                circles.sort((o1, o2) -> o1[1] > o2[1] ? 1 : o1[1] == o2[1] ? 0 : -1);
                double median = circles.get(circles.size() / 2)[1];
                List<double[]> toRemove = new ArrayList<>();
                for (double[] circle : circles) {
                    if (circle[1] < median - outlierDist || circle[1] > median + outlierDist)
                        toRemove.add(circle);
                }
                circles.removeAll(toRemove);
            }
            double outlierRadius = 50;
            { // Remove R outliers
                circles.sort((o1, o2) -> o1[2] > o2[2] ? 1 : o1[2] == o2[2] ? 0 : -1);
                double median = circles.get(circles.size() / 2)[2];
                List<double[]> toRemove = new ArrayList<>();
                for (double[] circle : circles) {
                    if (circle[1] < median - outlierRadius || circle[2] > median + outlierRadius)
                        toRemove.add(circle);
                }
                circles.removeAll(toRemove);
            }

            // Find average of meaningful circles
            double x = 0;
            double y = 0;
            double r = 0;
            {
                for (double[] circle : circles) {
                    x += circle[0];
                    y += circle[1];
                    r += circle[2];
                }
                x /= circles.size();
                y /= circles.size();
                r /= circles.size();
            }

            // Calculate spread of data
            double deviationSqrSum = 0;
            for (double[] circle : circles) {
                double dx = (x - circle[0]);
                double dy = (y - circle[1]);
                double distSqr = dx * dx + dy * dy;
                deviationSqrSum += distSqr;
            }
            deviationSqrSum /= (circles.size() * circles.size());
            System.out.println(deviationSqrSum);

            // Draw the found ball
            drawMarker(image, new Point(x, y), new Scalar(0, 0, 255));
            boolean isBallValid = deviationSqrSum < 10 && deviationSqrSum > 1 && circles.size() > 10;
            if (isBallValid) {
                // Draw circles that survived filtering
//                for (double[] circle : circles) {
//                    circle(image, new Point(circle[0], circle[1]), (int) r, new Scalar(0, 255 - deviationSqrSum / 5, 0));
//                }

                circle(image, new Point(x, y), (int) r, new Scalar(0, 255, 0));
            }
        }

        return image;
    }

    private double[] circleFromPoints(final Point p1, final Point p2, final Point p3) {
        final double offset = Math.pow(p2.x, 2) + Math.pow(p2.y, 2);
        final double bc = (Math.pow(p1.x, 2) + Math.pow(p1.y, 2) - offset) / 2.0;
        final double cd = (offset - Math.pow(p3.x, 2) - Math.pow(p3.y, 2)) / 2.0;
        final double det = (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p2.y);

        // This removes very large circles
        if (Math.abs(det) < 0.0000001) {
            return new double[]{0, 0, 0};
        }

        final double idet = 1 / det;

        final double centerx = (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
        final double centery = (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
        final double radius =
                Math.sqrt(Math.pow(p2.x - centerx, 2) + Math.pow(p2.y - centery, 2));

        return new double[]{centerx, centery, radius};
    }

}
