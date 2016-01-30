package org.usfirst.frc.team4737.robot.vision.balldetect;

import org.opencv.core.*;

import java.util.*;

import static org.opencv.imgproc.Imgproc.*;

/**
 * @author Brian Semrau
 * @version 1/15/2016
 */
public class BallDetector {

    private Mat debugImage;
    private boolean drawFilteredCircles;
    private boolean drawAllPoints;
    private boolean drawBalls;

    public BallDetector(boolean drawFilteredCircles, boolean drawAllPoints, boolean drawBalls) {
        this.drawFilteredCircles = drawFilteredCircles;
        this.drawAllPoints = drawAllPoints;
        this.drawBalls = drawBalls;
    }

    /**
     * Finds all the boulders in the given image
     *
     * @param sourceImageRGB        The source image given, in an RGB format
     * @param scale                 The amount to divide the image by, in order to increase processing speed
     * @param blurSize              The size of the blur filter used before Canny edge filtering
     * @param cannyLow              The low threshold for Canny filtering
     * @param cannyHigh             The high threshold for Canny filtering
     * @param pointDist             The distance between edge points used to find circles from the edges
     * @param filtRadMin            The low radius threshold bound for filtering out circles
     * @param filtRadMax            The high radius threshold bound for filtering out circles
     * @param binSize               The size of bins to filter data points into
     * @param outsideBinBorderDepth The number of bins to stretch outside the image for managing points lying outside.
     * @param minBinVolume          The minimum number of points inside a bin. Anything less, and the points will be deleted.
     * @param outlierDist           The distance used to remove outliers from calculating the centerpoint of the ball
     * @param maxDeviation          The maximum deviation of points defining a ball before it is declared too deviant.
     * @return Returns an array of circles defining the found boulders in the image.
     */
    public Circle[] findBoulders(Mat sourceImageRGB, int scale, int blurSize, int cannyLow, int cannyHigh,
                                 int pointDist, int filtRadMin, int filtRadMax, int binSize, int outsideBinBorderDepth,
                                 int minBinVolume, int outlierDist, double maxDeviation) {

        // ##############################
        // Image filtering
        // ##############################

        // Resize the image
        Mat resized = new Mat();
        resize(sourceImageRGB, resized, new Size(sourceImageRGB.width() / scale, sourceImageRGB.height() / scale));

        debugImage = resized.clone();

        // Convert to gray scale for Canny edge detection
        Mat gray = new Mat();
        cvtColor(resized, gray, COLOR_RGB2GRAY);

        // Blur the image for Canny edge detection
        Mat blurred = new Mat();
        blur(gray, blurred, new Size(blurSize, blurSize));

        // Perform edge detection
        Mat canny = new Mat();
        Canny(blurred, canny, cannyLow, cannyHigh);

        // Find contours of edges
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(canny, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

        // #################################
        // Calculate circles from contours
        // #################################

        // Calculate circles fitting on each contour
        ArrayList<Circle> circleList = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {

            MatOfPoint contour = contours.get(i);
            Point[] points = contour.toArray();

            for (int j = 0; j < points.length - pointDist * 3; j++) {
                // Calculate circle from three points
                Point a = points[j];
                Point b = points[j + pointDist];
                Point c = points[j + pointDist * 2];

                Circle circle = new Circle(a, b, c);

                // Filter circle by radius
                if (circle.r >= filtRadMin && circle.r < filtRadMax) {
                    if (drawFilteredCircles)
                        circle(debugImage, circle.center(), (int) circle.r, new Scalar(255, 0, 0));
                    circleList.add(circle);
                }
            }

        }

        // Draw all the data points
        if (drawAllPoints)
            for (Circle circle : circleList) {
                rectangle(debugImage, circle.center(), circle.center(), new Scalar(0, 0, 255));
            }

        // ##############################
        // Data Filtering
        // Detect balls
        // ##############################

        ArrayList<Circle> balls = new ArrayList<>();

        while (true) {
            // Create bins for data filtering
            int w = (resized.width() / binSize) + outsideBinBorderDepth * 2;
            int h = (resized.height() / binSize) + outsideBinBorderDepth * 2;

            // noinspection unchecked (There is an unavoidable type check error with arrays of ArrayLists)
            ArrayList<Circle>[] bins = new ArrayList[w * h];
            for (int i = 0; i < bins.length; i++)
                bins[i] = new ArrayList<>();

            // Sort circles into bins
            for (Circle circle : circleList) {
                int x = (int) (circle.x / binSize) + outsideBinBorderDepth;
                int y = (int) (circle.y / binSize) + outsideBinBorderDepth;
                if (x >= 0 && x < w && y >= 0 && y < h) {
                    bins[x + y * w].add(circle);
                }
            }

            // Filter out bins with really low volume
            for (int i = 0; i < bins.length; i++) {
                if (bins[i].size() < minBinVolume) {
                    circleList.removeAll(bins[i]);
                    bins[i].clear();
                }
            }

            // DEBUG
//            for (int x = 0; x < w; x++) {
//                for (int y = 0; y < h; y++) {
//                    int val = bins[x + y * w].size() * 5;
//                    if (val == 0) val = 128;
//                    rectangle(
//                            debugImage,
//                            new Point((x - outsideBinBorderDepth) * binSize, (y - outsideBinBorderDepth) * binSize),
//                            new Point((x - outsideBinBorderDepth + 1) * binSize, (y - outsideBinBorderDepth + 1) * binSize),
//                            new Scalar(0, 255 - val, val),
//                            Core.FILLED);
//                }
//            }
            // END DEBUG

            // Find the bin with the largest size
            int largestBinIndex = 0;
            {
                int largestBinSize = 0;
                for (int i = 0; i < bins.length; i++) {
                    if (bins[i].size() > largestBinSize) {
                        largestBinSize = bins[i].size();
                        largestBinIndex = i;
                    }
                }
            }

            // Find bias of points inside the bin
            Point avgLocationBin = findAverageLoc(bins[largestBinIndex]);

            // Filter points in the region around the bias of the bin
            ArrayList<Circle> regionalCircles = removeOutliers2(circleList, avgLocationBin, binSize * binSize);

            // Find the average point location from the region
            Point avgLocationReg = findAverageLoc(regionalCircles);

            // Filter out outliers from the region
            ArrayList<Circle> ballDefiningCircles = removeOutliers2(circleList, avgLocationReg, outlierDist * outlierDist);

            for (Circle circle : ballDefiningCircles) {
                rectangle(debugImage, circle.center(), circle.center(), new Scalar(0, 255, 255));
            }

            // Calculate deviation of the region
            double deviation = calculateSqrDeviation(ballDefiningCircles, avgLocationReg);

            // Determine if deviation is low enough
            if (deviation < maxDeviation) {
                // Calculate average radius
                double avgRadius = 0;
                for (Circle circle : circleList)
                    avgRadius += circle.r;
                avgRadius /= circleList.size();

                // Add the ball
                balls.add(new Circle(avgLocationReg, avgRadius));

                // Remove used data points
                circleList.removeAll(ballDefiningCircles);
            } else {
                break;
            }
        }

        // DEBUG
//        for (Circle circle : circleList) {
//            rectangle(debugImage, circle.center(), circle.center(), new Scalar(255, 0, 0));
//        }
        // END DEBUG

        // Draw found balls
        if (drawBalls)
            for (Circle circle : balls) {
                circle(debugImage, circle.center(), (int) circle.r, new Scalar(0, 255, 0));
                drawMarker(debugImage, circle.center(), new Scalar(0, 0, 255));
            }

        // Return an array
        return balls.toArray(new Circle[balls.size()]);
    }

    private double calculateSqrDeviation(List<Circle> circleList, Point center) {
        // Calculate spread of data

        if (circleList.size() == 0) return Double.POSITIVE_INFINITY;

        double deviationSqrSum = 0;

        for (Circle circle : circleList) {
            double dx = (center.x - circle.x);
            double dy = (center.y - circle.y);
            double distSqr = dx * dx + dy * dy;

            deviationSqrSum += distSqr;
        }

        return deviationSqrSum / (circleList.size() * circleList.size());
    }

    private Point findAverageLoc(ArrayList<Circle> circleList) {
        double x = 0;
        double y = 0;
        for (Circle circle : circleList) {
            x += circle.x;
            y += circle.y;
        }
        return new Point(x / circleList.size(), y / circleList.size());
    }

    private ArrayList<Circle> removeOutliers2(List<Circle> circleList, Point boundsCenter, double radiusSqr) {
        ArrayList<Circle> output = new ArrayList<>();

        for (Circle circle : circleList) {
            double dx = circle.x - boundsCenter.x;
            double dy = circle.y - boundsCenter.y;
            double sqrDist = dx * dx + dy * dy;

            if (sqrDist <= radiusSqr)
                output.add(circle);
        }

        return output;
    }

    /**
     * Removes outlying points from the given bounds.
     *
     * @param circleList   The list of circles to analyze
     * @param boundsCenter The center of the circular region defining what is an outlier or not
     * @param radiusSqr    The squared radius of the circular region defining what is an outlier or not
     * @return the list of removed points
     */
    private ArrayList<Circle> removeOutliers(List<Circle> circleList, Point boundsCenter, double radiusSqr) {
        ArrayList<Circle> toRemove = new ArrayList<>();

        for (Circle circle : circleList) {
            double dx = circle.x - boundsCenter.x;
            double dy = circle.y - boundsCenter.y;
            double sqrDist = dx * dx + dy * dy;

            if (sqrDist > radiusSqr)
                toRemove.add(circle);
        }

        circleList.removeAll(toRemove);

        return toRemove;
    }

    public Mat getDebugImg() {
        return debugImage;
    }

}
