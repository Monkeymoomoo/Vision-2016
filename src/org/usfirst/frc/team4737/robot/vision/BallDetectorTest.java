package org.usfirst.frc.team4737.robot.vision;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.*;

import static org.opencv.imgproc.Imgproc.*;
import static org.opencv.core.Core.*;

/**
 * @author Brian Semrau
 * @version 1/12/2016
 */
public class BallDetectorTest {

    static {
        System.load(new File("libs/opencv_java310.dll").getAbsolutePath());
    }

    public static void main(String[] args) {

        Mat src_img = new Mat();
        resize(Imgcodecs.imread("test/Capture.PNG"), src_img, new Size(960, 720));

        Mat gray = new Mat();
        cvtColor(src_img, gray, COLOR_RGB2GRAY);

        blur(gray, gray, new Size(7, 7));
        blur(gray, gray, new Size(7, 7));
        blur(gray, gray, new Size(7, 7));
        blur(gray, gray, new Size(7, 7));

        Mat canny = new Mat();
        Canny(gray, canny, 5, 11000, 7, true);

        showResult(canny);

        Mat circles = new Mat();
        HoughCircles(canny, circles, CV_HOUGH_GRADIENT, 1, gray.height() / 3, 250, 100, 100, 500);

        for (int i = 0; i < circles.cols(); i++) {
            double[] circle = circles.get(0, i);
            Point center = new Point(circle[0], Math.round(circle[1]));
            int radius = (int) Math.round(circle[2]);
            // circle center
            circle(src_img, center, 3, new Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(src_img, center, radius, new Scalar(0, 0, 255), 3, 8, 0);
        }

        showResult(src_img);

    }

    public static void hsvTest() {
        Mat image = Imgcodecs.imread("test/Capture.PNG");

        Mat hsv = new Mat(image.rows(), image.cols(), image.type());
        cvtColor(image, hsv, COLOR_RGB2HSV);

        List<Mat> planes = new ArrayList<>();
        Core.split(hsv, planes);

        showResult(planes.get(1));
    }

    public static void showResult(Mat img) {
        MatOfByte matOfByte = new MatOfByte();
        Imgcodecs.imencode(".png", img, matOfByte);
        byte[] byteArray = matOfByte.toArray();
        BufferedImage bufImage;
        try {
            InputStream in = new ByteArrayInputStream(byteArray);
            bufImage = ImageIO.read(in);
            JFrame frame = new JFrame();
            frame.getContentPane().add(new JLabel(new ImageIcon(bufImage)));
            frame.pack();
            frame.setVisible(true);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
