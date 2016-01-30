package org.usfirst.frc.team4737.robot.vision;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.*;
import org.usfirst.frc.team4737.robot.vision.balldetect.*;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;

import static org.opencv.imgproc.Imgproc.*;

/**
 * @author Brian Semrau
 * @version 1/11/2016
 */
public class Vision {

    // Cameras
    private VideoCapture camera0;
    private VideoCapture camera1;

    // GUI
    private JFrame frame;
    private JLabel imageLabel;

    // Ball detector variables
    private BallDetector ballDetector;
    private int blurSize = 6;
    private int cannyLow = 20;
    private int cannyHigh = 150;
    private int pointDist = 10;
    private int filtRadMin = 10;
    private int filtRadMax = 60;
    private int binSize = 20;
    private int outsideBinBorderDepth = 3;
    private int minBinVolume = 5;
    private int outlierDist = 30;
    private double maxDensitySpread = 2.5;

    public Vision() {
        // Create cameras
        camera0 = new VideoCapture(0);
//        camera1 = new VideoCapture(1);

        // Create ball detector
        ballDetector = new BallDetector(false, true, true);

        // Create GUI
        frame = new JFrame();
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

        imageLabel = new JLabel();
        frame.getContentPane().add(imageLabel);

        frame.setVisible(true);
    }

    public void start() {
        while (true) {

            // Grab image from video feed
            Mat sourceImage = new Mat();
            camera0.grab();
            camera0.retrieve(sourceImage);

            // Check if it is a valid image
            if (sourceImage.dims() < 2)
                continue;

            // Convert to RGB
            Mat rgbImage = new Mat();
            cvtColor(sourceImage, rgbImage, COLOR_YUV2RGB);

            // Detect balls
//            Circle[] balls = ballDetector.findBoulders(
//                    sourceImage,
//                    4, 4,
//                    49, 50,
//                    10,
//                    10, 60,
//                    20, 3,
//                    30,
//                    3);

            Circle[] balls = ballDetector.findBoulders(
                    sourceImage,
                    4, blurSize,
                    cannyLow, cannyHigh,
                    pointDist,
                    filtRadMin, filtRadMax,
                    binSize, outsideBinBorderDepth,
                    minBinVolume,
                    outlierDist,
                    maxDensitySpread);

            // Update image
            Mat img = ballDetector.getDebugImg();
            updateImage(img);
        }
//        System.out.println("Video finished.");
    }

    public void updateImage(Mat image) {
        if (image == null) return;

        MatOfByte matOfByte = new MatOfByte();
        Imgcodecs.imencode(".png", image, matOfByte);
        byte[] byteArray = matOfByte.toArray();
        BufferedImage bufImage;
        int scale = 4;
        try {
            InputStream in = new ByteArrayInputStream(byteArray);
            bufImage = ImageIO.read(in);
            imageLabel.setIcon(new ImageIcon(bufImage.getScaledInstance(bufImage.getWidth() * scale, bufImage.getHeight() * scale, Image.SCALE_FAST)));
            frame.pack();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        new Vision().start();
    }

}
