package org.usfirst.frc.team4737.robot.vision;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;
import org.usfirst.frc.team4737.robot.vision.balldetect.*;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.*;

import static org.opencv.imgproc.Imgproc.*;

/**
 * @author Brian Semrau
 * @version 1/11/2016
 */
public class Vision {

    private VideoCapture video;

    private boolean feedPaused;

    private JFrame frame;
    private JLabel label;

    private BallDetector2 ballDetector;

    public Vision() {
        video = new VideoCapture("test/capture2.avi");

        ballDetector = new BallDetector2 ();

        frame = new JFrame();
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.getContentPane().add(label = new JLabel());
        frame.setVisible(true);

        frame.addKeyListener(new KeyListener() {
            @Override
            public void keyTyped(KeyEvent e) {
            }

            @Override
            public void keyPressed(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_SPACE)
                    feedPaused = !feedPaused;
            }

            @Override
            public void keyReleased(KeyEvent e) {
            }
        });
    }

    public void start() {
        long frameLength = 1000000000 / 30;
        long currentTime;
        long previousTime = System.nanoTime();

        while (true) {
            currentTime = System.nanoTime();
            if (currentTime - previousTime < frameLength || feedPaused)
                continue;

            // Grab image from video feed
            Mat sourceImage = new Mat();
            video.grab();
            video.retrieve(sourceImage);

            // Check if it is a valid image
            if (sourceImage.dims() < 2)
                break;

            // Convert to RGB
            Mat rgbImage = new Mat();
            cvtColor(sourceImage, rgbImage, COLOR_YUV2RGB);

            // Detect balls
            Circle[] balls = ballDetector.findBoulders(sourceImage);

            // Update image
            Mat img = ballDetector.getDebugImg();
            updateImage(img);

            previousTime = currentTime;
        }
        System.out.println("Video finished.");
    }

    public void updateImage(Mat image) {
        if (image == null) return;

        MatOfByte matOfByte = new MatOfByte();
        Imgcodecs.imencode(".png", image, matOfByte);
        byte[] byteArray = matOfByte.toArray();
        BufferedImage bufImage;
        int scale = 7;
        try {
            InputStream in = new ByteArrayInputStream(byteArray);
            bufImage = ImageIO.read(in);
            label.setIcon(new ImageIcon(bufImage.getScaledInstance(bufImage.getWidth() * scale, bufImage.getHeight() * scale, Image.SCALE_FAST)));
            frame.setSize(image.width() * scale, image.height() * scale);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        System.load(new File("libs/opencv/build/java/x64/opencv_java310.dll").getAbsolutePath());
        System.load(new File("libs/opencv/build/bin/opencv_ffmpeg310_64.dll").getAbsolutePath());

        new Vision().start();
    }

}
