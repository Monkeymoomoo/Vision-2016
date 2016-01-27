package org.usfirst.frc.team4737.robot.vision.opticalflow;

import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;
import org.opencv.videoio.VideoCapture;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;

/**
 * @author Brian Semrau
 * @version 1/25/2016
 */
public class OpticalFlowExample {

    static {
        System.load(new File("libs/opencv/build/java/x64/opencv_java310.dll").getAbsolutePath());
        System.load(new File("libs/opencv/build/x64/vc14/bin/opencv_ffmpeg310_64.dll").getAbsolutePath());
    }

    private static Mat drawFlow(Mat im, Mat flow) {
        float step = 16;
        int h = im.height();
        int w = im.width();

        Mat out = new Mat();
        Imgproc.cvtColor(im, out, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(out, out, Imgproc.COLOR_RGB2HSV);

        for (int x = 0; x < w; x += step) {
            for (int y = 0; y < h; y += step) {
                double[] delta = flow.get(y, x);
                Point point = new Point(x, y);

                double direction = Math.atan2(delta[1], delta[0]) * 180. / (Math.PI * 2);
                while (direction < 0) direction += 180;
                while (direction > 180) direction -= 180;
                double magnitude = Math.hypot(delta[0], delta[1]) * 10;
                Scalar color = new Scalar(
                        direction,
                        255,
                        magnitude);

//                Imgproc.rectangle(out, point, point, color);
                Imgproc.line(out, new Point(x, y), new Point(x + delta[0], y + delta[1]), new Scalar(0, 255, 0), 1);
                Imgproc.circle(out, new Point(x, y), 3, new Scalar(0, 255, 0), 1);
            }
        }

        // to hsv

        return out;
    }

    private static JFrame frame;
    private static JLabel label;

    static {
        frame = new JFrame();
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.getContentPane().add(label = new JLabel());
        frame.setVisible(true);
    }

    private static void updateImage(Mat image) {
        if (image == null || image.width() == 0 || image.height() == 0) return;

        MatOfByte matOfByte = new MatOfByte();
        Imgcodecs.imencode(".PNG", image, matOfByte);
        byte[] byteArray = matOfByte.toArray();
        BufferedImage bufImage;
        int scale = 2;
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

        // Setup capture
        VideoCapture cap = new VideoCapture(1);

        Mat im = new Mat();
        Mat prevGray = new Mat();
        Mat gray = new Mat();
        Mat flow = new Mat();
        Mat output = new Mat();

        // Get first frame
        while (!cap.grab()) ; // Wait until camera is ready

        cap.retrieve(im);
        Imgproc.cvtColor(im, prevGray, Imgproc.COLOR_RGB2GRAY);

        while (frame.isFocused()) {
            // Grab frame
            if (!cap.grab()) continue;
            cap.retrieve(im);
            Imgproc.cvtColor(im, gray, Imgproc.COLOR_RGB2GRAY);

            // Compute flow
            Video.calcOpticalFlowFarneback(prevGray, gray, flow, 0.55, 3, 15, 3, 5, 1.2, 0);
            prevGray = gray.clone();

            // Draw the flow vectors
            output = drawFlow(gray, flow);
            updateImage(output);

        }

    }

}
