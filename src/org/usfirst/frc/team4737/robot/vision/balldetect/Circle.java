package org.usfirst.frc.team4737.robot.vision.balldetect;

import org.opencv.core.Point;

/**
 * @author Brian Semrau
 * @version 1/15/2016
 */
public class Circle {

    public double x, y;
    public double r;

    public Circle() {
    }

    public Circle(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public Circle(double[] params) {
        if (params.length != 3) throw new IllegalArgumentException("Circle must have three parameters: x, y, and r.");

        x = params[0];
        y = params[0];
        r = params[0];
    }

    public Circle(Point a, Point b, Point c) {
        final double offset = Math.pow(b.x, 2) + Math.pow(b.y, 2);
        final double bc = (Math.pow(a.x, 2) + Math.pow(a.y, 2) - offset) / 2.0;
        final double cd = (offset - Math.pow(c.x, 2) - Math.pow(c.y, 2)) / 2.0;
        final double det = (a.x - b.x) * (b.y - c.y) - (b.x - c.x) * (a.y - b.y);

        // This removes very large circles
        if (Math.abs(det) < 0.0000001) {
            return;
        }

        final double idet = 1 / det;

        final double centerx = (bc * (b.y - c.y) - cd * (a.y - b.y)) * idet;
        final double centery = (cd * (a.x - b.x) - bc * (b.x - c.x)) * idet;
        final double radius =
                Math.sqrt(Math.pow(b.x - centerx, 2) + Math.pow(b.y - centery, 2));

        this.x = centerx;
        this.y = centery;
        this.r = radius;
    }

    public Point center() {
        return new Point(x, y);
    }

}
