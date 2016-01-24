package org.usfirst.frc.team4737.robot.vision;

import cl.eye.CLCamera;
import org.opencv.core.*;

/**
 * Wrapper for the CLCamera class, allowing interfacing between OpenCL and a PS3Eye.
 *
 * @author Brian Semrau
 * @version 1/11/2016
 */
public class PS3Eye {

    public enum Mode {
        MONO(0), COLOR(1),
        MONO_RAW(2), COLOR_RAW(3),
        BAYER(4);

        public final int value;

        Mode(int value) {
            this.value = value;
        }
    }

    public enum Resolution {
        VGA(0), QVGA(1);

        public final int value;

        Resolution(int value) {
            this.value = value;
        }
    }

    public enum Param {
        AUTO_GAIN(0),
        GAIN(1),
        AUTO_EXPOSURE(2),
        EXPOSURE(3),
        AUTO_WHITEBALANCE(4),
        WHITEBALANCE_RED(5),
        WHITEBALANCE_GREEN(6),
        WHITEBALANCE_BLUE(7),
        HFLIP(8),
        VFLIP(9),
        HKEYSTONE(10),
        VKEYSTONE(11),
        XOFFSET(12),
        YOFFSET(13),
        ROTATION(14),
        ZOOM(15),
        LENSCORRECTION1(16),
        LENSCORRECTION2(17),
        LENSCORRECTION3(18),
        LENSBRIGHTNESS(19);

        public final int value;

        Param(int value) {
            this.value = value;
        }
    }

    private int index;
    private Mode mode;
    private Resolution resolution;
    private int framerate;
    private CLCamera camera;

    private int[] data;
    private int waitTimeout = 20;

    public PS3Eye(int index, Mode mode, Resolution resolution, int framerate) {
        this.index = index;
        this.mode = mode;
        this.resolution = resolution;
        this.framerate = framerate;

        camera = new CLCamera();

        camera.createCamera(index, mode.value, resolution.value, framerate);
    }

    public void setParam(Param parameter, int value) {
        camera.setCameraParam(parameter.value, value);
    }

    public int getParam(Param parameter) {
        return camera.getCameraParam(parameter.value);
    }

    public void setWaitTimeout(int millis) {
        waitTimeout = millis;
    }

    public void start() {
        camera.startCamera();
        System.out.println("Started PS3Eye Camera " + index + ".");
    }

    public Mat grab() {
        camera.getCameraFrame(data, waitTimeout);
        return new MatOfInt(data);
    }

    public void stop() {
        camera.stopCamera();
    }

    public void dispose() {
        camera.dispose();
    }

}
