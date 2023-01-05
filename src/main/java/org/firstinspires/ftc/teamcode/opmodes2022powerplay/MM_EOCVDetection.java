package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MM_EOCVDetection extends OpenCvPipeline {

    public static int RED = 0;
    public static int BLUE = 1;
    public static int YELLOW = 2;

    private int maxColor = RED;
    private int mode = 1;
    private int coneColor = RED;

    private Mat max = new Mat();
    private Mat cropped =  new Mat();
    public Mat processFrame(Mat frame) {

        //new matrix is created that process the image in RGB and converts to HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        //if they frame is empty, return the original frame
        if (mat.empty()) {
            maxColor = -1; //no max
            return frame;
        }

        if (mode == 1) {
            int loopCount = 0;
            double mean[] = new double[3];
            while(loopCount <= 2) {
                Mat thresh = new Mat();
                Core.inRange(mat, lowerBoundColorSleeve(loopCount), upperBoundColorSleeve(loopCount), thresh);

                cropped = new Mat(thresh, new Range(85, 145), new Range(135, 185));
                mean[loopCount] = Core.mean(cropped).val[0];
                if (loopCount == RED) {
                    max = cropped;
                    maxColor = RED;
                } else if (loopCount == BLUE && Math.max(mean[BLUE], mean[RED]) == mean[BLUE]) {
                    max = cropped;
                    maxColor = BLUE;
                } else if (loopCount == YELLOW && Math.max(mean[YELLOW], mean[BLUE]) == mean[YELLOW] && Math.max(mean[YELLOW], mean[RED]) == mean[YELLOW]) {
                    max = cropped;
                    maxColor = YELLOW;
                }
                loopCount += 1;
            }
        } else {
            Mat thresh = new Mat();
            Core.inRange(mat, lowerBoundColorCone(coneColor), upperBoundColorCone(coneColor), thresh);
            cropped = new Mat(thresh, new Range(65, 165), new Range(150, 230));
            max = cropped;
        }
        return max;
    }

    public int getMaxColor(){return maxColor;}

    public double getMean() {
        return Core.mean(max).val[0];
    }

    public void changeMode() {
        if (mode == 1) {
            mode = 2;
        } else {
            mode = 1;
        }
    }

    public void setConeColor(int coneColor) {
        this.coneColor = coneColor;
    }

    public boolean goodToCollect() {
        return Core.mean(max).val[0] > 120;
    }

    private Scalar upperBoundColorSleeve(int color) {
        if (color == RED) {
            return new Scalar(210, 200, 240);
        } else if (color == BLUE) {
            return new Scalar(135, 255, 200);
        } else {
            return new Scalar(35, 165, 255);
        }
    }

    private Scalar lowerBoundColorSleeve(int color) {
        if (color == RED) {
            return new Scalar(130, 110, 140);
        } else if (color == BLUE) {
            return new Scalar(75, 195, 100);
        } else {
            return new Scalar(10, 90, 165);
        }
    }

    private Scalar upperBoundColorCone(int color) {
        if (color == RED) {
            return new Scalar(210, 235, 235);
        } else {
            return new Scalar(145, 255, 210);
        }
    }

    private Scalar lowerBoundColorCone(int color) {
        if (color == RED) {
            return new Scalar(120, 80, 85);
        } else {
            return new Scalar(60, 190, 90);
        }
    }

    public String getMaxColorString() {
        if (maxColor == BLUE) {
            return "Blue";
        } else if (maxColor == RED) {
            return "Red";
        } else if (maxColor == YELLOW) {
            return "Yellow";
        } else {
            return "no max";
        }
    }

    //163, 221, 192
    //blue 107, 239, 139
    //120
}