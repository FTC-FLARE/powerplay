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
    private double highestMean = 0;

    private Mat max = new Mat();
    private Mat right = new Mat();
    private Mat left = new Mat();
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
                cropped = new Mat(thresh, new Range(70, 135), new Range(75, 125));
                mean[loopCount] = Core.mean(cropped).val[0];
                if (loopCount == RED) {
                    max = cropped;
                    maxColor = RED;
                    highestMean = mean[RED];
                } else if (loopCount == BLUE && Math.max(mean[BLUE], mean[RED]) == mean[BLUE]) {
                    max = cropped;
                    maxColor = BLUE;
                    highestMean = mean[BLUE];
                } else if (loopCount == YELLOW && Math.max(mean[YELLOW], mean[BLUE]) == mean[YELLOW] && Math.max(mean[YELLOW], mean[RED]) == mean[YELLOW]) {
                    max = cropped;
                    maxColor = YELLOW;
                    highestMean = mean[YELLOW];
                }
                loopCount += 1;
            } if (highestMean < 10) {
                maxColor = 4;
            }
        } else if (mode == 2) {
            Mat thresh = new Mat();
            Core.inRange(mat, lowerBoundColorCone(coneColor), upperBoundColorCone(coneColor), thresh);
            cropped = new Mat(thresh, new Range(65, 165), new Range(135, 215));
            max = cropped;
        } else {
            Mat thresh = new Mat();
            Core.inRange(mat, lowerBoundColorCone(coneColor), upperBoundColorCone(coneColor), thresh);
            left = new Mat(thresh, new Range(65, 165), new Range(150, 190));
            right = new Mat(thresh, new Range(65, 165), new Range(190, 230));
        }
        return max;
    }

    public int getMaxColor(){return maxColor;}

    public double getMean() {
        return Core.mean(max).val[0];
    }

    public void changeMode(int mode) {
        this.mode =  mode;
    }

    private Scalar upperBoundColorSleeve(int color) {
        if (color == RED) {
            return new Scalar(210, 250, 250);
        } else if (color == BLUE) {
            return new Scalar(135, 255, 200);
        } else {
            return new Scalar(35, 165, 255);
        }
    }

    private Scalar lowerBoundColorSleeve(int color) {
        if (color == RED) {
            return new Scalar(160, 180, 170);
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
            return new Scalar(140, 100, 110);
        } else {
            return new Scalar(60, 190, 90);

        }
    }
}