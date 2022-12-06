package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MM_EOCVSleeveDetection extends OpenCvPipeline {

    private static int RED = 0;
    private static int BLUE = 1;
    private static int YELLOW = 2;

    private int maxColor = RED;

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

        int loopCount = 0;
        double mean[] = new double[3];
        while(loopCount <= 2) {
            Mat thresh = new Mat();
            Core.inRange(mat, lowerBoundColor(loopCount), upperBoundColor(loopCount), thresh);

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
        return max;
    }

    public int columns() {
        return max.cols();
    }

    public int getMaxColor(){return maxColor;}

    public double getMean() {
        return Core.mean(max).val[0];
    }

    private Scalar upperBoundColor(int color) {
        if (color == RED) {
            return new Scalar(210, 200, 240);
        } else if (color == BLUE) {
            return new Scalar(135, 255, 200);
        } else {
            return new Scalar(35, 165, 255);
        }
    }

    private Scalar lowerBoundColor(int color) {
        if (color == RED) {
            return new Scalar(130, 110, 140);
        } else if (color == BLUE) {
            return new Scalar(75, 195, 100);
        } else {
            return new Scalar(10, 90, 165);
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
}