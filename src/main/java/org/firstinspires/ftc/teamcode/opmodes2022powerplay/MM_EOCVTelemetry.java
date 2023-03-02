package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MM_EOCVTelemetry extends OpenCvPipeline {

    private Mat max = new Mat();
    private Mat cropped =  new Mat();
    public Mat processFrame(Mat frame) {

        //new matrix is created that process the image in RGB and converts to HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        //if they frame is empty, return the original frame
        if (mat.empty()) {
            return frame;
        }
        cropped = new Mat(mat, new Range(90, 115), new Range(90, 110));

        return cropped;
    }

    public int columns() {
        return max.cols();
    }

    public double getMean1() {
        return Core.mean(cropped).val[0];
    }

    public double getMean2() {
        return Core.mean(cropped).val[1];
    }

    public double getMean3() {
        return Core.mean(cropped).val[2];
    }
}