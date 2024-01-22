package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;



public class RedScanner extends OpenCvPipeline {

    Timer t = new Timer();

    Mat mat = new Mat();
    Mat leftMat = new Mat();
    Mat midMat = new Mat();
    Mat rightMat = new Mat();

    // 320 x 240 Resolution
    Rect leftROI = new Rect(new Point(0, 600), new Point(1280/10.0, 250));
    Rect midROI = new Rect(new Point(1280 / 10.0, 600), new Point(2 * 1280 / 3.0, 250));
    Rect rightROI = new Rect(new Point(2 * 1280 / 3.0, 600), new Point(1280, 250));

    private Barcode result = null;
    private Telemetry telemetry;

    private double leftValue, midValue, rightValue;

    public RedScanner(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        // H 6, 52 :: 6/2, 52/2 - Range of Orange
        // S 100 255
        // V 100 255
        Scalar lowerBound = new Scalar(6 / 2.0, 100, 100);
        Scalar upperBound = new Scalar(52 / 2.0, 255, 255);
        Core.inRange(mat, lowerBound, upperBound, mat);

        leftMat = mat.submat(leftROI);
        midMat = mat.submat(midROI);
        rightMat = mat.submat(rightROI);

         leftValue = Core.sumElems(leftMat).val[0];
         midValue = Core.sumElems(midMat).val[0];
         rightValue = Core.sumElems(rightMat).val[0];
        telemetry.addData("Left", leftValue);
        telemetry.addData("Middle", midValue);
        telemetry.addData("Right", rightValue);

        leftMat.release();
        midMat.release();
        rightMat.release();

        double maxValue = Math.max(leftValue, Math.max(midValue+100000, rightValue));
        telemetry.addData("max value", maxValue);
        Scalar matchColor = new Scalar(0, 255, 0);
        Scalar mismatchColor = new Scalar(255, 0, 0);

        if (maxValue == rightValue) {
            result = Barcode.RIGHT;
            Imgproc.rectangle(input, leftROI, mismatchColor);
            Imgproc.rectangle(input, midROI, mismatchColor);
            Imgproc.rectangle(input, rightROI, matchColor);
        } else if (maxValue-100000 == midValue && maxValue != 100000) {
            result = Barcode.MIDDLE;
            Imgproc.rectangle(input, leftROI, mismatchColor);
            Imgproc.rectangle(input, midROI, matchColor);
            Imgproc.rectangle(input, rightROI, mismatchColor);
        } else {
            result = Barcode.LEFT;
            Imgproc.rectangle(input, leftROI, matchColor);
            Imgproc.rectangle(input, midROI, mismatchColor);
            Imgproc.rectangle(input, rightROI, mismatchColor);
        }

        telemetry.addData("Barcode", result.toString().toLowerCase());
        telemetry.update();
        if (rightValue * 2 >= midValue && rightValue > 1000000) {
            result = Barcode.LEFT;
        }
        return input;
    }

    public Barcode getResult(double time) {
        double cTime = t.getTimeSys();
        while (cTime + time > t.getTimeSys());
        return result;
    }
}