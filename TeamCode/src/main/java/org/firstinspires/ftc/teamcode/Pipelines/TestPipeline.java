package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 255, 255);
    public TestPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    Mat inputGray = new Mat();
    Mat inputHSV = new Mat();
    Mat inputHSV_Range = new Mat();
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
    @Override
    public void init(Mat input) {
        // Executed before the first call to processFrame for subframes and other presistent frames.
        //inputGray = input; //needed?
        //inputHSV = input;
        //inputHSV_Range = input;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched
        Imgproc.cvtColor(input,inputGray,Imgproc.COLOR_RGB2GRAY);
        Imgproc.rectangle(input,new Point(140,20), new Point(180,50),new Scalar(255,0,0),3);
        Imgproc.cvtColor(input,inputHSV,Imgproc.COLOR_RGB2HSV);
        Core.inRange(inputHSV,lower,upper,inputHSV_Range);

        Imgproc.erode(inputHSV_Range,inputHSV_Range,erodeElement);
        Imgproc.erode(inputHSV_Range,inputHSV_Range,erodeElement);
        Imgproc.dilate(inputHSV_Range,inputHSV_Range,dilateElement);
        Imgproc.dilate(inputHSV_Range,inputHSV_Range,dilateElement);

        telemetry.addData("Value of Image:",inputGray.get(105,125)[0]);
        telemetry.addData("Value of HSV Image:",inputHSV_Range.get(20,120)[0]);
        telemetry.update();
        return input; // Return the image that will be displayed in the viewport
        // (In this case the input mat directly)
    }


    @Override
    public void onViewportTapped() {
        // Executed when the image display is clicked by the mouse or tapped
        // This method is executed from the UI thread, so be careful to not
        // perform any sort heavy processing here! Your app might hang otherwise
    }

}

