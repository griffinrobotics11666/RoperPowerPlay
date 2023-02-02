package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DrawShapesPipeline extends OpenCvPipeline {

    //required to send telemetry
    Telemetry telemetry;
    public DrawShapesPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    //end required to send telemetry.

    //public values become sliders to modify to tune
    public Point rectp1 = new Point(100,50);
    public Point rectp2 = new Point(150,100);
    public Point linep1 = new Point(175,25);
    public Point linep2 = new Point(200,150);
    public Point circleCenter = new Point (100,100);
    public int radius = 20;

    public Scalar rectColor = new Scalar(255,0,0);
    public int rectThickness = 2;
    public Scalar lineColor = new Scalar(0,255,0);
    public int lineThickness = 4;
    public Scalar circleColor = new Scalar(0,0,255);
    public int circleThickness = 3;

    Mat B = new Mat();

    //Mat objects for a submat defined by the rectangular region
    Mat rectRegionB = new Mat();
    Mat rectRegion = new Mat();
    @Override
    public void init(Mat input) {
        // Executed before the first call to processFrame for subframes and other presistent frames.
        //extracts just the blue part of the image
        Core.extractChannel(input,B,2);
        //extracts just the rectangular region of the submat of the blue image
        rectRegionB = B.submat(new Rect(rectp1,rectp2));
        //extracts just the rectangular region of the submat
        rectRegion = input.submat(new Rect(rectp1,rectp2));
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched
        //Draw
        Imgproc.rectangle(input,rectp1,rectp2,rectColor,rectThickness);
        Imgproc.line(input,linep1,linep2,lineColor,lineThickness);
        Imgproc.circle(input,circleCenter,radius,circleColor,circleThickness);
        rectRegion = input.submat(new Rect(rectp1,rectp2));
        Core.extractChannel(input,B,2);
        double BavgB = Core.mean(rectRegionB).val[0];
        double avgR = Core.mean(rectRegion).val[0];
        double avgG = Core.mean(rectRegion).val[1];
        double avgB = Core.mean(rectRegion).val[2];
        telemetry.addData("B img Rect Mean B ", BavgB);
        telemetry.addData("img Rect Mean R ", avgR);
        telemetry.addData("img Rect Mean G ", avgG);
        telemetry.addData("img Rect Mean B ", avgB);

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

