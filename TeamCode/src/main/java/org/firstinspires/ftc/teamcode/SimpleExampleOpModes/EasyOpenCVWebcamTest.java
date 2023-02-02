/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.SimpleExampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EasyOpenCVExamples.PipelineStageSwitchingExample;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "EasyOpenCV Webcam Test", group = "Concept")
//@Disabled
public class EasyOpenCVWebcamTest extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  OpenCvWebcam webcam;

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
    //Determines where we view the image from the camera.  This one is the Driver Station location.
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

    //instantiates the camera and sets how we will view the image.
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    // To not view the image...
    //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

    //specifies the pipeline that does something to each image that comes from the camera.
    webcam.setPipeline(new EasyOpenCVWebcamTest.SamplePipeline());

    //Opens the connection to the camera.
    webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
      @Override
      public void onOpened()
      {
        //starts the webcam streaming.  You must specifiy the resolution.
        //supported resolutions: 320x240 (>30 fps), 640x480 (30 fps), 1280x720 (10 fps)
        //the higher the resolution the lower the frame rate
        //also sets the orientation of the image.
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
      }

      @Override
      public void onError(int errorCode)
      {
        /*
         * This will be called if the camera could not be opened
         */
      }
    });

    telemetry.addLine("Waiting for start");
    telemetry.update();
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    telemetry.addData("Frame Count", webcam.getFrameCount());
    telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
    telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
    telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
    telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
    telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
    telemetry.update();

    if(gamepad1.x)
    {
      webcam.stopStreaming();
      //webcam.closeCameraDevice();
    }
    if(gamepad1.b){
      webcam.pauseViewport();
    }
    if(gamepad1.a){
      webcam.resumeViewport();
    }


    //TODO FIX SLEEP? Probably note needed
  }


  //called once
  @Override
  public void start() {
    runtime.reset();
  }

  //main loop after start
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }

  class BasePipeline extends OpenCvPipeline
  {
    //variables to declare

    @Override
    public void init(Mat input){
      //code to run to set up submats or other variables
    }

    @Override
    public Mat processFrame(Mat input) {
      return null;
    }
  }

  class SamplePipeline extends OpenCvPipeline
  {
    boolean viewportPaused = false;

    @Override
    public Mat processFrame(Mat input)
    {

      Imgproc.rectangle(
              input,
              new Point(
                      input.cols()/8,
                      input.rows()/8),
              new Point(
                      input.cols()*(3f/4f),
                      input.rows()*(3f/4f)),
              new Scalar(0, 255, 0), 4);



      /**
       * NOTE: to see how to get data from your pipeline to your OpMode as well as how
       * to change which stage of the pipeline is rendered to the viewport when it is
       * tapped, please see {@link PipelineStageSwitchingExample}
       */

      return input;
    }

    class ConvertToGreyPipeline extends OpenCvPipeline
    {
      Mat grey = new Mat();

      @Override
      public Mat processFrame(Mat input){
        Imgproc.cvtColor(input,grey,Imgproc.COLOR_RGB2GRAY);
        return grey;
      }
    }

    //other pipeline
    class SubmatPipeline extends OpenCvPipeline
    {
      // Notice this is declared as an instance variable (and re-used), not a local variable
      Mat submat;

      @Override
      public void init(Mat firstFrame)
      {
        submat = firstFrame.submat(0,50,0,50);
      }

      @Override
      public Mat processFrame(Mat input)
      {
        // Because a submat is a persistent reference to a region of the parent buffer,
        // (which in this case is `input`) any changes to `input` will be reflected in
        // the submat (and vice versa).
        return submat;
      }
    }

    @Override
    public void onViewportTapped()
    {
      viewportPaused = !viewportPaused;
      if(viewportPaused) {webcam.pauseViewport();}
      else {webcam.resumeViewport();}
    }
  }
}
