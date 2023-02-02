package org.firstinspires.ftc.teamcode.SimpleExampleOpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 */
public class TestBotHardware
{
    /* Public OpMode members. */
    public DcMotor  lMotor   = null;
    public DcMotor  rMotor  = null;
    public Servo    lClaw    = null;
    public Servo    rClaw   = null;

    /* FTCDashboard Coefficients */
    public static double MAX_SPEED = 1;
    public static double CLAW_CENTER = 0.5;
    public static double CLAW_GAP = 0.5;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime clock  = new ElapsedTime();

    /* Constructor */
    public TestBotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lMotor  = hwMap.get(DcMotor.class, "lf");
        rMotor = hwMap.get(DcMotor.class, "rf");

        lMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        lMotor.setPower(0);
        rMotor.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        lClaw  = hwMap.get(Servo.class, "lclaw");
        rClaw = hwMap.get(Servo.class, "rclaw");
        lClaw.setPosition(CLAW_CENTER);
        rClaw.setPosition(CLAW_CENTER);
    }
}