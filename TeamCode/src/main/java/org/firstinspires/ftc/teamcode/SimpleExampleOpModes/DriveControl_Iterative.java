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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SS: Drive Control Iterative", group="Iterative Opmode")
//@Disabled
@Config
public class DriveControl_Iterative extends OpMode
{
    // Declare OpMode variables.
    private ElapsedTime runtime = new ElapsedTime();
    TestBotHardware bot = new TestBotHardware();
    FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25); //sets a frequency for transmission
        packet.put("Status","Initialized");             //FTC Dashboard Telemetry
        //telemetry.addData("Status", "Initialized");
        dashboard.sendTelemetryPacket(packet);

        bot.init(hardwareMap);
        // Tell the driver that initialization is complete.
        //telemetry.addData("Status", "Initialized");
        packet.put("Status", "Done");
        dashboard.sendTelemetryPacket(packet);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        packet.put("Time in Init: (%.1f)",runtime.seconds());
        //telemetry.addData("Time in Init: ","(%.1f)", runtime.seconds());
        packet.addLine("Press START when Ready");
        //telemetry.addLine("Press START when ready");
        dashboard.sendTelemetryPacket(packet);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -bot.MAX_SPEED, bot.MAX_SPEED) ;
        rightPower   = Range.clip(drive - turn, -bot.MAX_SPEED, bot.MAX_SPEED) ;

        // Send calculated power to wheels
        bot.lMotor.setPower(leftPower);
        bot.rMotor.setPower(rightPower);

        if (gamepad1.a){
            bot.lClaw.setPosition(Range.clip(bot.CLAW_CENTER-bot.CLAW_GAP/2,0,1));
            bot.rClaw.setPosition(Range.clip(bot.CLAW_CENTER+bot.CLAW_GAP/2,0,1));
        }
        else{
            bot.lClaw.setPosition(Range.clip(bot.CLAW_CENTER,0,1));
            bot.rClaw.setPosition(Range.clip(bot.CLAW_CENTER,0,1));
        }

        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        packet.put("Status", "Run Time: "+runtime.toString());
        packet.put("lMotor (%.2f)", leftPower);
        packet.put("rMotor (%.2f)", rightPower);
        dashboard.sendTelemetryPacket(packet); //sends lines of telemetry
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
