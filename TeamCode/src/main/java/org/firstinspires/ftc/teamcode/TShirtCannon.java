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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
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
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name="Applew", group="Iterative OpMode")

public class TShirtCannon extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // = null;



//   private CRServo intake = null;
    private DcMotor topRight = null;
    private DcMotor topLeft = null;
    private DcMotor bottomLeft = null;
    private DcMotor bottomRight = null;
    private DcMotor cannonRotater = null;
    private DcMotor cannonTilter = null;
    //add a dc motor named arm, a servo named jaw, and a CRServo named intake
    /*
     * Code to run ONCE when the driver hits INIT
     */
    enum ServoStates
    {


    }



    @Override
    public void init() {
       topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
        bottomRight = hardwareMap.get(DcMotor.class, "bottomRight");
        cannonTilter = hardwareMap.get(DcMotor.class, "cannonTilter");
        cannonRotater = hardwareMap.get(DcMotor.class, "cannonRotater");
        //Adds all motors to hardwear map
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cannonRotater.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cannonTilter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //set all motors to brake when there's no power
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }



    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */

    @Override
    public void loop() {
        //6 motors - one for cannon and other to turn cannon
        if (gamepad1.dpad_right){
            bottomLeft.setPower(0.5);
            bottomRight.setPower(0.5);
        }

        if (gamepad1.dpad_left) {
            topLeft.setPower(0.5);
            topRight.setPower(0.5);
        }

        if (gamepad1.left_bumper){
            cannonRotater.setPower(1);
        }

        if (gamepad1.right_bumper){
            cannonRotater.setPower(-1);
        }

        if (gamepad1.dpad_up){
            cannonTilter.setPower(1);
        }

        if (gamepad1.dpad_down){
            cannonTilter.setPower(-1);
        }

        else {
            topRight.setPower(0);
            topLeft.setPower(0);
            bottomRight.setPower(0);
            bottomLeft.setPower(0);
            cannonRotater.setPower(0);
            cannonTilter.setPower(0);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("top left power", topLeft.getPower());
        telemetry.addData("top right power", topRight.getPower());
        telemetry.addData("bottom left power", bottomLeft.getPower());
        telemetry.addData("bottom right power", bottomRight.getPower());
        telemetry.addData("cannon rotate power", cannonRotater.getPower());
        telemetry.addData("cannon rotate power", cannonTilter.getPower());

    }
}
/*print("Hello world"
 * DcMotorSimple
 *   tra
 */
