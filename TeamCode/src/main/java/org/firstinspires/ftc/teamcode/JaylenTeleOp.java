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
@TeleOp(name="bannana", group="Iterative OpMode")

public class JaylenTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    // = null;
   private DcMotor armTop = null;
   private DcMotor armBottom = null;
   private CRServo jaw = null;
   private DcMotor frontLeft = null;
   private DcMotor frontRight = null;
   private DcMotor backLeft = null;
   private DcMotor backRight = null;
   private Pcontroller armTopPController = new Pcontroller(0.005);
   private Pcontroller armBottomPController = new Pcontroller(0.005);


//   private CRServo intake = null;

    //add a dc motor named arm, a servo named jaw, and a CRServo named intake
    /*
     * Code to run ONCE when the driver hits INIT
     */
    enum ServoStates
    {
        SERVO_ZERO,
        SERVO_FULL

    }

    ServoStates servoStates = ServoStates.SERVO_ZERO;



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //motor  = hardwareMap.get(DcMotor.class, "motor");

        //add a hw map for all three members

        armTop = hardwareMap.get(DcMotor.class, "armTop");
        armBottom = hardwareMap.get(DcMotor.class, "armBottom");
        jaw = hardwareMap.get(CRServo.class, "jaw");
        frontLeft = hardwareMap.get(DcMotor.class,"front left motor");
        frontRight = hardwareMap.get(DcMotor.class,"front right motor");
        backLeft = hardwareMap.get(DcMotor.class,"back left motor");
        backRight = hardwareMap.get(DcMotor.class,"back right motor");


        armTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        jaw.setDirection(DcMotorSimple.Direction.REVERSE);
        //made it easier to code, reversing some wheel's directions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //stop arm when no power
        armTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //made it so, when no power, wheels stop
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        intake = hardwareMap.get(CRServo.class, "intake");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        // Setup a variable for each drive wheel to save power level for telemetry

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.

        if(gamepad1.dpad_up)
        {
            armTop.setPower(1);
            armBottom.setPower(1);
            armTopPController.setSetPoint(armTop.getCurrentPosition());
            armBottomPController.setSetPoint(armTop.getCurrentPosition());
        }

        else if(gamepad1.dpad_down)
        {
            armTop.setPower(-1);
            armBottom.setPower(-1);
            armTopPController.setSetPoint(armTop.getCurrentPosition());
            armBottomPController.setSetPoint(armTop.getCurrentPosition());
        }

        else
        {
            updateArmPosition();
        }

        switch (servoStates)
        {


            case SERVO_ZERO:
            {

                if(gamepad1.dpad_right)
                {
                    jaw.setPower(0);
                    timer.reset();
                    servoStates = ServoStates.SERVO_FULL;
                }
                break;
            }

            case SERVO_FULL:
            {
                    jaw.setPower(1);
                if(timer.milliseconds() > 1000)
                {
                    jaw.setPower(0);
                    timer.reset();
                    servoStates = ServoStates.SERVO_ZERO;
                }

            break;
            }

        }


//      '  if(gamepad1.dpad_up)
//        {
//            jaw.setPosition(0.4);
//            //set jaw to open
//        }
//        else if(gamepad1.dpad_down)
//        {
//            jaw.setPosition(0);
//           //close jaw'
//        }

//        if(gamepad1.left_bumper)
//        {
//            intake.setPower(1);
//        }
//
//        if(gamepad1.right_bumper)
//        {
//            intake.setPower(0);
//        }
//
        //add an if statement here to turn on and off the intake after button presses

        // Show the elapsed game time and wheel power.

    telemetry.addData("run time: ",runtime.toString());
//        telemetry.addData("intake power", intake.getPower());
        telemetry.addData("top power", armTop.getPower());
        telemetry.addData("bottom power",armBottom.getPower());
        telemetry.addData("jaw position", jaw.getPower());
        telemetry.addData("state: ", servoStates);
        telemetry.addData("timer: ", timer.milliseconds());
        telemetry.addData("front left power", frontLeft.getPower());
        telemetry.addData("front right power", frontRight.getPower());
        telemetry.addData("back left power", backLeft.getPower());
        telemetry.addData("back right power", backRight.getPower());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    public void updateArmPosition ()
    {
        int topArmMotorPosition = armTop.getCurrentPosition();
        int bottomArmMotorPosition = armBottom.getCurrentPosition();

        if (topArmMotorPosition < armTopPController.setPoint)
        {
            armTop.setPower(.01 + armTopPController.getComputedOutput(topArmMotorPosition));
        }
        else
        {
            armTop.setPower(.01 - armTopPController.getComputedOutput(topArmMotorPosition));
        }

        if (bottomArmMotorPosition < armBottomPController.setPoint)
        {
            armBottom.setPower(.01 + armBottomPController.getComputedOutput(bottomArmMotorPosition));
        }
        else
        {

            armBottom.setPower(.01 - armBottomPController.getComputedOutput(bottomArmMotorPosition));
        }


    }
}
/*print("Hello world"
   * DcMotorSimple
   *   tra
   */
