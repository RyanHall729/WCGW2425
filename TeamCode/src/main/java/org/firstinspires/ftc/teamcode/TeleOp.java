/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 *  Note that a Mecanum drive must display an X roller-pattern when viewed from above.

 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Basic: Sink", group="Linear OpMode")
//@Disabled
public class TeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
//    private ElapsedTime runtime = new ElapsedTime();
//    private ElapsedTime intakeStopwatch = new ElapsedTime();
//    private ElapsedTime elbowStopwatch = new ElapsedTime();
//    public DcMotor leftFront = null;
//    public DcMotor leftBack = null;
//    public DcMotor rightFront = null;
//    public DcMotor rightBack = null;
//    public Servo extender = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public CRServo intake2 = null;
    public Servo extender = null;
    public Servo wrist = null;
    public DcMotorEx elbowTop = null;
    public DcMotorEx elbowBottom = null;
    public ElapsedTime intakeStopwatch = null;
    public boolean isOutaking = false;
    public boolean isIntaking = false;
    public boolean elbowFunctionUp = false;
    public boolean elbowFunctionDown = false;
    public IMU imu = null;
    public int armPosition = 0;
    public Pcontroller pTopControllerArm = new Pcontroller(.005);
    public Pcontroller pBottomControllerArm = new Pcontroller(.005);
    public int elbowTopMaxTicks = 1607;
    public int elbowBottomMaxTicks = 1559;
    public int state = 0;
    public boolean statePreValue = false;
    public double sensitivity = 5;
    public double elbowTopPosition;
    public double elbowFactor;
    public double ticsPerRotationTop;
    public double ticsPerRotationBottom;
    public double radPerTicTop;
    public double radPerTicBottom;
    public double extenderPosition;


   enum GrabAndDrop {
        GRAB_SAMPLE,
        DRIVE,
        DROP,
        HOME,
    }

   GrabAndDrop grabAndDrop = GrabAndDrop.GRAB_SAMPLE;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        intakeStopwatch = new ElapsedTime();
//        intake = hardwareMap.get(CRServo.class, "intake");
        elbowTop = hardwareMap.get(DcMotorEx.class, "elbowTop");
        elbowBottom = hardwareMap.get(DcMotorEx.class, "elbowBottom");
        intake2 = hardwareMap.get(CRServo.class, "intake2"); // expansion hub port 0
        intake2.setPower(0);
        extender = hardwareMap.get(Servo.class, "extender");//port 1
        //extender.setPosition(0.0); // use position 0 when installing the extender, otherwise use the position 6
        extender.setPosition(0.8);
        wrist = hardwareMap.get(Servo.class, "wrist");//expansion hub port 1
        wrist.setPosition(.475);
        elbowTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowTop.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elbowBottom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elbowTop.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elbowBottom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elbowTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elbowBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        IMU.Parameters parameters = new IMU.Parameters(hubOrientationOnRobot);
        imu.initialize(parameters);



//        extender = hardwareMap.get(Servo.class, "extender");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        elbowTop.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        ticsPerRotationTop= elbowTop.getMotorType().getTicksPerRev();
        ticsPerRotationBottom = elbowBottom.getMotorType().getTicksPerRev();
        radPerTicTop = Math.PI/(ticsPerRotationTop);
        radPerTicBottom = Math.PI/(ticsPerRotationBottom);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("radPerTicTop", radPerTicTop);
        telemetry.addData("radPerTicBottom",radPerTicBottom);
        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.addData("Extender Position", extender.getPosition());

        //intake = hardwareMap.get(CRServo.class, "intake");
        pTopControllerArm.setInputRange(0, elbowTopMaxTicks);;//////////////////////
        pTopControllerArm.setOutputRange(.01,0.60);
        pTopControllerArm.setThresholdValue(2);

        pBottomControllerArm.setInputRange(0, elbowBottomMaxTicks);;////////////////
        pBottomControllerArm.setOutputRange(.01,0.60);
        pBottomControllerArm.setThresholdValue(2);
        telemetry.update();



        waitForStart();
//        runtime.reset();

        // run until the end of the match (driver presses STOP)
        double leftBackPower = 0;
        double leftFrontPower = 0;
        double rightBackPower = 0;
        double rightFrontPower = 0;


        while (opModeIsActive()) {


            double max;
            double sensitivity = 5.0;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
         //       double axial = Math.abs(axial1)/axial1 * (Math.pow(sensitivity, Math.abs(axial1)) - 1) / (sensitivity - 1);
            double lateral = gamepad1.left_stick_x;
         //       double lateral = Math.abs(lateral1)/lateral1 * (Math.pow(sensitivity, Math.abs(lateral1)) - 1) / (sensitivity - 1);
            double yaw = gamepad1.right_stick_x;
         //       double yaw = Math.abs(yaw1)/yaw1*(Math.pow(sensitivity, Math.abs(yaw1)) - 1)/(sensitivity - 1);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
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
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            //extender
            // code change is to increment extender position when moving in and out
            // for  each increment


                // extender
            extenderPosition = extender.getPosition();
            if (gamepad1.left_bumper && extenderPosition >= 0.1 && extenderPosition <= .9 && gamepad1.right_bumper == false)
            {
                    extenderPosition += 0.001; ///adjust last term to change speed
                    extender.setPosition(extenderPosition);
            }
            if (gamepad1.right_bumper && extenderPosition >= 0.2 && extenderPosition <= .9 && gamepad1.left_bumper == false) ;
            {
                extenderPosition -= 0.001; ///adjust last term to change speed
                extender.setPosition(extenderPosition);
            }
            //change elbow position as arm extends and contracts when elbow is in a sample pickup position
            //elbowTopPosition = elbowTop.getCurrentPosition();
//                if (elbowTopPosition > 1400.0){
//                    // calculate present extended length of arm (the value 5 is the servo position when extender fully retracted) (4 is the servo position when elbow down and wrist touches mat)
//                    double extenderPosition = (4.5 - extender.getPosition()) * 5.0/4.5; // 0 is the servo position  at pinion wheel touch with the end of the rack (initial setup), one rotation per inch
//                    if (extenderPosition < 1.125) {
//                        // calculate theta
//                        double theta = Math.asin(15.125 / 18.0);
//                        // calculate phi
//                        double phi = Math.asin(15.125 / (15.25 + extenderPosition));
//                        //calculate the required tics for new Betta
//                        double TicTop = elbowTopMaxTicks - (theta - phi) / radPerTicTop;
//                        if (TicTop < elbowTopMaxTicks) {
//                        } //turn on motors to adjust arm to TicTop
//                        //need to add power to both motors to move to present position to present position plus corrected amount (if extender position is negative then just move extender
//                    }
//                }
//            }

            //wrist
            if (gamepad1.left_trigger > 0.0 && gamepad1.right_trigger == 0.0)
            {
               wrist.setPosition(.375);
            }
            else if (gamepad1.left_trigger == 0.0 && gamepad1.right_trigger == 0.0)
            {
                wrist.setPosition(.475);
            }
            if (gamepad1.right_trigger > 0.0 && gamepad1.left_trigger == 0.0)
            {
                wrist.setPosition(.575);
            }

            //intake-in
            if (gamepad1.a)
            {
                intake2.setPower(1);
            }
            else if (gamepad1.b)
            {
                intake2.setPower(0);
            }


            //intake-out?
            if (gamepad1.x)
            {
                intake2.setPower(-1);
                isIntaking = true;
                intakeStopwatch.reset();
            }
            else if (isIntaking)
            {
                intake2.setPower(0);
            }

            //outake?
            if (gamepad1.b)
            {
                intake2.setPower(1);
                //intakeStopwatch.reset();
                isOutaking = true;
            } else if (isOutaking)
            {
                intake2.setPower(0);
                isOutaking = false;
            }

            //elbowup
            //double elbowTopPosition = elbowTop.getCurrentPosition();
            //double elbowFactor = 0.3 * Math.sin(Math.PI*(elbowTopPosition/elbowTopMaxTicks/2)) + 0.2;
            double elbowFactor = 0.3;
             if (gamepad1.dpad_up && elbowTop.getCurrentPosition()<elbowTopMaxTicks)
            {
                //elbowTop.setPower(elbowFactor);
                //elbowBottom.setPower(elbowFactor);
                elbowTop.setVelocity(400);
                elbowBottom.setVelocity(400);
                pTopControllerArm.setSetPoint(elbowTop.getCurrentPosition());
                pBottomControllerArm.setSetPoint(elbowBottom.getCurrentPosition());
            }

            // elbowdown
            else if (gamepad1.dpad_down && elbowTop.getCurrentPosition()>50)
            {
                //elbowTop.setPower(-elbowFactor);
                //elbowBottom.setPower(-elbowFactor);
                elbowTop.setVelocity(-400);
                elbowBottom.setVelocity(-400);
                pTopControllerArm.setSetPoint(elbowTop.getCurrentPosition());
                pBottomControllerArm.setSetPoint(elbowBottom.getCurrentPosition());
            }

            else {
                 updateArmPosition();
             }


            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Is outake Active:", isOutaking);
            telemetry.addData("intake:", intake2.getPower());
            telemetry.addData("rotation", imu.getRobotYawPitchRollAngles());
            telemetry.addData("elbowTop Power", elbowTop.getPower());
            telemetry.addData("elbowBottom Power", elbowBottom.getPower());
            telemetry.addData("elbowTop Position", elbowTop.getCurrentPosition());
            telemetry.addData("elbowBottom Position", elbowBottom.getCurrentPosition());
            telemetry.addData("p Top controller set pt ", pTopControllerArm.setPoint);
            telemetry.addData("p Bottom controller set pt ", pBottomControllerArm.setPoint);
            telemetry.addData("state", state);
            telemetry.addData("Extender Position", extender.getPosition());
            telemetry.addData("Double Extender Position", extenderPosition);
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Left Bumper", gamepad1.left_bumper);


            //telemetry.addData("intake", intake.getPower());
            telemetry.update();
        }
    }

    public void updateArmPosition ()
    {
        int topArmMotorPosition = elbowTop.getCurrentPosition();
        int bottomArmMotorPosition = elbowBottom.getCurrentPosition();

        if (topArmMotorPosition < pTopControllerArm.setPoint)
        {
            elbowTop.setPower(.01 + pTopControllerArm.getComputedOutput(topArmMotorPosition));
        }
        else
        {
            elbowTop.setPower(.01 - pTopControllerArm.getComputedOutput(topArmMotorPosition));
        }

        if (bottomArmMotorPosition < pBottomControllerArm.setPoint)
        {
            elbowBottom.setPower(.01 + pBottomControllerArm.getComputedOutput(bottomArmMotorPosition));
        }
        else
        {

            elbowBottom.setPower(.01 - pBottomControllerArm.getComputedOutput(bottomArmMotorPosition));
        }


    }

    public int averageArmPosition () {
        int topArmMotorPosition = elbowTop.getCurrentPosition();
        int bottomArmMotorPosition = elbowBottom.getCurrentPosition();

        return (topArmMotorPosition+bottomArmMotorPosition)/2;

    }



}

