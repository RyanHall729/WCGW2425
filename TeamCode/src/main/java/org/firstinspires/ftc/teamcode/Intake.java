package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    public DcMotor tilter = null;
    public Servo wrist = null;
    public CRServo intake = null;
    public ElapsedTime intakeStopwatch = null;
    public ElapsedTime tilterStopwatch = null;
    public boolean isOutaking = false;
    public boolean isTiltingUp = false;
    public boolean isTiltingDown = false;

}
