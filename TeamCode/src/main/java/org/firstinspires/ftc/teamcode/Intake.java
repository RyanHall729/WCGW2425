package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor rotator = null;
    public Servo extender = null;
    public CRServo intake = null;

    public Intake(HardwareMap hardwareMap)
    {
        rotator = hardwareMap.get(DcMotor.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        extender = hardwareMap.get(Servo.class, "tilt");
    }


}
