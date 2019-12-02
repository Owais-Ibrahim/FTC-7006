package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class HardwarePushbot
{
    //Declare Variables
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor RWheel = null;
    public DcMotor LWheel=null;
    public DcMotor intarm=null;
    public Servo arm=null;


    /* local OpMode members. */
    HardwareMap hwMap = null;

    //Constructing Hardware Map
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init (HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Variables
        rightFront = hwMap.get(DcMotor.class, "fr");
        rightBack = hwMap.get(DcMotor.class, "br");
        leftFront = hwMap.get(DcMotor.class, "fl");
        leftBack = hwMap.get(DcMotor.class,"bl");
        LWheel=hwMap.get(DcMotor.class,"lw");
        RWheel=hwMap.get(DcMotor.class,"rw");
        intarm=hwMap.get(DcMotor.class, "int");
        arm=hwMap.get(Servo.class,"arm");


        // Set all motors to zero power

        //Drive System
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);

        //Intake
        LWheel.setPower(0);
        RWheel.setPower(0);
        intarm.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        //Drive System
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Intake
        LWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Set Motors to brake

        //Drive system
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setPosition(0.0);
    }
}