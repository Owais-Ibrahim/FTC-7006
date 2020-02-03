package org.firstinspires.ftc.teamcode;
import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class HardwarePushbot
{
    //Declare Variables
    //Drive terrain
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;

    //intake
    public DcMotor rightIntake=null;
    public DcMotor leftIntake=null;

    //Outake
    public Servo GrabMove=null;
    public Servo Grabber=null;
    public CRServo Push=null;
    public DcMotor Lift=null;


    //Additional Servo
    public Servo GrabFoundationRight =null;
    public Servo GrabFoundationLeft =null;


    //Sensor
    public ModernRoboticsI2cGyro gyro = null;
    //public ModernRoboticsI2cRangeSensor rangeSensor=null;





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
        //Drive Terrain
        rightFront = hwMap.get(DcMotor.class, "fr");
        rightBack = hwMap.get(DcMotor.class, "br");
        leftFront = hwMap.get(DcMotor.class, "fl");
        leftBack = hwMap.get(DcMotor.class,"bl");

        //Intake

        rightIntake=hwMap.get(DcMotor.class,"ri");
        leftIntake=hwMap.get(DcMotor.class,"li");


        //Outake
        Lift=hwMap.get(DcMotor.class,"lift");
        Grabber=hwMap.get(Servo.class,"grabber");
        GrabMove=hwMap.get(Servo.class,"grabservo");
        Push=hwMap.get(CRServo.class,"push");



        //Aditional Servos
        GrabFoundationRight=hwMap.get(Servo.class,"gr");
        GrabFoundationLeft=hwMap.get(Servo.class,"gl");



        //Sensor
        gyro=hwMap.get(ModernRoboticsI2cGyro.class,"g");
        //rangeSensor=hwMap.get(ModernRoboticsI2cRangeSensor.class,"r");






        // Set all motors to zero power

        //Drive System
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);

        //Intake
        rightIntake.setPower(0);
        leftIntake.setPower(0);


        //Outake
        Lift.setPower(0);
        Push.setPower(0);



        //Run Wihtout Encoders

        //Drive System
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Intake
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Outake
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set Motors to brake

        //Drive system
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intake
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Outake
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo Position
        GrabMove.setPosition(0);
        Grabber.setPosition(0.5);
        GrabFoundationLeft.setPosition(0);
        GrabFoundationRight.setPosition(0);


    }
}