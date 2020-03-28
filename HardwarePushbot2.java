package org.firstinspires.ftc.teamcode.TESTING;
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


public class HardwarePushbot2
{
    //Declare Variables

    public DcMotor right=null;
    public DcMotor left=null;
    public Servo arm=null;
    public ModernRoboticsI2cRangeSensor rangeSensor=null;


    /* local OpMode members. */
    HardwareMap hwMap = null;

    //Constructing Hardware Map
    public HardwarePushbot2(){

    }

    /* Initialize standard Hardware interfaces */
    public void init (HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Variables

        left = hwMap.get(DcMotor.class, "l");
        right=hwMap.get(DcMotor.class,"r");
        arm=hwMap.get(Servo.class,"arm");
        rangeSensor=hwMap.get(ModernRoboticsI2cRangeSensor.class,"range");

        // Set all motors to zero power

        left.setPower(0.0);
        right.setPower(0);
        arm.setPosition(0);

        //Drive System

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
}