package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.motors.TetrixMotor;
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
    public ModernRoboticsI2cGyro gyro=null;
    public ModernRoboticsI2cColorSensor colorSensor=null;


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
        gyro=hwMap.get(ModernRoboticsI2cGyro.class,"gyro");
        colorSensor=hwMap.get(ModernRoboticsI2cColorSensor.class,"color");

        // Set all motors to zero power

        left.setPower(0.0);
        right.setPower(0);

        //Drive System

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
}