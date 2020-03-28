package org.firstinspires.ftc.teamcode.TESTING;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwarePushbot;

import java.lang.Math;


//PERSONAL ALGORITHIM
@Autonomous(name="MR1", group="Pushbot")
@Disabled
public class ModernRoboticsTest1 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.858;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);



    @Override
    public void runOpMode() throws InterruptedException{


        robot.init(hardwareMap);



        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        EncoderReset();


        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
            telemetry.update();
        }

        robot.gyro.resetZAxisIntegrator();


        waitForStart();


        telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
        telemetry.update();

        EncodedForwardDrive(20,0.4);
        sleep(50);
        GyroTurn(0.3,0,1);
        sleep(50);
        EncodedForwardDrive(20,0.4);
        sleep(50);
        GyroTurn(0.3,0,1);
        sleep(50);
        EncodedBackwardDrive(20,0.4);
        sleep(50);
        AbsoluteTurn(0.4,83,90);

        EncodedForwardDrive(60,0.4);
        //sleep(50);






        telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
        telemetry.update();
        sleep(1000);



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void AbsoluteTurn(double speed, double angle,double angle2) throws InterruptedException
    {
        int gyro=robot.gyro.getIntegratedZValue();

        if (angle>0)
        {
            while(gyro<angle)
            {
                robot.leftFront.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.leftBack.setPower(speed);
                robot.rightBack.setPower(speed);
                gyro=robot.gyro.getIntegratedZValue();
            }

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }
        if (angle<0)
        {
            while(gyro>angle)
            {
                robot.leftFront.setPower(-speed);
                robot.rightFront.setPower(-speed);
                robot.leftBack.setPower(-speed);
                robot.rightBack.setPower(-speed);
                gyro=robot.gyro.getIntegratedZValue();
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }

        GyroTurn(0.2,angle2,1);


        waitOneFullHardwareCycle();

        gyro=robot.gyro.getIntegratedZValue();
        telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
        telemetry.update();


    }


    public void GyroTurn(double speed, double angle,int threshold) throws InterruptedException
    {
        int gyro=robot.gyro.getIntegratedZValue();
        while(Math.abs(gyro-angle)>threshold)
        {
            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
            telemetry.update();

            if (gyro<angle)
            {
                robot.leftFront.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.leftBack.setPower(speed);
                robot.rightBack.setPower(speed);
                gyro=robot.gyro.getIntegratedZValue();

                telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
                telemetry.update();


                //angle=angle+threshold;

            }
            if (gyro>angle)
            {
                robot.leftFront.setPower(-speed);
                robot.rightFront.setPower(-speed);
                robot.leftBack.setPower(-speed);
                robot.rightBack.setPower(-speed);
                gyro=robot.gyro.getIntegratedZValue();

                telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
                telemetry.update();

                //angle=angle+threshold;
            }
            waitOneFullHardwareCycle();
            gyro=robot.gyro.getIntegratedZValue();
            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
            telemetry.update();

        }
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }


    public void EncodedBackwardDrive(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.leftFront.setTargetPosition((-RotationsNeeded)+(robot.leftFront.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.addData("LeftFront:", RotationsNeeded);

        telemetry.update();



        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightFront.setPower(speed);
        robot.leftFront.setPower(speed);

        while (opModeIsActive()&& (robot.rightFront.isBusy()))

        {
            robot.rightBack.setPower(speed);
            robot.leftBack.setPower(-speed);

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void EncodedForwardDrive(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.leftFront.setTargetPosition((RotationsNeeded)+(robot.leftFront.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.addData("LeftFront:", RotationsNeeded);

        telemetry.update();



        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sleep(1000);
        robot.rightFront.setPower(speed);
        robot.leftFront.setPower(speed);
        //sleep(100);

        while (opModeIsActive()&& (robot.rightFront.isBusy()))

        {
            robot.rightBack.setPower(-speed);
            robot.leftBack.setPower(speed);

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void EncoderReset()throws InterruptedException
    {
        telemetry.addLine("Status Resetting Encoders");
        telemetry.update();


        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //robot.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Right:", "at %2d", robot.rightFront.getCurrentPosition());
        telemetry.addData("Left:", "at %2d", robot.leftFront.getCurrentPosition());
        telemetry.update();


    }

}
