package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name= "Red Block Vision", group="Sky autonomous")
//@Disabled
public class RedBlocksVision extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    String Skystone="";

    OpenCvCamera phoneCam;
    //Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.858;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     TourqueMotorTicks    = 1440 ;
    static final double     PulleyDiameter   = 1.25;
    static final double     LiftInch         = (TourqueMotorTicks) / (PulleyDiameter * 3.1415);

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        Detect();
        GyroCalibirate();
        EncoderReset();

        telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
        telemetry.update();

        waitForStart();

        phoneCam.closeCameraDevice();

        if (valRight==0)
        {
            Skystone="Right";
            telemetry.addData("Skystone Position:",Skystone);
            telemetry.update();

            GetBlock1();
        }

        if(valMid==0)
        {
            Skystone="Center";
            telemetry.addData("Skystone Position:",Skystone);
            telemetry.update();

            GetBlock2();
        }
        if(valLeft==0)
        {
            Skystone="Left";
            telemetry.addData("Skystone Position:",Skystone);
            telemetry.update();

            GetBlock3();
        }

    }

    //1st Block Code (Based on vision)
    public void GetBlock1() throws InterruptedException
    {
        EncodedStraightDrive(24,0.6);
        sleep(100);
        EncodedStrafe(-11,0.6);//Negative for right and positive for left
        GrabBlock(32,0.5);
        robot.Push.setPower(1.0);
        sleep(600);
        EncodedStraightDrive(-32,0.5);
        robot.Push.setPower(1.0);
        sleep(100);
        robot.Grabber.setPosition(1.0);
        robot.Push.setPower(0);
        sleep(50);
        TurnAbsolute(286,0.3,2);
        sleep(100);
        EncodedStraightDrive(-75,0.5);
        sleep(100);
        TurnAbsolute(195,0.3,2);
        EncodedStraightDrive(-22,0.3);
        Foundation(); //Moves Foundation and drops block
        Park();
    }

    public void GetBlock2()throws InterruptedException
    {
        EncodedStraightDrive(24,0.6);
        sleep(100);
        GrabBlock(32,0.5);
        robot.Push.setPower(1.0);
        sleep(600);
        EncodedStraightDrive(-32,0.5);
        robot.Push.setPower(1.0);
        sleep(100);
        robot.Grabber.setPosition(1.0);
        robot.Push.setPower(0);
        sleep(50);
        TurnAbsolute(286,0.3,2);
        sleep(100);
        EncodedStraightDrive(-85,0.5);
        sleep(100);
        TurnAbsolute(195,0.3,2);
        EncodedStraightDrive(-22,0.3);
        Foundation(); //Moves Foundation and drops block
        Park();
    }

    public void GetBlock3()throws InterruptedException
    {
        EncodedStraightDrive(24,0.6);
        sleep(100);
        EncodedStrafe(8,0.6);
        sleep(100);
        GrabBlock(32,0.5);
        robot.Push.setPower(1.0);
        sleep(600);
        EncodedStraightDrive(-32,0.5);
        robot.Push.setPower(1.0);
        sleep(100);
        robot.Grabber.setPosition(1.0);
        robot.Push.setPower(0);
        sleep(50);
        TurnAbsolute(286,0.3,2);
        sleep(100);
        EncodedStraightDrive(-92,0.5);
        sleep(100);
        TurnAbsolute(195,0.3,2);
        EncodedStraightDrive(-22,0.3);
        Foundation(); //Moves Foundation and drops block
        Park();
    }


    public void Park() throws InterruptedException
    {
        EncodedStraightDrive(5,1.0);
        EncodedStrafe(8,0.5);
        EncodedStrafe(-20,0.5);
        EncodedStraightDrive(40,1.0);

    }

    //Encoder

    public void LiftUp (double inches, double speed) throws InterruptedException
    {
        int RotationsNeeded= (int)(inches*(LiftInch));

        robot.Lift.setTargetPosition((-RotationsNeeded)+(robot.Lift.getCurrentPosition()));
        // Turn On RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Lift.setPower(speed);
        while (robot.Lift.isBusy())
        {
            // Display it for the driver.
            telemetry.addData("Lift Up:", robot.Lift.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();

        // Stop all motion;
        robot.Lift.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void LiftDown (double inches, double speed) throws InterruptedException
    {
        int RotationsNeeded= (int)(inches*(LiftInch));

        robot.Lift.setTargetPosition((RotationsNeeded)+(robot.Lift.getCurrentPosition()));
        // Turn On RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Lift.setPower(speed);
        while (robot.Lift.isBusy())
        {

            // Display it for the driver.
            telemetry.addData("Lift Up:", robot.Lift.getCurrentPosition());
            telemetry.update();
        }

        waitOneFullHardwareCycle();

        // Stop all motion;
        robot.Lift.setPower(0);
        robot.Push.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void GrabBlock(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.rightBack.setTargetPosition((-RotationsNeeded)+(robot.rightBack.getCurrentPosition()));
        robot.leftFront.setTargetPosition((RotationsNeeded)+(robot.leftFront.getCurrentPosition()));
        robot.leftBack.setTargetPosition((RotationsNeeded)+(robot.leftBack.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.addData("RightBack:", RotationsNeeded);
        telemetry.addData("LeftFront:", RotationsNeeded);
        telemetry.addData("LeftBack:", RotationsNeeded);

        telemetry.update();



        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sleep(1000);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        //sleep(100);

        while (opModeIsActive() && (robot.rightFront.isBusy() && robot.leftFront.isBusy())
                && (robot.rightBack.isBusy() && robot.leftBack.isBusy()))
        {
            robot.rightIntake.setPower(-1.0);
            robot.leftIntake.setPower(1.0);

            // Display it for the driver.
            telemetry.addData("Running:", robot.rightFront.getCurrentPosition());
            telemetry.addData("Left:", robot.leftFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        robot.rightIntake.setPower(0);
        robot.leftIntake.setPower(0);
        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void EncodedStraightDrive(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.rightBack.setTargetPosition((-RotationsNeeded)+(robot.rightBack.getCurrentPosition()));
        robot.leftFront.setTargetPosition((RotationsNeeded)+(robot.leftFront.getCurrentPosition()));
        robot.leftBack.setTargetPosition((RotationsNeeded)+(robot.leftBack.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.addData("RightBack:", RotationsNeeded);
        telemetry.addData("LeftFront:", RotationsNeeded);
        telemetry.addData("LeftBack:", RotationsNeeded);

        telemetry.update();



        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sleep(1000);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        //sleep(100);

        while (opModeIsActive() && (robot.rightFront.isBusy() && robot.leftFront.isBusy())
                && (robot.rightBack.isBusy() && robot.leftBack.isBusy()))
        {

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Negative for right and positive for left
    public void EncodedStrafe(double inches, double speed) throws InterruptedException //Make negative if want to go right but positive if want to go left
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));


        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.rightBack.setTargetPosition((RotationsNeeded)+(robot.rightBack.getCurrentPosition()));
        robot.leftFront.setTargetPosition((-RotationsNeeded)+(robot.leftFront.getCurrentPosition()));
        robot.leftBack.setTargetPosition((RotationsNeeded)+(robot.leftBack.getCurrentPosition()));

        // Turn On RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);

        while (opModeIsActive() && (robot.rightFront.isBusy() && robot.leftFront.isBusy())
                && (robot.rightBack.isBusy() && robot.leftBack.isBusy()))
        {



            // Display it for the driver.
            telemetry.addData("Right:", robot.rightFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void EncoderReset()throws InterruptedException
    {
        telemetry.addLine("Status Resetting Encoders");
        telemetry.update();


        //robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Right:", "at %2d", robot.rightFront.getCurrentPosition());
        telemetry.update();
        telemetry.addData("Left:", "at %2d", robot.leftFront.getCurrentPosition());
        telemetry.update();


    }


    //Gyro
    public void GyroCalibirate() throws InterruptedException
    {
        robot.gyro.calibrate();

        while (robot.gyro.isCalibrating())
        {
            telemetry.addLine("Calibrating");
            telemetry.update();
        }

        telemetry.addLine("Robot Done Calibrating");
        telemetry.update();
    }

    public void TurnAbsolute(int target,double turnspeed, int threshold) throws InterruptedException
    {
        int Heading= robot.gyro.getHeading();
        if (Heading>180)
        {
            Heading= robot.gyro.getHeading()-360;
        }

        while (Math.abs(Heading-target) > threshold)
        {
            if (Heading>180)
            {
                Heading= robot.gyro.getHeading()-360;
            }

            telemetry.addData("CurrentHeading:", Heading);
            telemetry.update();

            if (Heading>target)
            {
                //CW
                robot.rightFront.setPower(turnspeed);
                robot.rightBack.setPower(turnspeed);
                robot.leftFront.setPower(turnspeed);
                robot.leftBack.setPower(turnspeed);

                telemetry.addData("Heading>Target",Heading );
                telemetry.update();

            }
            if (Heading<target)
            {
                //CCW
                robot.rightFront.setPower(-turnspeed);
                robot.rightBack.setPower(-turnspeed);
                robot.leftFront.setPower(-turnspeed);
                robot.leftBack.setPower(-turnspeed);

                telemetry.addData("Heading<Target", Heading );
                telemetry.update();
            }


            Heading=robot.gyro.getHeading();
        }



        waitOneFullHardwareCycle();

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        telemetry.addData("CurrentHeading:", robot.gyro.getHeading());
        telemetry.update();
    }
    public void TurnAbsolutePositive(int target,double turnspeed, int threshold) throws InterruptedException
    {
        int Heading= robot.gyro.getHeading();
        if (Heading>180)
        {
            Heading= robot.gyro.getHeading()-360;
        }

        while (Math.abs(Heading-target) > threshold)
        {
            if (Heading>180)
            {
                Heading= robot.gyro.getHeading()-360;
            }

            telemetry.addData("CurrentHeading:", Heading);
            telemetry.update();

            if (Heading>target)
            {
                //CW
                robot.rightFront.setPower(turnspeed);
                robot.rightBack.setPower(turnspeed);
                robot.leftFront.setPower(turnspeed);
                robot.leftBack.setPower(turnspeed);






                telemetry.addData("Heading>Target",Heading );
                telemetry.update();

            }
            if (Heading<target)
            {
                //CCW
                robot.rightFront.setPower(turnspeed);
                robot.rightBack.setPower(turnspeed);
                robot.leftFront.setPower(turnspeed);
                robot.leftBack.setPower(turnspeed);



                telemetry.addData("Heading<Target", Heading );
                telemetry.update();
            }


            Heading=robot.gyro.getHeading();
        }



        waitOneFullHardwareCycle();

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        telemetry.addData("CurrentHeading:", robot.gyro.getHeading());
        telemetry.update();
    }
    public void EncodedStraightDriveFoundation(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.rightBack.setTargetPosition((-RotationsNeeded)+(robot.rightBack.getCurrentPosition()));
        robot.leftFront.setTargetPosition((RotationsNeeded)+(robot.leftFront.getCurrentPosition()));
        robot.leftBack.setTargetPosition((RotationsNeeded)+(robot.leftBack.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.addData("RightBack:", RotationsNeeded);
        telemetry.addData("LeftFront:", RotationsNeeded);
        telemetry.addData("LeftBack:", RotationsNeeded);

        telemetry.update();



        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sleep(1000);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        //sleep(100);

        while (opModeIsActive() && (robot.rightFront.isBusy() && robot.leftFront.isBusy())
                && (robot.rightBack.isBusy() && robot.leftBack.isBusy()))
        {
            robot.Lift.setPower(-0.8);
            sleep(500);
            robot.Lift.setPower(0);
            sleep(100);

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.Lift.setPower(0);


        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void EncodedStraightDriveFoundation2(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.rightBack.setTargetPosition((-RotationsNeeded)+(robot.rightBack.getCurrentPosition()));
        robot.leftFront.setTargetPosition((RotationsNeeded)+(robot.leftFront.getCurrentPosition()));
        robot.leftBack.setTargetPosition((RotationsNeeded)+(robot.leftBack.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.addData("RightBack:", RotationsNeeded);
        telemetry.addData("LeftFront:", RotationsNeeded);
        telemetry.addData("LeftBack:", RotationsNeeded);

        telemetry.update();



        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sleep(1000);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        //sleep(100);

        while (opModeIsActive() && (robot.rightFront.isBusy() && robot.leftFront.isBusy())
                && (robot.rightBack.isBusy() && robot.leftBack.isBusy()))
        {

            robot.GrabMove.setPosition(1.0);
        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.Lift.setPower(0);
        robot.Grabber.setPosition(0.5);
        sleep(200);


        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Foundation

    public void Foundation() throws InterruptedException
    {
        robot.GrabFoundationRight.setPosition(0.5);
        robot.GrabFoundationLeft.setPosition(1.0);
        sleep(100);

        EncodedStraightDriveFoundation(30,0.4);//Brings Lift Up as well
        sleep(100);
        EncodedStrafe(-18,0.3);

        robot.GrabFoundationRight.setPosition(0.5);
        robot.GrabFoundationLeft.setPosition(1.0);
        sleep(10);

        TurnAbsolutePositive(270,0.3,2);

        robot.GrabFoundationRight.setPosition(0.25);
        robot.GrabFoundationLeft.setPosition(0.5);
        sleep(1000);

        EncodedStraightDriveFoundation2(-15,0.5);//Brings block out and throws

        robot.GrabMove.setPosition(0);
        sleep(600);

        LiftDown(7,1.0);

    }

    //Detection Stuff
    public void Detect()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
    }

    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }

}
