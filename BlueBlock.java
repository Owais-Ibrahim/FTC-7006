package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name= "Blue Block", group="Sky autonomous")
@Disabled
public class BlueBlock extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();


    //Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.858;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     TourqueMotorTicks    = 1440 ;
    static final double     PulleyDiameter   = 1.25;
    static final double     LiftInch         = (TourqueMotorTicks) / (PulleyDiameter * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        GyroCalibirate();
        EncoderReset();

        waitForStart();


        //GetBlock3();
    }


    //1st Block Code (Based on vision)
    public void GetBlock1() throws InterruptedException
    {
        EncodedStraightDrive(24,0.6);
        sleep(100);
        EncodedStrafe(11,0.6);//Negative for right and positive for left
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
        EncodedStraightDrive(75,0.5);
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
        EncodedStraightDrive(85,0.5);
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
        EncodedStraightDrive(92,0.5);
        sleep(100);
        TurnAbsolute(195,0.3,2);
        EncodedStraightDrive(-22,0.3);
        Foundation(); //Moves Foundation and drops block
        Park();
    }


    public void Park() throws InterruptedException
    {
        EncodedStraightDrive(30,0.5);
        EncodedStrafe(5,0.5);
        EncodedStraightDrive(10,0.5);
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
        EncodedStrafe(18,0.3);

        robot.GrabFoundationRight.setPosition(0.5);
        robot.GrabFoundationLeft.setPosition(1.0);
        sleep(10);

        TurnAbsolute(90,0.3,2);

        robot.GrabFoundationRight.setPosition(0.25);
        robot.GrabFoundationLeft.setPosition(0.5);
        sleep(1000);

        EncodedStraightDriveFoundation2(-15,0.5);//Brings block out and throws

        robot.GrabMove.setPosition(0);
        sleep(600);

        LiftDown(7,1.0);

    }



}
