package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpOwais", group="Pushbot")
public class Teleop1 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot(); // Use a Pushbot's hardware

    //Variables
    double drive;
    double strafe;
    double spin;
    double intake;
    double lift;
    double push;

    @Override
    public void runOpMode() throws InterruptedException

    {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver"); //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /** DRIVE CONTROLS
             * GAMEPAD 1
             * Drive with right stick and spin with left trigger and left stick x
             * Intake with right trigger and left stick
             * Control auto arm with button x
             * GAMEPAD 2
             * Lift up with right stick y and lift rotate with right stick x
             * Move grabber with left stick x
             * Grab with button a
             */

            //Define Variables

            //Gamepad 1
            drive= gamepad1.right_stick_y;
            strafe= -gamepad1.right_stick_x;
            spin=gamepad1.left_stick_x;
            intake=gamepad2.right_stick_y;
            lift=gamepad2.left_stick_y;
            push=gamepad1.left_stick_y;









            //TELEOP

            robot.rightFront.setPower(drive-strafe);
            robot.rightBack.setPower(drive+strafe);
            robot.leftFront.setPower(-drive-strafe);
            robot.leftBack.setPower(-drive+strafe);

            if (spin>0.5|| spin<-0.5)
            {
                robot.rightFront.setPower(spin);
                robot.rightBack.setPower(spin);
                robot.leftFront.setPower(spin);
                robot.leftBack.setPower(spin);
            }



            robot.rightIntake.setPower(intake);
            robot.leftIntake.setPower(-intake);




            robot.Lift.setPower(lift);


            if (gamepad2.a)
            {
                robot.Grabber.setPosition(1.0);

            }

            if (gamepad2.b)
            {
                robot.Grabber.setPosition(0.5);
            }

            if (gamepad2.x)
            {
                robot.GrabMove.setPosition(1.0);
            }
            if (gamepad2.y)
            {
                robot.GrabMove.setPosition(0);
            }

            if (push>0.75 || push<-0.75)
            {
                robot.Push.setPower(push);
            }










        }
    }
}
