package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Pushbot")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot(); // Use a Pushbot's hardware

    double drive;
    double strafe;
    double spin;
    double intake;
    double intarm;

    @Override
    public void runOpMode() throws InterruptedException

    {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver"); //
        telemetry.update();
        robot.arm.setPosition(0.0);//Bring arm up

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive= -gamepad1.right_stick_y;
            strafe= gamepad1.right_stick_x;
            spin= gamepad1.left_stick_x;
            intake=-gamepad2.right_stick_y;
            intarm=-gamepad2.left_stick_y;

            robot.rightFront.setPower(drive-strafe-spin);
            robot.rightBack.setPower(drive+strafe-spin);
            robot.leftFront.setPower(-drive-strafe-spin);
            robot.leftBack.setPower(-drive+strafe-spin);

           

            //Bring arm down
            if (gamepad1.y)
            {
                robot.arm.setPosition(0.0);

                telemetry.addLine("The arm is up");
                telemetry.update();

            }

            //Bring arm up
            if (gamepad1.x)
            {
                robot.arm.setPosition(0.75);

                telemetry.addLine("The arm is down");
                telemetry.update();

            }
            
            if (gamepad1.a)
            {
                robot.arm.setPosition(0.5);

                telemetry.addLine("The arm is half way");
                telemetry.update();
            }


            robot.LWheel.setPower(intake);
            robot.RWheel.setPower(-intake);
            robot.intarm.setPower(intarm);
            

        }
    }
}
