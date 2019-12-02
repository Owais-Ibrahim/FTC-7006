package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp5", group="Pushbot")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot(); // Use a Pushbot's hardware


    @Override
    public void runOpMode() throws InterruptedException

    {
        double drive ;
        
        
        robot.init(hardwareMap);
        

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver"); //
        telemetry.update();
        
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive = -gamepad1.right_stick_y;
            
                robot.rightFront.setPower(drive);
                robot.rightBack.setPower(drive);
                robot.leftFront.setPower(-drive);
                robot.leftBack.setPower(-drive);
            
            }

         


        }
    }

