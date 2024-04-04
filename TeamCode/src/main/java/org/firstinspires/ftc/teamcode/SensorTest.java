package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name= "Distance Sensor Test")
public class SensorTest extends LinearOpMode {
    private LaserRangeSensor rangeSensor;

    public void runOpMode() throws InterruptedException {

        rangeSensor = hardwareMap.get(LaserRangeSensor.class, "dist_sensor");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Distance", rangeSensor.getDistance());
            telemetry.update();
        }
    }
}
