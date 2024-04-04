package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Optical Flow Test")
public class OpticalFlowTest extends LinearOpMode {
    private OpticalFlowSensor opticalFlow;

    public void runOpMode() throws InterruptedException {

        opticalFlow = hardwareMap.get(OpticalFlowSensor.class, "opt_flow");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Product ID: ", opticalFlow.getProductID());
            telemetry.update();
        }
    }
}
