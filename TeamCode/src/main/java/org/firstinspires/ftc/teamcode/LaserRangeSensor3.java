package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;


@I2cDeviceType
@DeviceProperties(name = "Laser Range Sensor (0x09)", xmlTag = "RS21221")
public class LaserRangeSensor3 extends LaserRangeSensor {

    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    protected synchronized boolean doInitialize(){
        return true;
    }
    public String getDeviceName(){
        return "Waveshare Laser Range Sensor 21221";
    }

    public LaserRangeSensor3(I2cDeviceSynch deviceClient){
        super(deviceClient,0x09);
    }

}
