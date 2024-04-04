package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;


//@I2cDeviceType
//@DeviceProperties(name = "Laser Range Sensor (0x08)", xmlTag = "RS21221")
public class LaserRangeSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    protected synchronized boolean doInitialize(){
        return true;
    }
    public String getDeviceName(){
        return "Waveshare Laser Range Sensor 21221";
    }

    /*public LaserRangeSensor(I2cDeviceSynch deviceClient){
        super(deviceClient,true);
        this.addr = I2cAddr.create7bit(0x08);
        this.deviceClient.setI2cAddress(this.addr);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage(); // communication over i2c
    }
    */

    public LaserRangeSensor(I2cDeviceSynch deviceClient,int addr){
        super(deviceClient,true);

        this.deviceClient.setI2cAddress(new I2cAddr(addr));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage(); // communication over i2c
    }
    protected int readShort(int addr){

        byte[] arr = deviceClient.read(addr,4);
        byte temp = 0;
        temp = arr[0];
        arr[0] = arr[3];
        arr[3] = temp;
        temp = arr[1];
        arr[1] = arr[2];
        arr[2] = temp;
        return TypeConversion.byteArrayToInt(arr);
    }

    public int getDistance(){
        return readShort(0x24);
    }

}
