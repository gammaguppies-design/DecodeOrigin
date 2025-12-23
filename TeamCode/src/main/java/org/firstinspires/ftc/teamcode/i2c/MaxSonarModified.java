package org.firstinspires.ftc.teamcode.i2c;

import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(xmlTag = "MaxSonarModified", name = "MaxSonar Modified")
public class MaxSonarModified extends MaxSonarI2CXL {


    //------------------------------------------------------------------------------------------
    // Constructor
    //------------------------------------------------------------------------------------------

    public MaxSonarModified(I2cDeviceSynch deviceClient)
    {
        super(deviceClient);
    }


    /**
     * If sonarPropagationDelayMs is <=0, 'reset' the pipeline of Async measurements by sending a
     * ping, updating lastPingTime, and returning -1.
     *
     * If sonarPropagationDelayMs >0 and at least sonarPropagationDelayMs have elapsed since the
     * last ping, return the ranging result, send another ping, and update lastPingTime.
     *
     * Otherwise, return -1.
     *
     * So, at the beginning of a series of measurements, call once with sonarPropagationDelayMs=0,
     * then call repeatedly with sonarPropagationDelayMs>0. Return values <0 mean no new range
     * value available.
     *
     * For a single measurement, use getDistanceSync instead.
     *
     * @param sonarPropagationDelayMs Number of milliseconds that are required to elapse since the
     *                                last ping before a ranging result will be returned and a new
     *                                ping sent.
     *
     * @param units DistanceUnit to be used for the result
     *
     * @return ranging result (if sufficient time has elapsed since the last ping), or -1.
     */
    @Override
    public double getDistanceAsync(int sonarPropagationDelayMs, DistanceUnit units)
    {
        long curTime = System.currentTimeMillis();

        double distance;

        if (sonarPropagationDelayMs <= 0) {
            ping();
            lastPingTime = System.currentTimeMillis();
            distance = -1;
        } else if(((curTime - lastPingTime) > sonarPropagationDelayMs)) {
            distance = getRangingResult(units);
            ping();
            lastPingTime = System.currentTimeMillis();
        } else {
            distance = -1;
        }

        return distance;
    }

}
