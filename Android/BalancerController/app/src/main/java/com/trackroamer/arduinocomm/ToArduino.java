package com.trackroamer.arduinocomm;

/**
 * Created by sergei on 7/27/2014.
 */
public class ToArduino
{
    public int channel;
    public int command;
    public int[] commandValues = null;

    @Override
    public String toString()
    {
        int commandValuesLength = commandValues == null ? 0 : commandValues.length;
        int cmdPlusCount = command + (commandValuesLength << 8);
        int checksum = -(channel + cmdPlusCount);

        String ret = "*" + channel + " " + cmdPlusCount;

        if (commandValuesLength > 0)
        {
            for (int val : commandValues)
            {
                ret = ret + " " + val;
                checksum -= val;
            }
        }

        // note: on Arduino side the last Serial.ParseInt must meet a non-digit char.
        //       that's why we need a trailing whitespace below:
        return  ret + " " + checksum + "\n";
    }
}
