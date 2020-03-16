/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;


/**
 * Add your docs here.
 */
public class CircularBuffer extends edu.wpi.first.wpiutil.CircularBuffer 
{
    private int size;
    public CircularBuffer(int size)
    {
        super(size);
        this.size = size;
    }
    public double getSize()
    {
        return this.size;
    }
    public double getAverage()
    {
        double average = 0;

        for(int i =0; i<size; i++)
        {
            average+=this.get(i);
        }
        
        average/=size;

        return average;
    }
}
