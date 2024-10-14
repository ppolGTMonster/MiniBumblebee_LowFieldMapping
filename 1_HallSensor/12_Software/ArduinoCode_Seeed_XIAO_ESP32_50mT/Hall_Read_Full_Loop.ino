String Hall_Read_Full_Loop(int busAddress, String Hallname) {
  /*
    Reading out a sensor in “full loop” mode (see data sheet), which allows the maximum resolution at the maximum speed
  

  // busAddress: Adress of the unique Hallsensor itself
  // Read the X, Y, Z 12 bit values from Register 0x28 and 0x29
  // eight times quickly using the full loop mode.

  // Return: 
    true -> Everything worked, new data is available | 
    false -> no changes to the data
  */

  uint32_t value0x27;
  String Text_Result = "";

  // Read the register the I2C loop mode is in
  uint16_t error = Hall_Read(busAddress, 0x27, value0x27);
  if (error != kNOERROR) {
    Text_Result = Hallname + ";Unable to read the ALS31300. error=" + String(error);
    return Text_Result;
  }

  // I2C loop mode is in bits 2 and 3 so mask them out
  // and set them to the full loop mode
  value0x27 = (value0x27 & 0xFFFFFFF3) | (0x2 << 2);

  /* Structure Register 0x27
   bit  1 & 0:    Powermode = 00 -> Active Mode
        3 & 2:    I2C mode = 10 -> Full Loop Mode
        6 & 5 & 4: Low Power Counter = 000 -> 0.5ms inactive (bad!) 

    Rest reserved
    --> Value: 11111111 11111111 11111111 10001000  
  */

  // Write the new values to the register the I2C loop mode is in
  error = Hall_Write(busAddress, 0x27, value0x27);
  if (error != kNOERROR) {
    Text_Result = Hallname + ";Unable to write the ALS31300. error=" + String(error);
    return Text_Result;
  }

  // Write the address that is going to be read from the ALS31300
  Wire.beginTransmission(busAddress);
  Wire.write(0x28);
  error = Wire.endTransmission(false);

  xd = 0;
  yd = 0;
  zd = 0;
  td = 0;

  // The ALS31300 accepted the address
  if (error == kNOERROR) {
    int x;
    int y;
    int z;
    int t;
    int status;
    int newData;
    int newData_total;

    xd = 0;
    yd = 0;
    zd = 0;
    td = 0;
    newData_total = 0;
    float Repeat_Steps_real = (float)Repeat_Steps;

    
    // Start the readout of the 1000 Values (which should be averaged than)
    // First: Readout + Summing up. Averaging at the end (no moving average)
    for (int count = 0; count < Repeat_Steps; ++count) { 
      // Start the read and request 8 bytes which is the contents of register 0x28 and 0x29
      Wire.requestFrom(busAddress, 8);

      // Read the first 4 bytes which are the contents of register 0x28
      x = Wire.read() << 4;  // Byte 1: X MSB
      y = Wire.read() << 4;  // Byte 2: Y MSB
      z = Wire.read() << 4;  // Byte 3: Z MSB
      //Byte 4: New Data/INT und Temp MSB
      byte d = Wire.read();
      t = (d & 0x3F) << 6;        // Bitshift by 6 Bits (Temp LSB)
      newData = (d & 0x80) >> 7;  //newData: Update flag for X/Y/Z: If new values are available = high
      

      // Register 0x29
      // Read the next 4 bytes which are the contents of register 0x29
      Wire.read();                   // Byte 5: Reserved -> Not used
      x = x | (Wire.read() & 0x0F);  // Byte 6: 4 Bits for X LSB
      d = Wire.read();               // Byte 7: 4 Bits for Y LSB and 4 Bits for X LSB
      y = y | ((d >> 4) & 0x0F);
      z = z | (d & 0x0F);
      //Byte 8: Hall Status and Temp LSB
      d = Wire.read();
      t = t | (d & 0x3F);  // Use the lower 6 Bits

      // Check the sign of the field
      // Sign extend the 12th bit for x, y and z
      x = SignExtendBitfield((uint32_t)x, 12);
      y = SignExtendBitfield((uint32_t)y, 12);
      z = SignExtendBitfield((uint32_t)z, 12);

      // New Data are read -> Sum them up
      if (newData == 1) {
        xd += x;
        yd += y;
        zd += z;
        td += t;
        newData_total = newData_total+1;
       } else {
        Repeat_Steps_real = Repeat_Steps_real - 1;
      }
    }

    // Conversion to Gauss + arithmetic mean over the number in “Repeat_Steps_real” in the for-loop + Conversion to float Type
    float mx = ((float)xd / Repeat_Steps_real)/ Hall_Sensitivity;
    float my = ((float)yd / Repeat_Steps_real)/ Hall_Sensitivity;
    float mz = ((float)zd / Repeat_Steps_real)/ Hall_Sensitivity;
    
    //  Conversion from Gauss to mT
    mx = mx / 10;
    my = my / 10;
    mz = mz / 10;
    //Conversion to °C
    float td_temp1 = (float)td/Repeat_Steps_real - 1708.0;
    float mt = 302.0 / 4096.0 * td_temp1;

    //Build String for USB Transmit
    Text_Result = Hallname + ";Bx=" + String(mx, num_decimal) + "(" + String(x) + 
                             ");By=" + String(my, num_decimal) + "(" + String(y) + 
                             ");Bz=" + String(mz, num_decimal) + "(" + String(z) + 
                             ");Temp=" + String(mt, num_decimal) + ";ND=" + String(newData_total);

    return Text_Result;


  } else {
    Text_Result = Hallname + ";Unable to read FullLoop of the ALS31300. error=" + String(error);
    return Text_Result;
  }
}