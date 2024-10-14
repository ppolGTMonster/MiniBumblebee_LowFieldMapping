uint16_t Hall_Write(int busAddress, uint8_t address, uint32_t value)
{
    /*
    Writing some values to a dedicated Register Values of the Hallsensor
  
    // busAddress: Address of the individual Hall sensor itself
    // address:   Address of the register
    // value:     32-bit data which are written

    */

    // Write the address that is to be written to the device
    // and then the 4 bytes of data, MSB first
    Wire.beginTransmission(busAddress);

    Wire.write(address);
    Wire.write((byte)(value >> 24));
    Wire.write((byte)(value >> 16));
    Wire.write((byte)(value >> 8));
    Wire.write((byte)(value));    

    
    return Wire.endTransmission();

    
}