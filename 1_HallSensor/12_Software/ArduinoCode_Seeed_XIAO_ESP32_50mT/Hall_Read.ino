uint16_t Hall_Read(int busAddress, uint8_t address, uint32_t& value)
{

    /*
    Reading out dedicated Register Values from the Hallsensor
  
    // busAddress: Address of the individual Hall sensor itself
    // address:   Address of the register
    // value:     32-bit data which are read

    */

    // Write the address that is to be read to the device
    Wire.beginTransmission(busAddress);
    Wire.write(address);
    int error = Wire.endTransmission(false);

    // if the device accepted the address,
    // request 4 bytes from the device
    // and then read them, MSB first
    if (error == kNOERROR)
    {
        Wire.requestFrom(busAddress, 4);
        value = Wire.read() << 24;
        value += Wire.read() << 16;
        value += Wire.read() << 8;
        value += Wire.read();
    }

    return error;
}