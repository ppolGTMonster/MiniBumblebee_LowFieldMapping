void Hall_SetCustomer_Access(int busAddress, String Hall_Name)
{
  // Set Customer Write Access on ALS31300 (p20 Datasheet)
  uint16_t error = Hall_Write(busAddress, 0x35, 0x2C413534);  // Super important! And often forgotten on other published codes 
  if (error != kNOERROR ) 
  {
    String Text = Hall_Name + ": Error while enter customer access mode. error = " + String(error);
    Serial.println(Text);
  }
  }