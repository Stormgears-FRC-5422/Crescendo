package frc.utils.lights;

// The AddressableLED approach that WPI provides is intended for RGB lights. Stormgears has a lot of
// RGBW lights
// (lots of variations, GRBW might actually be more appropriate for this code...)
// The good news is that the protocol is really easy - the main part is just sending an array of
// bytes, so we can fool the
// RGB variant by sending the right bytes down the wire.
// To use, the example code does something like this in robotInit();
//    Specify PWM port - Must be a PWM header, not MXP or DIO
//    m_led = new AddressableLED(0);
//
//    // Reuse buffer
//    // Length is expensive to set, so only set it once, then just update data
//    m_ledBuffer = new AddressableLEDBufferRGBW(16);
//
//    // Call the effectiveLength to get the correct number of bytes set up. As opposed to length()
// which is the number of RGBW devices
//    m_led.setLength(m_ledBuffer.getEffectiveLength());
//
//    // Set the data - you can do this in periodic, too.
//    m_led.setData(m_ledBuffer);
//    // Only need to start once
//    m_led.start();

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class AddressableLEDBufferMulti extends AddressableLEDBuffer {
  int ledLength;
  int start;
  int substringLength;

  public AddressableLEDBufferMulti(int effectiveLength) {
    super(effectiveLength);
    ledLength = 0;
    start = 0;
    substringLength = 0;
  }

  public void setRGBW4(int start, int index, int r, int g, int b, int w) {
    // actual array order for the NEOPixels we have is GRBW
    // So, where are the bytes for index i? They might straddle two pixels...
    // array locations = (i-1) * 4 + 0,1,2,3 for G R B W
    // these need to be mapped to (j-1) * 3  + 0, 1, 2 for G R B
    // This is further confused because G comes before R in the array, but not in the function call.

    // byte  0    1    2 |  3    4    5 |  6    7    8 |  9   10   11
    // RGB   0    0    0 |  1    1    1 |  2    2    2 |  3    3    3
    //       G    R    B |  G    R    B |  G    R    B |  G    R    B
    // RGBW  0    0    0    0 |  1    1    1    1 |  2    2    2    2
    //       G    R    B    W |  G    R    B    W |  G    R    B    W
    // %0    g    r    b |  w  c1.r c1.b|
    // %1                |c0.g   g    r |  b    w  c1.b|
    // %2                                c0.g c0.r   g |  r    b    w

    Color8Bit c0;
    Color8Bit c1;

    // allow for substrings using an arbitrary start position
    int aG = start + index * 4;
    int j = aG / 3;

    // Stash the current color, if any. We need to merge our new colors into these slots
    c0 = super.getLED8Bit(j);
    c1 = super.getLED8Bit(j + 1);

    // which byte are we aligned with in the RGB world? Set the new colors, but keep the other bytes
    // as-is
    switch (aG % 3) {
      case 0:
        super.setRGB(j, r, g, b);
        super.setRGB(j + 1, c1.red, w, c1.blue);
        break;
      case 1:
        super.setRGB(j, g, c0.green, r);
        super.setRGB(j + 1, w, b, c1.blue);
        break;
      case 2:
        super.setRGB(j, c0.red, c0.green, g);
        super.setRGB(j + 1, b, r, w);
        break;
    }
  }

  public void setRGB4(int start, int index, int r, int g, int b) {
    setRGBW4(start, index, r, g, b, 0);
  }

  public void setRGB3(int start, int index, int r, int g, int b) {
    int j = start / 3;
    super.setRGB(j + index, r, g, b);
  }

  @Override
  public int getLength() {
    return substringLength;
  }

  public int getEffectiveLength() {
    return super.getLength();
  }
}