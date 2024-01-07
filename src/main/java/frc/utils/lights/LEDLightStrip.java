package frc.utils.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.ArrayList;
import java.util.List;

/*
 *  This class represents LED Light Strip.
 */
public class LEDLightStrip {

  AddressableLED lightStrip;
  AddressableLEDBufferMulti buffer;

  private List<SegmentDetails> segments = null;

  int effectiveLength = 0;

  class SegmentDetails {
    int startPosition;  // The position of the first byte of the light segment
    int endPosition; // The position of the last byte of the light segment
    int numLEDs;
    LightType lightType;

    SegmentDetails(int numLEDs, LightType lightType) {
      this.numLEDs = numLEDs;
      this.lightType = lightType;
    }

    int getLEDSize() {
      int ledSize = 0;
      switch (lightType) {
        case RGBW:
          ledSize = 4;
          break;
        case RGB:
          ledSize = 3;
          break;
        default:
          break;
      }
      return ledSize;
    }
  }

  protected void addSegment(SegmentDetails segment) {
    if (segments.size() == 0) {
      segment.startPosition = 0;
    } else {
      segment.startPosition = segments.get(segments.size() - 1).endPosition + 1;
    }
    segment.endPosition = segment.startPosition + (segment.numLEDs * segment.getLEDSize()) - 1;

    effectiveLength = (segment.endPosition + 1) / 3;
    segments.add(segment);

//    System.out.println("start" + segment.startPosition + ", end: " + segment.endPosition + ", size: " + segments.size() + ", effective length: " + effectiveLength + ", num of LEDS: " + segment.numLEDs );

  }

  public SegmentDetails getSegment(int index) {
    return this.segments.get(index);
  }

  public int numOfSegments() {
    return this.segments.size();
  }

  public LEDLightStrip() {
    this.segments = new ArrayList<>();
  }

  public void addSegment(int numOfLEDs, LightType lightType) {
    addSegment(new SegmentDetails(numOfLEDs, lightType));
  }

  // This method should be called when all segments have been added
  public void setUp(int port) {
    this.lightStrip = new AddressableLED(port);
    this.lightStrip.setLength(effectiveLength);
    this.buffer = new AddressableLEDBufferMulti(effectiveLength);
    lightStrip.setData(this.buffer);
    lightStrip.start();
  }

  public void setLEDColor(int segmentNumber, int index, Color8Bit color) {
    SegmentDetails segment = this.getSegment(segmentNumber);
    if (segment.lightType == LightType.RGBW) {
      this.buffer.setRGBW4(segment.startPosition, index, color.red, color.green, color.blue, 0);
    } else if (segment.lightType == LightType.RGB) {
      this.buffer.setRGB3(segment.startPosition, index, color.red, color.green, color.blue);
//      System.out.println("Segment Number: " + segmentNumber + " Index: " + index);
    }

  }

  // This will setup color to the entire segment
  public void setLEDColor(int segmentNumber, Color8Bit color) {
    SegmentDetails segment = this.getSegment(segmentNumber);
    for (int index = 0; index < segment.numLEDs; index++) {
      setLEDColor(segmentNumber, index, color);
    }
  }

  public void setLEDData() {
    lightStrip.setData(this.buffer);
  }

}