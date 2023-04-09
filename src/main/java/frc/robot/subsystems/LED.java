package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    
  private static LED instance;
  public static LED getInstance() {
      if (instance == null) instance = new LED();
      return instance;
  }
    private AddressableLED m_led; 
    private AddressableLEDBuffer m_ledBuffer;

    public LED() {
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led = new AddressableLED(9);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

   private void ledOrange() {

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setRGB(i, 241, 102, 35);
     }
     m_led.setData(m_ledBuffer);
   }

   private void ledYellow() {
    
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setRGB(i, 255, 224, 0);
     }
     m_led.setData(m_ledBuffer);
   }

   private void ledPurple() {
    
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setRGB(i, 189, 0, 252);
     }
     m_led.setData(m_ledBuffer);
   }

}

   
