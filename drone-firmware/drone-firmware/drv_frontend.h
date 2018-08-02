#pragma once

#include "drv_i2c.h"
#include "blackboard.h"
#include "pitch.h" // for buzzer

#include "etc_basic_font.h"

class Frontend
{
  public:
    Frontend() {}

    void init()
    {
      leds.init();
      buzzer.init();
#if DRONE_HAS_OLED
      oled.init();
#endif
      shutdownBtn.init();
    }

    void tick()
    {
      leds.tick();
      //buzzer.tick();
#if DRONE_HAS_OLED
      oled.tick();
#endif
    }

    class Leds
    {
        static const int PIN = 13;
        static const int PIN_ERR = 12;

      public:

        Leds()
        {
        }

        void init()
        {
          pinMode(PIN, OUTPUT);
          digitalWrite(PIN, LOW);
          pinMode(PIN_ERR, OUTPUT);
          digitalWrite(PIN_ERR, LOW);

          _on = _orange = _blinking = false;
        }

        void state(SystemState state)
        {
          switch ( state )
          {
            case SS_INITIALIZING:
              set(true);
              orange(true);
              setBlinking(true, 10);
              break;
              
            case SS_READY:
              set(true);
              orange(false);
              setBlinking(true, 25);
              break;
              
            case SS_SHUTDOWN:
              set(true);
              orange(false);
              setBlinking(true, 10);
              break;
              
            case SS_FAULT:
              set(true);
              orange(true);
              setBlinking(false, 99);
              break;

            case SS_HALTED:
              set(false);
              break;
              
            default:
              set(false);
          }
        }

        void set(bool on)
        {
          _on = on;
          update();
        }

        void orange(bool err)
        {
          _orange = err;
          update();
        }

        void setBlinking(bool blinking, int duration)
        {
          _blinkTicks = duration;

          if (_blinking = blinking) return;
          _blinking = blinking;

          _blinkCount = 0;

          update();
        }

        void tick()
        {
          if (!_blinking || !_on) return;

          _blinkCount ++;
          update();
        }

        void update()
        {
          digitalWrite(PIN, LOW);
          digitalWrite(PIN_ERR, LOW);
          if (_on)
          {
            if (_blinking)
              digitalWrite(_orange ? PIN_ERR : PIN, (_blinkCount % (2 * _blinkTicks)) < _blinkTicks ? HIGH : LOW);
            else
              digitalWrite(_orange ? PIN_ERR : PIN, HIGH);
          }

        }

      private:

        bool _on, _orange;

        bool _blinking;
        int _blinkTicks;
        int _blinkCount;

    };

    class Buzzer
    {
        static const int PIN = 11;
      public:
        void init()
        {
          pinMode(PIN, OUTPUT);
          buzz(440, 300);
        }

        void buzz(unsigned frequency, unsigned long duration_ms)
        {
          unsigned long desired = micros() + duration_ms * 1000;
          int delay_ms = 500/frequency;
          while(micros() < desired)
          {
            digitalWrite(PIN, HIGH);
            delay(delay_ms);
            digitalWrite(PIN, LOW);
            delay(delay_ms);
            digitalWrite(PIN, HIGH);
            delay(delay_ms);
            digitalWrite(PIN, LOW);
            delay(delay_ms);
            digitalWrite(PIN, HIGH);
            delay(delay_ms);
            digitalWrite(PIN, LOW);
            delay(delay_ms);
            digitalWrite(PIN, HIGH);
            delay(delay_ms);
            digitalWrite(PIN, LOW);
            delay(delay_ms);
        
          }
        }

    };

    // ADAPTED FROM SEED GRAY OLED LIBRARY
    // "SSD1327 Gray OLED Driver Library"
    // Its LGPL so this is illegal maybe?
    // TODO review
    
    class OLED
    {
      public:

        static const char VERTICAL_MODE = 0x01;
        static const char HORIZONTAL_MODE = 0x01;

        static const char GrayOLED_Address    = 0x3c;
        static const char GrayOLED_Command_Mode  = 0x80;
        static const char GrayOLED_Data_Mode   = 0x40;
        
        static const char GrayOLED_Display_Off_Cmd = 0xAE;
        static const char GrayOLED_Display_On_Cmd  = 0xAF;
        
        static const char GrayOLED_Normal_Display_Cmd  = 0xA4;
        static const char GrayOLED_Inverse_Display_Cmd = 0xA7;
        static const char GrayOLED_Activate_Scroll_Cmd = 0x2F;
        static const char GrayOLED_Dectivate_Scroll_Cmd  = 0x2E;
        static const char GrayOLED_Set_ContrastLevel_Cmd = 0x81;
        
        static const char Scroll_Left     = 0x00;
        static const char Scroll_Right      = 0x01;
        
        static const char Scroll_2Frames      = 0x7;
        static const char Scroll_3Frames      = 0x4;
        static const char Scroll_4Frames      = 0x5;
        static const char Scroll_5Frames      = 0x0;
        static const char Scroll_25Frames     = 0x6;
        static const char Scroll_64Frames     = 0x1;
        static const char Scroll_128Frames    = 0x2;
        static const char Scroll_256Frames    = 0x3;
      
        char addressingMode;



        void initialScreen()
        {
          clearDisplay();         // Clear Display.
          setVerticalMode();      // Set to vertical mode for displaying text
      
          
          setTextXY(0, 0);
          putString("Hydrus MK 1");
          setTextXY(1, 0);
          for(int i = 0; i < (48); i++)
          {
            sendData(0x00);
            sendData(0x00);
            sendData(0x00);
            sendData(0x80);
            sendData(0x03);
            sendData(0x00);
            sendData(0x00);
            sendData(0x00);
          }

        }

        void tick()
        {
          if(I2C::busy()) return;

          if(BB.sys.state == SS_HALTED) 
          {
            clearDisplay();
            while(I2C::busy()); // wait for clear
          }

          // update data on screen
          setTextXY(3, 0);
          putString(toString(BB.sys.state));

          setTextXY(5, 0);
          if(BB.nav.gpsHasFix)
          {
            char buf[64];
            
            sprintf(buf, "%.3f", BB.nav.gpsLat);
            putString(buf);
            putString(BB.nav.gpsLat < 0? " S" : " N");

            setTextXY(6, 0);
            sprintf(buf, "%.3f", BB.nav.gpsLon);
            putString(buf);
            putString(BB.nav.gpsLon < 0? " W" : " E");
          }
          else 
          {
            putString("NO GPS FIX  ");
            setTextXY(6, 0);
            putString("            "); 

            //clear line
          }

          setTextXY(8, 0);
          putString(BB.comm.connected?"Connected   ":"Disconnected");

          setTextXY(10, 0);
          putNumber((int)(1000.0f*BB.sys.battVoltage));
          putString(" mV");

          time_t rawtime;
          struct tm * timeinfo;          
          time (&rawtime);
          timeinfo = localtime (&rawtime);
          
          char buf[13];
          strftime(buf, 13, "%d/%m %R", timeinfo);
          
          setTextXY(11, 0);
          putString(buf);
            
          
        }

        

        // IMPLEMENTATION
    
        void sendData(unsigned char data)
        {
            //Wire.beginTransmission(GrayOLED_Address); // begin I2C transmission
            //Wire.write(GrayOLED_Data_Mode);            // data mode
            //Wire.write(data);
            //Wire.endTransmission();                    // stop I2C transmission

            I2C::write(GrayOLED_Address, GrayOLED_Data_Mode, data, I2C::P_LOWEST);
            
        }
        
        
        void sendCommand(unsigned char command)
        {
            //Wire.beginTransmission(GrayOLED_Address); // begin I2C communication
            //Wire.write(GrayOLED_Command_Mode);    // Set OLED Command mode
            //Wire.write(command);
            //Wire.endTransmission();                  // End I2C communication

            I2C::write(GrayOLED_Address, GrayOLED_Command_Mode, command, I2C::P_LOWEST);
            
        }
        
        
        void init(void)
        {
          
            sendCommand(0xFD); // Unlock OLED driver IC MCU interface from entering command. i.e: Accept commands
            sendCommand(0x12);
            sendCommand(0xAE); // Set display off
            sendCommand(0xA8); // set multiplex ratio
            sendCommand(0x5F); // 96
            sendCommand(0xA1); // set display start line
            sendCommand(0x00);
            sendCommand(0xA2); // set display offset
            sendCommand(0x60);
            sendCommand(0xA0); // set remap
            sendCommand(0x46);
            sendCommand(0xAB); // set vdd internal
            sendCommand(0x01); //
            sendCommand(0x81); // set contrasr
            sendCommand(0x53); // 100 nit
            sendCommand(0xB1); // Set Phase Length
            sendCommand(0X51); //
            sendCommand(0xB3); // Set Display Clock Divide Ratio/Oscillator Frequency
            sendCommand(0x01);
            sendCommand(0xB9); //
            sendCommand(0xBC); // set pre_charge voltage/VCOMH
            sendCommand(0x08); // (0x08);
            sendCommand(0xBE); // set VCOMH
            sendCommand(0X07); // (0x07);
            sendCommand(0xB6); // Set second pre-charge period
            sendCommand(0x01); //
            sendCommand(0xD5); // enable second precharge and enternal vsl
            sendCommand(0X62); // (0x62);
            sendCommand(0xA4); // Set Normal Display Mode
            sendCommand(0x2E); // Deactivate Scroll
            sendCommand(0xAF); // Switch on display
            delay(100);
        
            // Row Address
            sendCommand(0x75);     // Set Row Address 
            sendCommand(0x00);    // Start 0
            sendCommand(0x5f);    // End 95 
        
        
            // Column Address
            sendCommand(0x15);    // Set Column Address 
            sendCommand(0x08);    // Start from 8th Column of driver IC. This is 0th Column for OLED 
            sendCommand(0x37);    // End at  (8 + 47)th column. Each Column has 2 pixels(segments)
        
            // Init gray level for text. Default:Brightest White
            grayH= 0xF0;
            grayL= 0x0F;

            initialScreen();
        
        }
        void setContrastLevel(unsigned char ContrastLevel)
        {
            sendCommand(GrayOLED_Set_ContrastLevel_Cmd);
            sendCommand(ContrastLevel);
        }
        
        void setHorizontalMode()
        {
            sendCommand(0xA0); // remap to
            sendCommand(0x42); // horizontal mode
        
            // Row Address
            sendCommand(0x75);    // Set Row Address 
            sendCommand(0x00);    // Start 0
            sendCommand(0x5f);    // End 95 
        
            // Column Address
            sendCommand(0x15);    // Set Column Address 
            sendCommand(0x08);    // Start from 8th Column of driver IC. This is 0th Column for OLED 
            sendCommand(0x37);    // End at  (8 + 47)th column. Each Column has 2 pixels(or segments)
        }
        
        void setVerticalMode()
        {
            sendCommand(0xA0); // remap to
            sendCommand(0x46); // Vertical mode
        }
        
        void setTextXY(unsigned char Row, unsigned char Column)
        {
            //Column Address
            sendCommand(0x15);             /* Set Column Address */
            sendCommand(0x08+(Column*4));  /* Start Column: Start from 8 */
            sendCommand(0x37);             /* End Column */
            // Row Address
            sendCommand(0x75);             /* Set Row Address */
            sendCommand(0x00+(Row*8));     /* Start Row*/
            sendCommand(0x07+(Row*8));     /* End Row*/
        }
        
        void clearDisplay()
        {
            unsigned char i,j;
            for(j=0;j<48;j++)
            {
                for(i=0;i<96;i++)  //clear all columns
                {
                    sendData(0x00);
                }
            }
        
        }
        
        
        void setGrayLevel(unsigned char grayLevel)
        {
            grayH = (grayLevel << 4) & 0xF0;
            grayL =  grayLevel & 0x0F;
        }
        
        void putChar(unsigned char C)
        {
            if(C < 32 || C > 127) //Ignore non-printable ASCII characters. This can be modified for multilingual font.
            {
                C=' '; //Space
            } 
        
            
            for(char i=0;i<8;i=i+2)
            {
                for(char j=0;j<8;j++)
                {
                    // Character is constructed two pixel at a time using vertical mode from the default 8x8 font
                    char c=0x00;
                    char bit1=(pgm_read_byte(&BasicFont[C-32][i]) >> j)  & 0x01;  
                    char bit2=(pgm_read_byte(&BasicFont[C-32][i+1]) >> j) & 0x01;
                   // Each bit is changed to a nibble
                    c|=(bit1)?grayH:0x00;
                    c|=(bit2)?grayL:0x00;
                    sendData(c);
                }
            }
        }
        
        void putString(const char *String)
        {
            unsigned char i=0;
            while(String[i])
            {
                putChar(String[i]);     
                i++;
            }
        }
        
        unsigned char putNumber(long long_num)
        {
            unsigned char char_buffer[10]="";
            unsigned char i = 0;
            unsigned char f = 0;
        
            if (long_num < 0)
            {
                f=1;
                putChar('-');
                long_num = -long_num;
            }
            else if (long_num == 0)
            {
                f=1;
                putChar('0');
                return f;
            }
        
            while (long_num > 0)
            {
                char_buffer[i++] = long_num % 10;
                long_num /= 10;
            }
        
            f=f+i;
            for(; i > 0; i--)
            {
                putChar('0'+ char_buffer[i - 1]);
            }
            return f;
        
        }
        
        
        
        void drawBitmap(unsigned char *bitmaparray,int bytes)
        {
            char localAddressMode = addressingMode;
            if(addressingMode != HORIZONTAL_MODE)
            {
                //Bitmap is drawn in horizontal mode
                setHorizontalMode();
            }
        
            for(int i=0;i<bytes;i++)
            {
        
            for(int j=0;j<8;j=j+2) 
            {
                char c=0x00;
                char bit1=pgm_read_byte(&bitmaparray[i]) << j  & 0x80;  
                char bit2=pgm_read_byte(&bitmaparray[i]) << (j+1) & 0x80;
        
                // Each bit is changed to a nibble
                c|=(bit1)?grayH:0x00;
                // Each bit is changed to a nibble
                c|=(bit2)?grayL:0x00;
                sendData(c);
             }
            }
            if(localAddressMode == VERTICAL_MODE)
            {
                //If Vertical Mode was used earlier, restore it.
                setVerticalMode();
            }
        
        }
        
        void setHorizontalScrollProperties(bool direction,unsigned char startRow, unsigned char endRow,unsigned char startColumn, unsigned char endColumn, unsigned char scrollSpeed)
        {
            /*
        Use the following defines for 'direction' :
        
         Scroll_Left      
         Scroll_Right     
        
        Use the following defines for 'scrollSpeed' :
        
         Scroll_2Frames   
         Scroll_3Frames
         Scroll_4Frames
         Scroll_5Frames 
         Scroll_25Frames
         Scroll_64Frames
         Scroll_128Frames
         Scroll_256Frames
        
        */
        
            if(Scroll_Right == direction)
            {
                //Scroll Right
                sendCommand(0x27);
            }
            else
            {
                //Scroll Left  
                sendCommand(0x26);
            }
            sendCommand(0x00);       //Dummmy byte
            sendCommand(startRow);
            sendCommand(scrollSpeed);
            sendCommand(endRow);
            sendCommand(startColumn+8);
            sendCommand(endColumn+8);
            sendCommand(0x00);      //Dummmy byte
        
        }
        
        void activateScroll()
        {
            sendCommand(GrayOLED_Activate_Scroll_Cmd);
        }
        
        void deactivateScroll()
        {
            sendCommand(GrayOLED_Dectivate_Scroll_Cmd);
        }
        
        void setNormalDisplay()
        {
            sendCommand(GrayOLED_Normal_Display_Cmd);
        }
        
        void setInverseDisplay()
        {
            sendCommand(GrayOLED_Inverse_Display_Cmd);
        }

        void turnOff()
        {
            sendCommand(0xAE); // Set display off
        }

    private:
    
      unsigned char grayH;
      unsigned char grayL;
    };

    class ShutdownButton
    {
      public:

      static const int PIN = 10;
      
      ShutdownButton()
      {
      }

      void init()
      {
        pinMode(PIN, INPUT);
      }

      bool pressed() const 
      {
#if DRONE_IGNORE_SHUTDOWN
          return false;
#else          
          return !digitalRead(PIN); /*active low*/ 
#endif       
      }
      
    };

    Leds leds;
    Buzzer buzzer;
#if DRONE_HAS_OLED
    OLED oled;
#endif
    ShutdownButton shutdownBtn;
};
