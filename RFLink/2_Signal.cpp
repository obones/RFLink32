// ************************************* //
// * Arduino Project RFLink-esp        * //
// * https://github.com/couin3/RFLink  * //
// * 2018..2020 Marc RIVES             * //
// * More details in RFLink.ino file   * //
// ************************************* //

#include <Arduino.h>
#include "1_Radio.h"
#include "2_Signal.h"
#include "5_Plugin.h"
#include "4_Display.h"

unsigned long SignalCRC = 0L;   // holds the bitstream value for some plugins to identify RF repeats
unsigned long SignalCRC_1 = 0L; // holds the previous SignalCRC (for mixed burst protocols)
byte SignalHash = 0L;           // holds the processed plugin number
byte SignalHashPrevious = 0L;   // holds the last processed plugin number
unsigned long RepeatingTimer = 0L;

namespace RFLink
{
  namespace Signal
  {

    RawSignalStruct RawSignal = {0, 0, 0, 0, 0UL, -9999.0F}; // current message

    namespace commands
    {
      String sendRF("sendRF");
      String testRF("testRF");
    }

    namespace counters
    {
      unsigned long int receivedSignalsCount;
      unsigned long int successfullyDecodedSignalsCount;
    }

    namespace params
    {
      // All json variable names
      bool async_mode_enabled = false;
      unsigned short int sample_rate;
      unsigned long int min_raw_pulses;
      unsigned long int seek_timeout;       // MS
      unsigned long int min_preamble;       // US
      unsigned long int min_pulse_len;      // US
      unsigned long int signal_end_timeout; // US
      unsigned long int signal_repeat_time; // MS
      unsigned long int scan_high_time;     // MS

    }

    const char json_name_async_mode_enabled[] = "async_mode_enabled";
    const char json_name_sample_rate[] = "sample_rate";
    const char json_name_min_raw_pulses[] = "min_raw_pulses";
    const char json_name_seek_timeout[] = "seek_timeout";
    const char json_name_min_preamble[] = "min_preamble";
    const char json_name_min_pulse_len[] = "min_pulse_len";
    const char json_name_signal_end_timeout[] = "signal_end_timeout";
    const char json_name_signal_repeat_time[] = "signal_repeat_time";
    const char json_name_scan_high_time[] = "scan_high_time";

    Config::ConfigItem configItems[] = {
            Config::ConfigItem(json_name_async_mode_enabled, Config::SectionId::Signal_id, false, paramsUpdatedCallback),
            Config::ConfigItem(json_name_sample_rate, Config::SectionId::Signal_id, DEFAULT_RAWSIGNAL_SAMPLE_RATE, paramsUpdatedCallback),
            Config::ConfigItem(json_name_min_raw_pulses, Config::SectionId::Signal_id, MIN_RAW_PULSES, paramsUpdatedCallback),
            Config::ConfigItem(json_name_seek_timeout, Config::SectionId::Signal_id, SIGNAL_SEEK_TIMEOUT_MS, paramsUpdatedCallback),
            Config::ConfigItem(json_name_min_preamble, Config::SectionId::Signal_id, SIGNAL_MIN_PREAMBLE_US, paramsUpdatedCallback),
            Config::ConfigItem(json_name_min_pulse_len, Config::SectionId::Signal_id, MIN_PULSE_LENGTH_US, paramsUpdatedCallback),
            Config::ConfigItem(json_name_signal_end_timeout, Config::SectionId::Signal_id, SIGNAL_END_TIMEOUT_US, paramsUpdatedCallback),
            Config::ConfigItem(json_name_signal_repeat_time, Config::SectionId::Signal_id, SIGNAL_REPEAT_TIME_MS, paramsUpdatedCallback),
            Config::ConfigItem(json_name_scan_high_time, Config::SectionId::Signal_id, SCAN_HIGH_TIME_MS, paramsUpdatedCallback),
            Config::ConfigItem()};

    void paramsUpdatedCallback()
    {
      refreshParametersFromConfig();
    }

    void refreshParametersFromConfig(bool triggerChanges)
    {

      Config::ConfigItem *item;
      bool changesDetected = false;

      item = Config::findConfigItem(json_name_async_mode_enabled, Config::SectionId::Signal_id);
      if (item->getBoolValue() != params::async_mode_enabled)
      {
        changesDetected = true;
        params::async_mode_enabled = item->getBoolValue();
      }

      item = Config::findConfigItem(json_name_sample_rate, Config::SectionId::Signal_id);
      if (item->getLongIntValue() != params::sample_rate)
      {
        changesDetected = true;
        params::sample_rate = item->getLongIntValue();
      }

      item = Config::findConfigItem(json_name_min_raw_pulses, Config::SectionId::Signal_id);
      if (item->getLongIntValue() != params::min_raw_pulses)
      {
        changesDetected = true;
        params::min_raw_pulses = item->getLongIntValue();
      }

      item = Config::findConfigItem(json_name_seek_timeout, Config::SectionId::Signal_id);
      if (item->getLongIntValue() != params::seek_timeout)
      {
        changesDetected = true;
        params::seek_timeout = item->getLongIntValue();
      }

      item = Config::findConfigItem(json_name_min_preamble, Config::SectionId::Signal_id);
      if (item->getLongIntValue() != params::min_preamble)
      {
        changesDetected = true;
        params::min_preamble = item->getLongIntValue();
      }

      item = Config::findConfigItem(json_name_min_pulse_len, Config::SectionId::Signal_id);
      if (item->getLongIntValue() != params::min_pulse_len)
      {
        changesDetected = true;
        params::min_pulse_len = item->getLongIntValue();
      }

      item = Config::findConfigItem(json_name_signal_end_timeout, Config::SectionId::Signal_id);
      if (item->getLongIntValue() != params::signal_end_timeout)
      {
        changesDetected = true;
        params::signal_end_timeout = item->getLongIntValue();
      }

      item = Config::findConfigItem(json_name_signal_repeat_time, Config::SectionId::Signal_id);
      if (item->getLongIntValue() != params::signal_repeat_time)
      {
        changesDetected = true;
        params::signal_repeat_time = item->getLongIntValue();
      }

      item = Config::findConfigItem(json_name_scan_high_time, Config::SectionId::Signal_id);
      if (item->getLongIntValue() != params::scan_high_time)
      {
        changesDetected = true;
        params::scan_high_time = item->getLongIntValue();
      }

      // Applying changes will happen in mainLoop()
      if (triggerChanges && changesDetected)
      {
        Serial.println(F("Signal parameters have changed."));
        if (params::async_mode_enabled && AsyncSignalScanner::isStopped())
        {
          AsyncSignalScanner::startScanning();
        }
      }
    }

    void setup()
    {
      params::async_mode_enabled = false;
      refreshParametersFromConfig();
    }

    boolean FetchSignal_sync()
    {
      // *********************************************************************************
      static bool Toggle;
      static unsigned long timeStartSeek_ms;
      static unsigned long timeStartLoop_us;
      static unsigned int RawCodeLength;
      static unsigned long PulseLength_us;
      static const bool Start_Level = LOW;
      // *********************************************************************************

#define RESET_SEEKSTART timeStartSeek_ms = millis();
#define RESET_TIMESTART timeStartLoop_us = micros();
#define CHECK_RF ((digitalRead(Radio::pins::RX_DATA) == Start_Level) ^ Toggle)
#define CHECK_TIMEOUT ((millis() - timeStartSeek_ms) < params::seek_timeout)
#define GET_PULSELENGTH PulseLength_us = micros() - timeStartLoop_us
#define SWITCH_TOGGLE Toggle = !Toggle
#define STORE_PULSE RawSignal.Pulses[RawCodeLength++] = PulseLength_us / params::sample_rate;

      // ***   Init Vars   ***
      Toggle = true;
      RawCodeLength = 0;
      PulseLength_us = 0;

      // ***********************************
      // ***   Scan for Preamble Pulse   ***
      // ***********************************
      RESET_SEEKSTART;

      while (PulseLength_us < params::min_preamble)
      {
        while (CHECK_RF && CHECK_TIMEOUT)
          ;
        RESET_TIMESTART;
        SWITCH_TOGGLE;
        while (CHECK_RF && CHECK_TIMEOUT)
          ;
        GET_PULSELENGTH;
        SWITCH_TOGGLE;
        if (!CHECK_TIMEOUT)
          return false;
      }

      RESET_TIMESTART; // next pulse starts now before we do anything else
      //Serial.print ("PulseLength: "); Serial.println (PulseLength_us);
      STORE_PULSE;

      // ************************
      // ***   Message Loop   ***
      // ************************
      while (RawCodeLength < RAW_BUFFER_SIZE)
      {

        while (CHECK_RF)
        {
          GET_PULSELENGTH;
          if (PulseLength_us > params::signal_end_timeout)
            break;
        }

        // next Pulse starts now (while we are busy doing calculation)
        RESET_TIMESTART;

        // ***   Too short Pulse Check   ***
        if (PulseLength_us < params::min_pulse_len)
        {
          // NO RawCodeLength++;
          return false; // Or break; instead, if you think it may worth it.
        }

        // ***   Ending Pulse Check   ***
        if (PulseLength_us > params::signal_end_timeout) // Again, in main while this time
        {
          RawCodeLength++;
          break;
        }

        if(RawCodeLength%2 == 0) { // has RSSI gone up? then we reset the packet
          auto newRssi = Radio::getCurrentRssi();
          if( RawSignal.rssi+10 < newRssi ) {
            RawCodeLength = 0;
            RawSignal.rssi = newRssi;
          }
        }

        // ***   Prepare Next   ***
        SWITCH_TOGGLE;

        // ***   Store Pulse   ***
        STORE_PULSE;
      }

      if (RawCodeLength >= params::min_raw_pulses)
      {
        RawSignal.Pulses[RawCodeLength] = params::signal_end_timeout;  // Last element contains the timeout.
        RawSignal.Number = RawCodeLength - 1; // Number of received pulse times (pulsen *2)
        RawSignal.Multiply = params::sample_rate;
        RawSignal.Time = millis(); // Time the RF packet was received (to keep track of retransmits
        //Serial.print ("D");
        //Serial.print (RawCodeLength);
        return true;
      }
      else
      {
        RawSignal.Number = 0;
      }

      return false;
    }

    boolean FetchSignal_sync_rssi()
    {
      // *********************************************************************************
      static bool Toggle;
      static unsigned long timeStartSeek_ms;
      static unsigned long timeStartLoop_us;
      static unsigned int RawCodeLength;
      static unsigned long PulseLength_us;
      static const bool Start_Level = LOW;

      unsigned long gapsTotalLength; // To make statistics on Gaps length and find an earlier end to the signal
      unsigned long averagedGapsLength;
      unsigned long dynamicGapEnd_us;
      // *********************************************************************************

#define RESET_SEEKSTART timeStartSeek_ms = millis();
#define RESET_TIMESTART timeStartLoop_us = micros();
#define CHECK_RF ((digitalRead(Radio::pins::RX_DATA) == Start_Level) ^ Toggle)
#define CHECK_TIMEOUT ((millis() - timeStartSeek_ms) < params::seek_timeout)
#define GET_PULSELENGTH PulseLength_us = micros() - timeStartLoop_us
#define SWITCH_TOGGLE Toggle = !Toggle
#ifdef RFLINK_SIGNAL_RSSI_DEBUG
#define STORE_PULSE RawSignal.Pulses[RawCodeLength++] = PulseLength_us / params::sample_rate; RawSignal.Rssis[RawCodeLength] = Radio::getCurrentRssi();
#else
#define STORE_PULSE RawSignal.Pulses[RawCodeLength++] = PulseLength_us / params::sample_rate
#endif

      // ***   Init Vars   ***
      Toggle = true;
      RawCodeLength = 0;
      PulseLength_us = 0;
      gapsTotalLength = 0;
      averagedGapsLength = 0;
      dynamicGapEnd_us = 0;

      // ***********************************
      // ***   Scan for Preamble Pulse   ***
      // ***********************************
      RESET_SEEKSTART;

      while (PulseLength_us < params::min_preamble)
      {
        while (CHECK_RF && CHECK_TIMEOUT)
          ;
        RESET_TIMESTART;
        SWITCH_TOGGLE;
        while (CHECK_RF && CHECK_TIMEOUT)
          ;
        GET_PULSELENGTH;
        SWITCH_TOGGLE;
        if (!CHECK_TIMEOUT)
          return false;
      }

      RESET_TIMESTART; // next pulse starts now before we do anything else
      //Serial.print ("PulseLength: "); Serial.println (PulseLength_us);
      STORE_PULSE;

      RawSignal.rssi = Radio::getCurrentRssi();


      // ************************
      // ***   Message Loop   ***
      // ************************
      while (RawCodeLength < RAW_BUFFER_SIZE)
      {
        while (CHECK_RF)
        {
          GET_PULSELENGTH;
          if (PulseLength_us > params::signal_end_timeout)
            break;
          if (dynamicGapEnd_us > 200 && !Toggle &&  PulseLength_us > dynamicGapEnd_us) // if this is a gap and we've over the dynamic limit
            break;
        }

        // next Pulse starts now (while we are busy doing calculation)
        RESET_TIMESTART;

        // ***   Too short Pulse Check   ***
        if (PulseLength_us < params::min_pulse_len)
        {
          if(Toggle) { // current loop is the beginning of a Gap so current PulseLength_us is previous Pulse length
            // if previous pulse is happening after a long Gap, may be it's some noise that is corrupting our signal_end so we should keep the transmission
            if(dynamicGapEnd_us > 0) // we have seen enough Pulses to make statistical assumptions
            {
              if( ((unsigned long)RawSignal.Pulses[RawCodeLength])*(unsigned long)params::sample_rate >= ((unsigned long)averagedGapsLength)*(unsigned long)150/(unsigned long)100 ) {
                // if previous Gap is 1.5x the average of Gaps we will try to decode still!
                //Serial.println("attempted noise filter");
                break;
              }
            }
          }
          return false; // it seems to be noise se we're out !
        }

        if (dynamicGapEnd_us > 200 && !Toggle && PulseLength_us > dynamicGapEnd_us)
        {
          STORE_PULSE;
          //Serial.printf_P(PSTR("Ended signal because of dynamic gap length reached (pulse=%lu dynamicGap=%lu pos=%i)\r\n"), PulseLength_us, dynamicGapEnd_us, (int) RawCodeLength);
          break;
        }

        // ***   Ending Pulse Check   ***
        if (PulseLength_us > params::signal_end_timeout)
        {
          STORE_PULSE;
          break;
        }

        if(Toggle) { // current loop is the beginning of a Gap so current PulseLength_us is previous Pulse length


        } else { // This is the beginning of a Pulse so current PulseLength_us is previous Gap length
          auto newRssi = Radio::getCurrentRssi();
          if( RawSignal.rssi+10 < newRssi ) {
            // has RSSI gone up? then we reset the packet
            //Serial.printf_P(PSTR("Restart signal because of RSSI gap: old=%.0f new=%.0f at pos %i\r\n"), RawSignal.rssi, newRssi, RawCodeLength);
            RawCodeLength = 0;
            gapsTotalLength = 0;
            averagedGapsLength = 0;
            dynamicGapEnd_us = 0;
            RawSignal.rssi = newRssi;
          }

          if(RawCodeLength > 15) {
            averagedGapsLength = gapsTotalLength/(RawCodeLength/2);
            dynamicGapEnd_us = averagedGapsLength*3;
            //Serial.printf_P(PSTR("averagedGaps=%lu vs PulseLength_us=%lu\r\n"), averagedGapsLength, PulseLength_us);
            /*if( PulseLength_us > gapsTotalLength/(RawCodeLength/2) * 2 )  {
              RFLink::sendRawPrint(F("Ended signal because of averaged gaps"), true);
              STORE_PULSE;
              break;
            }*/
          }
          gapsTotalLength += PulseLength_us / params::sample_rate;
        }

        // ***   Prepare Next   ***
        SWITCH_TOGGLE;

        // ***   Store Pulse   ***
        STORE_PULSE;
      }

      if (RawCodeLength >= params::min_raw_pulses)
      {
        RawSignal.Pulses[RawCodeLength] = params::signal_end_timeout;  // Last element contains the timeout.
        RawSignal.Number = RawCodeLength - 1; // Number of received pulse times (pulse *2)
        RawSignal.Multiply = params::sample_rate;
        RawSignal.Time = millis(); // Time the RF packet was received (to keep track of retransmits
        //Serial.print ("D");
        //Serial.print (RawCodeLength);
        return true;
      }
      else
      {
        RawSignal.Number = 0;
      }

      return false;
    }

    boolean ScanEvent()
    {
      if (Radio::current_State != Radio::States::Radio_RX)
        return false;

      if (!params::async_mode_enabled)
      {

        unsigned long Timer = millis() + params::scan_high_time;

        while (Timer > millis()) // || RepeatingTimer > millis())
        {
          bool success;

          if(Radio::hardware == Radio::HardwareType::HW_SX1278_t || Radio::hardware == Radio::HardwareType::HW_RFM69NEW_t)
            success = FetchSignal_sync_rssi();
          else
            success = FetchSignal_sync();

          if (success)
          { // RF: *** data start ***
            counters::receivedSignalsCount++;
            if (PluginRXCall(0, 0))
            { // Check all plugins to see which plugin can handle the received signal.
              counters::successfullyDecodedSignalsCount++;
              RepeatingTimer = millis() + params::signal_repeat_time;
              auto responseLength = strlen(pbuffer);
              if(responseLength>1)
                sprintf(&pbuffer[responseLength-2], "RSSI=%i;\r\n", (int)RawSignal.rssi);
              return true;
            }
          }
        } // while
        return false;
      }

      // here we are in ASYNC mode

      if (!RawSignal.readyForDecoder)
      {
        if (AsyncSignalScanner::nextPulseTimeoutTime_us > 0 && AsyncSignalScanner::nextPulseTimeoutTime_us < micros())
        { // may be current pulse has now timedout so we have a signal?

          AsyncSignalScanner::onPulseTimerTimeout(); // refresh signal properties

          if (!RawSignal.readyForDecoder) // still dont have a valid signal?
            return false;
        }
        else
          return false;
      }

      counters::receivedSignalsCount++; // we have a signal, let's increment counters

      byte signalWasDecoded = PluginRXCall(0, 0); // Check all plugins to see which plugin can handle the received signal.
      if (signalWasDecoded)
      { // Check all plugins to see which plugin can handle the received signal.
        counters::successfullyDecodedSignalsCount++;
        RepeatingTimer = millis() + params::signal_repeat_time;
      }
      AsyncSignalScanner::startScanning();
      return (signalWasDecoded != 0);
    }

    namespace AsyncSignalScanner
    {
      unsigned long int lastChangedState_us = 0;
      unsigned long int nextPulseTimeoutTime_us = 0;
      bool scanningStopped = true;

      void enableAsyncReceiver()
      {
        params::async_mode_enabled = true;
        startScanning();
      }

      void disableAsyncReceiver()
      {
        params::async_mode_enabled = false;
        stopScanning();
      }

      void startScanning()
      {
        if (params::async_mode_enabled)
        {
          scanningStopped = false;
          RawSignal.readyForDecoder = false;
          RawSignal.Number = 0;
          RawSignal.Time = 0;
          RawSignal.Multiply = params::sample_rate;
          lastChangedState_us = 0;
          nextPulseTimeoutTime_us = 0;
          attachInterrupt(digitalPinToInterrupt(Radio::pins::RX_DATA), RX_pin_changed_state, CHANGE);
        }
        else
        {
          Serial.println(F("Start of async Receiver was requested but it's not enabled!"));
        }
      }

      void stopScanning()
      {
        scanningStopped = true;
        detachInterrupt(Radio::pins::RX_DATA);
      }

      void IRAM_ATTR RX_pin_changed_state()
      {
        static unsigned long lastChangedState_us = 0;
        unsigned long changeTime_us = micros();

        if (RawSignal.readyForDecoder) // it means previous packet has not been decoded yet, let's forget about it
          return;

        unsigned long pulseLength_us = changeTime_us - lastChangedState_us;
        lastChangedState_us = changeTime_us;

        if (pulseLength_us < MIN_PULSE_LENGTH_US)
        {                              // this is too short, noise?
          nextPulseTimeoutTime_us = 0; // stop watching for a timeout
          RawSignal.Number = 0;
          RawSignal.Time = 0;
        }

        int pinState = digitalRead(Radio::pins::RX_DATA);

        if (RawSignal.Time == 0)
        {                    // this is potentially the beginning of a new signal
          if (pinState != 1) // if we get 0 here it means that we are in the middle of a signal, let's forget about it
            return;

          RawSignal.Time = millis(); // record when this signal started
          RawSignal.Multiply = Signal::params::sample_rate;
          nextPulseTimeoutTime_us = changeTime_us + SIGNAL_END_TIMEOUT_US;

          return;
        }

        if (pulseLength_us > SIGNAL_END_TIMEOUT_US)
        { // signal timedout but was not caught by main loop! We will do its job
          onPulseTimerTimeout();
          return;
        }

        RawSignal.Number++;

        if (RawSignal.Number >= RAW_BUFFER_SIZE)
        {                              // this signal has too many pulses and will be dicarded
          nextPulseTimeoutTime_us = 0; // stop watching for a timeout
          RawSignal.Number = 0;
          RawSignal.Time = 0;
          //Serial.println("this signal has too many pulses and will be dicarded");
          return;
        }

        if (RawSignal.Number == 0 && pulseLength_us < SIGNAL_MIN_PREAMBLE_US)
        {                              // too short preamnble, let's drop it
          nextPulseTimeoutTime_us = 0; // stop watching for a timeout
          RawSignal.Number = 0;
          RawSignal.Time = 0;
          //Serial.print("too short preamnble, let's drop it:");Serial.println(pulseLength_us);
          return;
        }

        //Serial.print("found pulse #");Serial.println(RawSignal.Number);
        RawSignal.Pulses[RawSignal.Number] = pulseLength_us / Signal::params::sample_rate;
        nextPulseTimeoutTime_us = changeTime_us + SIGNAL_END_TIMEOUT_US;
      }

      void onPulseTimerTimeout()
      {
        if (RawSignal.readyForDecoder)
        { // it means previous packet has not been decoded yet, let's forget about it
          //Serial.println("previous signal not decoded yet, discarding this one");
          nextPulseTimeoutTime_us = 0;
          return;
        }

        /*if (digitalRead(RX_DATA) == HIGH) {   // We have a corrupted packet here
        Serial.println("corrupted signal ends with HIGH");
        RawSignal.Number = 0;
        RawSignal.Time = 0;
        nextPulseTimeoutTime_us = 0;
        return;
      }*/

        if (RawSignal.Number == 0)
        { // timeout on preamble!
          //Serial.println("timeout on preamble");
          nextPulseTimeoutTime_us = 0;
          RawSignal.Number = 0;
          RawSignal.Time = 0;
          return;
        }

        if (RawSignal.Number < MIN_RAW_PULSES)
        { // not enough pulses, we ignore it
          nextPulseTimeoutTime_us = 0;
          RawSignal.Number = 0;
          RawSignal.Time = 0;
          return;
        }

        // finally we have one!
        stopScanning();
        nextPulseTimeoutTime_us = 0;
        RawSignal.Number++;
        RawSignal.Pulses[RawSignal.Number] = SIGNAL_END_TIMEOUT_US / Signal::params::sample_rate;
        //Serial.print("found one packet, marking now for decoding. Pulses = ");Serial.println(RawSignal.Number);
        RawSignal.readyForDecoder = true;
      }
    };

    /*********************************************************************************************\
   Send bitstream to RF - Plugin 004 (Newkaku) special version
\*********************************************************************************************/
    void AC_Send(unsigned long data, byte cmd)
    {
#define AC_FPULSE 260 // Pulse width in microseconds
#define AC_FRETRANS 5 // Number of code retransmissions

      // Serial.println("Send AC");
      // Serial.println(data, HEX);
      // Serial.println(cmd, HEX);

      unsigned long bitstream = 0L;
      byte command = 0;
      // prepare data to send
      for (unsigned short i = 0; i < 32; i++)
      { // reverse data bits
        bitstream <<= 1;
        bitstream |= (data & B1);
        data >>= 1;
      }
      if (cmd != 0xff)
      { // reverse dim bits
        for (unsigned short i = 0; i < 4; i++)
        {
          command <<= 1;
          command |= (cmd & B1);
          cmd >>= 1;
        }
      }
      // send bits
      for (byte nRepeat = 0; nRepeat < AC_FRETRANS; nRepeat++)
      {
        data = bitstream;
        if (cmd != 0xff)
          cmd = command;
        digitalWrite(Radio::pins::TX_DATA, HIGH);
        //delayMicroseconds(fpulse);  //335
        delayMicroseconds(335);
        digitalWrite(Radio::pins::TX_DATA, LOW);
        delayMicroseconds(AC_FPULSE * 10 + (AC_FPULSE >> 1)); //335*9=3015 //260*10=2600
        for (unsigned short i = 0; i < 32; i++)
        {
          if (i == 27 && cmd != 0xff)
          { // DIM command, send special DIM sequence TTTT replacing on/off bit
            digitalWrite(Radio::pins::TX_DATA, HIGH);
            delayMicroseconds(AC_FPULSE);
            digitalWrite(Radio::pins::TX_DATA, LOW);
            delayMicroseconds(AC_FPULSE);
            digitalWrite(Radio::pins::TX_DATA, HIGH);
            delayMicroseconds(AC_FPULSE);
            digitalWrite(Radio::pins::TX_DATA, LOW);
            delayMicroseconds(AC_FPULSE);
          }
          else
            switch (data & B1)
            {
              case 0:
                digitalWrite(Radio::pins::TX_DATA, HIGH);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, LOW);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, HIGH);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, LOW);
                delayMicroseconds(AC_FPULSE * 5); // 335*3=1005 260*5=1300  260*4=1040
                break;
              case 1:
                digitalWrite(Radio::pins::TX_DATA, HIGH);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, LOW);
                delayMicroseconds(AC_FPULSE * 5);
                digitalWrite(Radio::pins::TX_DATA, HIGH);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, LOW);
                delayMicroseconds(AC_FPULSE);
                break;
            }
          //Next bit
          data >>= 1;
        }
        // send dim bits when needed
        if (cmd != 0xff)
        { // need to send DIM command bits
          for (unsigned short i = 0; i < 4; i++)
          { // 4 bits
            switch (cmd & B1)
            {
              case 0:
                digitalWrite(Radio::pins::TX_DATA, HIGH);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, LOW);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, HIGH);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, LOW);
                delayMicroseconds(AC_FPULSE * 5); // 335*3=1005 260*5=1300
                break;
              case 1:
                digitalWrite(Radio::pins::TX_DATA, HIGH);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, LOW);
                delayMicroseconds(AC_FPULSE * 5);
                digitalWrite(Radio::pins::TX_DATA, HIGH);
                delayMicroseconds(AC_FPULSE);
                digitalWrite(Radio::pins::TX_DATA, LOW);
                delayMicroseconds(AC_FPULSE);
                break;
            }
            //Next bit
            cmd >>= 1;
          }
        }
        //Send termination/synchronisation-signal. Total length: 32 periods
        digitalWrite(Radio::pins::TX_DATA, HIGH);
        delayMicroseconds(AC_FPULSE);
        digitalWrite(Radio::pins::TX_DATA, LOW);
        delayMicroseconds(AC_FPULSE * 40); //31*335=10385 40*260=10400
      }
      // End transmit
    }

    void RawSendRF(RawSignalStruct *signal)
    {
      int x;

      Radio::set_Radio_mode(Radio::States::Radio_TX);

      //RawSignal.Pulses[RawSignal.Number]=1;                                   // due to a bug in Arduino 1.0.1

      for (byte y = 0; y <= signal->Repeats; y++)
      { // herhaal verzenden RF code
        x = 1;
        noInterrupts();
        while (x < signal->Number)
        {
          digitalWrite(Radio::pins::TX_DATA, HIGH);
          delayMicroseconds(signal->Pulses[x++] * signal->Multiply); // min een kleine correctie
          digitalWrite(Radio::pins::TX_DATA, LOW);
          delayMicroseconds(signal->Pulses[x++] * signal->Multiply); // min een kleine correctie
        }
        interrupts();
        if (y != signal->Repeats)
          delay(signal->Delay); // Delay buiten het gebied waar de interrupts zijn uitgeschakeld! Anders werkt deze funktie niet.
      }

      Radio::set_Radio_mode(Radio::States::Radio_RX);
    }

    bool getSignalFromJson(RawSignalStruct &signal, const char *json_str)
    {
      int jsonSize = strlen(json_str) * 6;
      DynamicJsonDocument json(jsonSize);

      if (deserializeJson(json, json_str) != DeserializationError::Ok)
      {
        Serial.println(F("An error occured while reading json"));
        return false;
      }

      signal.Number = 0;
      signal.Delay = 0;
      signal.Multiply = params::sample_rate;
      signal.Time = 0UL;

      auto &&root = json.as<JsonObject>();
      JsonArrayConst pulsesJson = root.getMember("pulses");


      signal.Number = pulsesJson.size();

      if(signal.Number < 2) {
        Serial.println(F("error, your signal has 0 pulse defined!"));
        return false;
      }

      if(signal.Number > RAW_BUFFER_SIZE) {
        Serial.printf_P(PSTR("error, your Signal has %i pulses while this supports only %i\r\n"), signal.Number, RAW_BUFFER_SIZE);
        return false;
      }

      int index = 0;
      for (JsonVariantConst pulse : pulsesJson)
      {
        index++;
        signal.Pulses[index] = pulse.as<signed long int>() / params::sample_rate;
        //Serial.printf("Pulse=%i\r\n",signal.Pulses[index]);
      }

      signal.Repeats = root.getMember("repeat").as<signed int>();
      signal.Delay = root.getMember("delay").as<signed int>();

      return true;

    }

    void executeCliCommand(char *cmd)
    {
      static const char error_command_aborted[] PROGMEM = "An error occurred, invalid signal was given. Command aborted!";

      RawSignalStruct signal{};

      char *commaIndex = strchr(cmd, ';');

      if (commaIndex == nullptr)
      {
        Serial.println(F("Error : failed to find ending ';' for the command"));
        return;
      }

      *commaIndex = 0; // replace ';' with null termination

      if (strncasecmp(commands::sendRF.c_str(), cmd, commands::sendRF.length()) == 0)
      {
        if(!getSignalFromJson(signal, commaIndex + 1)) {
          RFLink::sendRawPrint(FPSTR(error_command_aborted), true);
          return;
        }

        Serial.printf_P(PSTR("** sending RF signal with the following properties: pulses=%i, repeat=%i, delay=%i, multiply=%i... "), signal.Number, signal.Repeats, signal.Delay, signal.Multiply);
        RawSendRF(&signal);
        Serial.println(F("done"));
      }
      else if (strncasecmp(commands::testRF.c_str(), cmd, commands::testRF.length()) == 0)
      {
        RawSignal.readyForDecoder = true;

        if(!getSignalFromJson(RawSignal, commaIndex+1)) {
          Serial.println(FPSTR(error_command_aborted));
          RawSignal.readyForDecoder = false;
          return;
        }

        Serial.printf_P(PSTR("Sending your signal to Plugins (%i pulses)\r\n"), RawSignal.Number);

        if (!PluginRXCall(0, 0)){
          Serial.println(F("No plugin has matched your signal"));
        }
        else
          RFLink::sendMsgFromBuffer();

        RawSignal.readyForDecoder = false;
      }
      else
      {
        Serial.printf_P(PSTR("Error : unknown command '%s'\r\n"), cmd);
      }
    }

    void getStatusJsonString(JsonObject &output)
    {
      auto &&signal = output.createNestedObject("signal");
      signal[F("received_signal_count")] = counters::receivedSignalsCount;
      signal[F("successfully_decoded_count")] = counters::successfullyDecodedSignalsCount;
    }

  } // end of ns Signal
} // end of ns RFLink

/*********************************************************************************************/