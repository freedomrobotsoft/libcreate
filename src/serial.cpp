
//#define _GNU_SOURCE
#define _BSD_SOURCE
//#define _SVID_SOURCE

#include <iostream>

#include "create/serial.h"
#include "create/types.h"

namespace create {

  Serial::Serial(boost::shared_ptr<Data> d) :
    data(d),
    port(io),
    isReading(false),
    dataReady(false),
    corruptPackets(0),
    totalPackets(0) {
  }

  Serial::~Serial() {
    disconnect();
  }

  bool Serial::connect(const std::string& portName, const int& baud, boost::function<void()> cb) {
    using namespace boost::asio;
    port.open(portName);
    //usleep(1000000);
    port.set_option(serial_port::baud_rate(baud));
    //usleep(1000000);
    //port.set_option(serial_port::flow_control(serial_port::flow_control::hardware));
    port.set_option(serial_port::flow_control(serial_port::flow_control::none));

    // HCL: Create 2 does not seem to reply in hardware control mode :-( 
    //https://github.com/boostorg/asio/issues/65
    //https://stackoverflow.com/questions/28274367/how-to-make-boostasioserial-port-baseflow-control-use-hardware-flow-contro
    // https://sourceware.org/ml/libc-help/2011-04/msg00038.html
    //port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::hardware));

    usleep(1000000);

    if (port.is_open()) {
      callback = cb;
      bool startReadSuccess = startReading();
      if (!startReadSuccess) {
        port.close();
      }
      return startReadSuccess;
    }
    return false;
  }

  void Serial::disconnect() {
    if (isReading) {
      stopReading();
    }

    if (connected()) {
      // Ensure not in Safe/Full modes
      sendOpcode(OC_START);
      // Stop OI
      sendOpcode(OC_STOP);
      port.close();
    }
  }

  bool Serial::startReading() {
    if (!connected()) return false;

    if (!data) {
      CERR("[create::Serial] ", "data pointer not initialized.");
      return false;
    }

    // Only allow once
    if (isReading) return true;

    // Start OI
    sendOpcode(OC_START);

    if (!startSensorStream()) return false;

    io.reset();

    // Start continuously reading one byte at a time
    boost::asio::async_read(port,
                            boost::asio::buffer(&byteRead, 1),
                            boost::bind(&Serial::onData, this, _1, _2));

    ioThread = boost::thread(boost::bind(&boost::asio::io_service::run, &io));

    // Wait for first complete read to finish
    boost::unique_lock<boost::mutex> lock(dataReadyMut);

    int attempts = 1;
    int maxAttempts = 10;
    while (!dataReady) {
      if (!dataReadyCond.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(500))) {
        if (attempts >= maxAttempts) {
          CERR("[create::Serial] ", "failed to receive data from Create. Check if robot is powered!");
          io.stop();
          ioThread.join();
          return false;
        }
        attempts++;

        // Request data again
        sendOpcode(OC_START);

        // HCL We could do a restart here also

        startSensorStream();
      }
    }

    isReading = true;
    return true;
  }

  void Serial::stopReading() {
    if (isReading) {
      sendOpcode(OC_STOP); // HCL ADDED

      io.stop();
      ioThread.join();
      isReading = false;
      {
        boost::lock_guard<boost::mutex> lock(dataReadyMut);
        dataReady = false;
      }
    }
  }


  void Serial::notifyDataReady() {
    // Validate all packets
    data->validateAll();

    // Notify first data packets ready
    {
      boost::lock_guard<boost::mutex> lock(dataReadyMut);
      if (!dataReady) {
        dataReady = true;
        dataReadyCond.notify_one();
      }
    }
    // Callback to notify data is ready
    if (callback)
      callback();
  }

  void Serial::onData(const boost::system::error_code& e, const std::size_t& size) {
    if (e) {
      CERR("[create::Serial] ", "serial error - " << e.message());
      return;
    }

    // Should have read exactly one byte
    if (size == 1) {
      processByte(byteRead);
    } // end if (size == 1)

    // Read the next byte
    boost::asio::async_read(port,
                            boost::asio::buffer(&byteRead, 1),
                            boost::bind(&Serial::onData, this, _1, _2));
  }

  bool Serial::send(const uint8_t* bytes, unsigned int numBytes) {
    if (!connected()) {
      CERR("[create::Serial] ", "send failed, not connected.");
      return false;
    }
    // TODO: catch boost exceptions
    boost::asio::write(port, boost::asio::buffer(bytes, numBytes));
    return true;
  }

  bool Serial::sendOpcode(const Opcode& code) {
    uint8_t oc = (uint8_t) code;
    return send(&oc, 1);
  }

  uint64_t Serial::getNumCorruptPackets() const {
    return corruptPackets;
  }

  uint64_t Serial::getTotalPackets() const {
    return totalPackets;
  }


  // In Passive mode, Roomba will go into power saving mode to conserve battery power after five minutes of
  // inactivity. To disable sleep, pulse the BRC pin low periodically before these five minutes expire. Each
  // pulse resets this five minute counter. (One example that would not cause the baud rate to inadvertently
  // change is to pulse the pin low for one second, every minute, but there are other periods and duty cycles
  // that would work, as well.) 

  // This directly sets the pins up and down 
  //https://stackoverflow.com/questions/30618678/boostasioserial-port-set-rts-dts
  void Serial::setRTS(bool enabled)
  {
      int fd = port.native_handle();
      int data = TIOCM_RTS;
      if (!enabled)
          ioctl(fd, TIOCMBIC, &data);        
      else
          ioctl(fd, TIOCMBIS, &data);
  }

  void Serial::setDTR(bool enabled)
  {
      int fd = port.native_handle();
      int data = TIOCM_DTR;
      if (!enabled)
          ioctl(fd, TIOCMBIC, &data);        // Clears the RTS pin
      else
          ioctl(fd, TIOCMBIS, &data);        // Sets the RTS pin
  }
} // namespace create
