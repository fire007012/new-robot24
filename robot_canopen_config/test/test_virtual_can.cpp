#include <gtest/gtest.h>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/clock.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/io2/vcan.hpp>
#include <lely/ev/loop.hpp>
#include <lely/ev/fiber_exec.hpp>
#include <lely/coapp/master.hpp>
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include <atomic>
#include <thread>

// Uses Lely's user-space CAN bus: no host interfaces, root privileges or real devices.
TEST(VirtualCan, FrameExchangeAndFaultEpochContract)
{
  lely::ev::FiberThread fiber_thread;
  lely::io::IoGuard guard;
  lely::io::Context context;
  lely::io::Poll poll(context);
  lely::ev::Loop loop(poll.get_poll());
  lely::io::VirtualCanController controller(lely::io::clock_monotonic);
  lely::io::VirtualCanChannel transmitter(context, loop.get_executor());
  lely::io::VirtualCanChannel receiver(context, loop.get_executor());
  transmitter.open(controller); receiver.open(controller);
  ros2_canopen::MotionHealth health;
  health.configure(true, std::chrono::milliseconds(100)); health.nmt(true);
  for (auto index : {0x6041, 0x6061, 0x6064, 0x606C}) {
    can_msg sent{}; sent.id=0x181; sent.len=2;sent.data[0]=index&255;sent.data[1]=index>>8;
    transmitter.write(sent,100);
    can_msg received{};ASSERT_EQ(receiver.read(&received,nullptr,nullptr,100),1);
    ASSERT_EQ(received.id,0x181u);health.feedback(received.data[0]|received.data[1]<<8);
  }
  ASSERT_TRUE(health.snapshot().ready); const auto old=health.snapshot().epoch;
  can_msg emcy{};emcy.id=0x81;emcy.len=8;emcy.data[0]=0x10;emcy.data[1]=0x23;
  transmitter.write(emcy,100);can_msg received{};ASSERT_EQ(receiver.read(&received,nullptr,nullptr,100),1);
  health.emergency(received.data[0]|received.data[1]<<8);
  EXPECT_FALSE(health.snapshot().ready);health.emergency(0);
  EXPECT_GT(health.snapshot().epoch,old);
  health.heartbeat(true);EXPECT_FALSE(health.snapshot().ready);
}

TEST(VirtualCan, LelyMasterBridgeReadsNumericTextAndTimesOutWithoutADeviceReply)
{
  lely::ev::FiberThread fiber_thread;
  lely::io::IoGuard guard;
  lely::io::Context context;
  lely::io::Poll poll(context);
  lely::ev::Loop loop(poll.get_poll());
  auto executor=loop.get_executor();
  lely::io::Timer timer(poll,executor,CLOCK_MONOTONIC);
  lely::io::VirtualCanController controller(lely::io::clock_monotonic);
  lely::io::VirtualCanChannel master_channel(context,executor),device_channel(context,executor);
  master_channel.open(controller);device_channel.open(controller);
  lely::canopen::AsyncMaster master(executor,timer,master_channel,TEST_MASTER_DCF,"",1);
  ros2_canopen::LelyDriverBridge driver(executor,master,2,"virtual_axis",TEST_DEVICE_EDS,"",std::chrono::milliseconds(100));
  EXPECT_FALSE(driver.is_booted());
  EXPECT_THROW(driver.wait_for_boot(), std::system_error);
  std::atomic<bool> stop{false},drop{false};
  std::thread device([&] {
    const std::string firmware = "MOCK-1.23";
    size_t segment_offset = 0;
    while(!stop) {
      can_msg request{};std::error_code ec;
      if(device_channel.read(&request,nullptr,nullptr,20,ec)!=1)continue;
      if(request.id!=0x602 || request.len!=8 || drop)continue;
      can_msg response{};response.id=0x582;response.len=8;
      response.data[1]=request.data[1];response.data[2]=request.data[2];response.data[3]=request.data[3];
      if(request.data[0]==0x40) {
        response.data[0]=0x43;
        const bool text = request.data[1]==0x0A && request.data[2]==0x10;
        response.data[0] = text ? 0x41 : 0x43;
        const uint32_t value = text ? firmware.size() : 0x12345678;
        segment_offset = 0;
        for(int i=0;i<4;++i)response.data[4+i]=value>>(8*i);
      } else if ((request.data[0] & 0xE0) == 0x60) {
        const size_t count=std::min(size_t(7),firmware.size()-segment_offset);
        response.data[0]=(request.data[0]&0x10)|((7-count)<<1)|(segment_offset+count==firmware.size()?1:0);
        for(size_t i=0;i<count;++i)response.data[i+1]=firmware[segment_offset+i];
        segment_offset+=count;
      } else if ((request.data[0] & 0xE0) == 0x20) response.data[0]=0x60;
      else continue;  // SDO aborts and unrelated frames never receive an acknowledgement.
      device_channel.write(response,100,ec);
    }
  });
  std::thread requests([&] {
    struct StopLoop { lely::io::Context &context; ~StopLoop(){context.shutdown();} } stop_loop{context};
    try {
  ros2_canopen::COData object{0x1018,1,0};
  auto numeric=driver.async_sdo_read(object);
  ASSERT_EQ(numeric.wait_for(std::chrono::seconds(3)),std::future_status::ready);
  EXPECT_EQ(numeric.get().data_,0x12345678u);
  auto text=driver.async_sdo_read_text(0x100A,0);
  ASSERT_EQ(text.wait_for(std::chrono::seconds(3)),std::future_status::ready);
  EXPECT_EQ(text.get(),"MOCK-1.23");
  drop=true;
  auto missing=driver.async_sdo_read(object);
  ASSERT_EQ(missing.wait_for(std::chrono::seconds(3)),std::future_status::ready);
  EXPECT_THROW(missing.get(),std::exception);
  drop=false;
  auto recovered=driver.async_sdo_read(object);
  ASSERT_EQ(recovered.wait_for(std::chrono::seconds(3)),std::future_status::ready);
  EXPECT_EQ(recovered.get().data_,0x12345678u);
    } catch (const std::exception &e) { ADD_FAILURE() << e.what(); }
  });
  master.Reset();
  loop.run();
  stop=true;
  requests.join();device.join();
}
