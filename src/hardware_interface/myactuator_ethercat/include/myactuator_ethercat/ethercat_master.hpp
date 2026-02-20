#ifndef MYACTUATOR_ETHERCAT__ETHERCAT_MASTER_HPP_
#define MYACTUATOR_ETHERCAT__ETHERCAT_MASTER_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include "myactuator_ethercat/myactuator_slave.hpp"

namespace myactuator_ethercat
{

/// EtherLab IgH EtherCAT master wrapper.
///
/// Manages the master lifecycle:
///   1. init()      — open master, create domain, configure slaves & PDOs
///   2. start()     — activate master, enable DC sync
///   3. exchange()  — cyclic: receive → read PDOs → [user logic] → write PDOs → send
///   4. stop()      — deactivate master, release resources
///
/// This class is not ROS-dependent. It runs in the real-time context.
/// Communication with the non-RT ROS 2 layer is through lock-free mechanisms
/// managed by the caller (myactuator_hardware).
class EthercatMaster
{
public:
  EthercatMaster();
  ~EthercatMaster();

  // Non-copyable, non-movable (owns kernel resources)
  EthercatMaster(const EthercatMaster &) = delete;
  EthercatMaster & operator=(const EthercatMaster &) = delete;

  /// Initialize master: open device, scan bus, configure slaves
  /// @param master_index  EtherLab master index (usually 0)
  /// @param slaves        Slave configurations to register
  /// @return true on success
  bool init(int master_index, std::vector<MyActuatorSlave> & slaves);

  /// Activate master and enable distributed clocks
  /// @param cycle_time_ns  Desired cycle time in nanoseconds (e.g., 1000000 for 1 ms)
  /// @return true on success
  bool start(uint64_t cycle_time_ns);

  /// Perform one cyclic exchange (call from RT thread)
  /// Receives process data, updates slave PDOs, sends process data
  void exchange();

  /// Deactivate master and release all resources
  void stop();

  /// Check if all slaves are in OP state
  bool all_slaves_operational() const;

  /// Get number of detected slaves on the bus
  uint32_t get_slave_count() const;

private:
  // EtherLab handles (opaque pointers, cast during implementation)
  // ec_master_t*  master_
  // ec_domain_t*  domain_
  void * master_{nullptr};
  void * domain_{nullptr};
  uint8_t * domain_pd_{nullptr};

  std::vector<MyActuatorSlave> * slaves_{nullptr};
  bool is_active_{false};
};

}  // namespace myactuator_ethercat

#endif  // MYACTUATOR_ETHERCAT__ETHERCAT_MASTER_HPP_
