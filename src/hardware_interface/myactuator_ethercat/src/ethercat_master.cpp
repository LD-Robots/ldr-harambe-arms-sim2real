#include "myactuator_ethercat/ethercat_master.hpp"
#include <ecrt.h>
#include <cstdio>
#include <ctime>

namespace myactuator_ethercat
{

EthercatMaster::EthercatMaster() = default;

EthercatMaster::~EthercatMaster()
{
  if (is_active_) {
    stop();
  }
}

bool EthercatMaster::init(int master_index, std::vector<MyActuatorSlave> & slaves)
{
  slaves_ = &slaves;

  // Request EtherCAT master
  auto * master = ecrt_request_master(static_cast<unsigned int>(master_index));
  if (!master) {
    fprintf(stderr, "ecrt_request_master(%d) failed\n", master_index);
    return false;
  }
  master_ = master;

  // Create process data domain
  auto * domain = ecrt_master_create_domain(master);
  if (!domain) {
    fprintf(stderr, "ecrt_master_create_domain() failed\n");
    ecrt_release_master(master);
    master_ = nullptr;
    return false;
  }
  domain_ = domain;

  // Configure each slave: slave config + PDO registration
  for (auto & slave : *slaves_) {
    if (!slave.register_pdos(master_, domain_)) {
      fprintf(stderr, "Failed to register PDOs for slave %u\n",
              slave.get_slave_position());
      ecrt_release_master(master);
      master_ = nullptr;
      domain_ = nullptr;
      return false;
    }
  }

  return true;
}

bool EthercatMaster::start(uint64_t cycle_time_ns)
{
  if (!master_ || !domain_) return false;

  auto * master = static_cast<ec_master_t *>(master_);
  auto * domain = static_cast<ec_domain_t *>(domain_);

  // Configure distributed clocks for each slave
  for (auto & slave : *slaves_) {
    slave.configure_dc(static_cast<uint32_t>(cycle_time_ns));
  }

  // Activate master (transitions slaves through PREOP -> SAFEOP -> OP)
  if (ecrt_master_activate(master)) {
    fprintf(stderr, "ecrt_master_activate() failed\n");
    return false;
  }

  // Get process data memory pointer
  domain_pd_ = ecrt_domain_data(domain);
  if (!domain_pd_) {
    fprintf(stderr, "ecrt_domain_data() returned NULL\n");
    ecrt_master_deactivate(master);
    return false;
  }

  is_active_ = true;
  return true;
}

void EthercatMaster::exchange()
{
  if (!is_active_) return;

  auto * master = static_cast<ec_master_t *>(master_);
  auto * domain = static_cast<ec_domain_t *>(domain_);

  // Receive EtherCAT frames
  ecrt_master_receive(master);
  ecrt_domain_process(domain);

  // Read TxPDOs from all slaves (feedback data)
  for (auto & slave : *slaves_) {
    slave.read_pdos(domain_pd_);
  }

  // Write RxPDOs for all slaves (command data)
  // In read-only mode, rx_pdo_ is zero-initialized: control_word = 0x0000
  for (auto & slave : *slaves_) {
    slave.write_pdos(domain_pd_);
  }

  // Queue domain for sending
  ecrt_domain_queue(domain);

  // Distributed clock synchronization
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  ecrt_master_application_time(
    master,
    static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
    static_cast<uint64_t>(ts.tv_nsec));
  ecrt_master_sync_reference_clock(master);
  ecrt_master_sync_slave_clocks(master);

  // Send EtherCAT frames
  ecrt_master_send(master);
}

void EthercatMaster::stop()
{
  if (master_) {
    auto * master = static_cast<ec_master_t *>(master_);
    if (is_active_) {
      ecrt_master_deactivate(master);
    }
    ecrt_release_master(master);
    master_ = nullptr;
    domain_ = nullptr;
    domain_pd_ = nullptr;
  }
  is_active_ = false;
}

bool EthercatMaster::all_slaves_operational() const
{
  if (!master_) return false;
  auto * master = static_cast<ec_master_t *>(master_);
  ec_master_state_t state;
  ecrt_master_state(master, &state);
  return state.al_states == 0x08;  // All slaves in OP
}

uint32_t EthercatMaster::get_slave_count() const
{
  if (!master_) return 0;
  auto * master = static_cast<ec_master_t *>(master_);
  ec_master_state_t state;
  ecrt_master_state(master, &state);
  return state.slaves_responding;
}

}  // namespace myactuator_ethercat
