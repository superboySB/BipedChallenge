#include "xsens_mti.h"

#include <functional>
#include <iomanip>
#include <iostream>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"

Journaller* gJournal = 0;

namespace my_drake_examples {
namespace biped {

using drake::systems::AbstractValues;
using drake::systems::CompositeEventCollection;
using drake::systems::EventStatus;
using drake::systems::UnrestrictedUpdateEvent;

namespace {
constexpr int kStateIndexMessage = 0;
// constexpr int kStateIndexMessageCount = 1;
constexpr int kMagic = 6832;  // An arbitrary value.
}  // namespace

namespace {

using namespace std;

class CallbackHandler : public XsCallback {
 public:
  CallbackHandler(size_t maxBufferSize = 5)
      : m_maxNumberOfPacketsInBuffer(maxBufferSize),
        m_numberOfPacketsInBuffer(0) {}

  virtual ~CallbackHandler() throw() {}

  bool packetAvailable() const {
    xsens::Lock locky(&m_mutex);
    return m_numberOfPacketsInBuffer > 0;
  }

  XsDataPacket getNextPacket() {
    assert(packetAvailable());
    xsens::Lock locky(&m_mutex);
    XsDataPacket oldestPacket(m_packetBuffer.front());
    m_packetBuffer.pop_front();
    --m_numberOfPacketsInBuffer;
    return oldestPacket;
  }

 protected:
  void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override {
    xsens::Lock locky(&m_mutex);
    assert(packet != 0);
    while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
      (void)getNextPacket();

    m_packetBuffer.push_back(*packet);
    ++m_numberOfPacketsInBuffer;
    assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
  }

 private:
  mutable xsens::Mutex m_mutex;

  size_t m_maxNumberOfPacketsInBuffer;
  size_t m_numberOfPacketsInBuffer;
  list<XsDataPacket> m_packetBuffer;
};

}  // namespace

XsensMti::XsensMti() : magic_number_(kMagic) {
  // "subscribe"
  cout << "Creating XsControl object..." << endl;
  control_ = XsControl::construct();
  assert(control_ != 0);

  // Lambda function for error handling
  auto handleError = [=](string errorString) {
    control_->destruct();
    cout << errorString << endl;
    cout << "Press [ENTER] to continue." << endl;
    cin.get();
    // return -1;
    exit(-1);
  };

  cout << "Scanning for devices..." << endl;
  XsPortInfoArray portInfoArray = XsScanner::scanPorts();

  // Find an MTi device
  for (auto const& portInfo : portInfoArray) {
    if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
      mtPort_ = portInfo;
      break;
    }
  }

  if (mtPort_.empty()) handleError("No MTi device found. Aborting.");

  cout << "Found a device with ID: "
       << mtPort_.deviceId().toString().toStdString()
       << " @ port: " << mtPort_.portName().toStdString()
       << ", baudrate: " << mtPort_.baudrate() << endl;

  cout << "Opening port..." << endl;
  if (!control_->openPort(mtPort_.portName().toStdString(), mtPort_.baudrate()))
    handleError("Could not open port. Aborting.");

  // Get the device object
  device_ = control_->device(mtPort_.deviceId());
  assert(device_ != 0);

  cout << "Device: " << device_->productCode().toStdString()
       << ", with ID: " << device_->deviceId().toString() << " opened." << endl;

  // Create and attach callback handler to device
  callback_ = new CallbackHandler();
  device_->addCallbackHandler(callback_);

  // Put the device into configuration mode before configuring the device
  cout << "Putting device into configuration mode..." << endl;
  if (!device_->gotoConfig())
    handleError("Could not put device into configuration mode. Aborting.");

  cout << "Configuring the device..." << endl;

  // Important for Public XDA!
  // Call this function if you want to record a mtb file:
  device_->readEmtsAndDeviceConfiguration();

  XsOutputConfigurationArray configArray;
  configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 1));
  configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 1));
  if (device_->deviceId().isVru() || device_->deviceId().isAhrs()) {
    configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 400));
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 400));
    configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 400));
    configArray.push_back(XsOutputConfiguration(XDI_AccelerationHR, 1000));
  } else {
    handleError("not AHRS device while configuring. Aborting.");
  }

  if (!device_->setOutputConfiguration(configArray))
    handleError("Could not configure MTi device. Aborting.");

  cout << "Putting device into measurement mode..." << endl;
  if (!device_->gotoMeasurement())
    handleError("Could not put device into measurement mode. Aborting.");

  static_assert(kStateIndexMessage == 0, "");
  auto message_state_index =
      this->DeclareAbstractState(drake::Value<XsDataPacket>());
  // static_assert(kStateIndexMessageCount == 1, "");
  // this->DeclareAbstractState(drake::Value<int>(0));

  this->DeclareDiscreteState(1);  // time elapsed since start measuring
  this->DeclareDiscreteState(1);  // time elapsed since last packet
  this->DeclareStateOutputPort("time interval",
                               drake::systems::DiscreteStateIndex{1});

  this->DeclareStateOutputPort(drake::systems::kUseDefaultName,
                               message_state_index);

  this->DeclareForcedUnrestrictedUpdateEvent(&XsensMti::Update);
}

XsensMti::~XsensMti() {
  magic_number_ = 0;
  // "destruct"
  // 直接复制过来，懒得仔细调
  auto handleError = [=](string errorString) {
    control_->destruct();
    cout << errorString << endl;
    cout << "Press [ENTER] to continue." << endl;
    cin.get();
    // return -1;
    exit(-1);
  };

  cout << "Closing port..." << endl;
  control_->closePort(mtPort_.portName().toStdString());

  cout << "Freeing XsControl object..." << endl;
  control_->destruct();
  delete callback_;
}

EventStatus XsensMti::Update(const drake::systems::Context<double>& context,
                             drake::systems::State<double>* state) const {
  AbstractValues& abstract_state = state->get_mutable_abstract_state();

  auto callback = dynamic_cast<CallbackHandler*>(callback_);
  if (callback->packetAvailable()) {
    // cout << setw(5) << fixed << setprecision(2);

    // Retrieve a packet
    XsDataPacket packet = callback->getNextPacket();
    abstract_state.get_mutable_value(kStateIndexMessage)
        .get_mutable_value<XsDataPacket>() = packet;
    // std::cout << "sample time fine: " << packet.sampleTimeFine() << "\n";
  }

  const auto& old_time_elapsed = context.get_discrete_state(0).value()(0);
  auto& time_elapsed =
      state->get_mutable_discrete_state(0).get_mutable_value()(0);
  auto& time_interval =
      state->get_mutable_discrete_state(1).get_mutable_value()(0);
  time_interval = context.get_time() - old_time_elapsed;
  time_elapsed = context.get_time();
  // std::cout << "time elapsed: " << time_elapsed << "\n";
  // std::cout << "time interval: " << time_interval << "\n";

  return EventStatus::Succeeded();
}

void XsensMti::DoCalcNextUpdateTime(
    const drake::systems::Context<double>& context,
    CompositeEventCollection<double>* events, double* time) const {
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  auto callback = dynamic_cast<CallbackHandler*>(callback_);
  if (!callback->packetAvailable()) {
    return;
  }

  UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback uucallback =
      [this](const drake::systems::Context<double>& c,
             const UnrestrictedUpdateEvent<double>&,
             drake::systems::State<double>* s) { this->Update(c, s); };

  *time = context.get_time();
  drake::systems::EventCollection<UnrestrictedUpdateEvent<double>>& uu_events =
      events->get_mutable_unrestricted_update_events();
  uu_events.AddEvent(UnrestrictedUpdateEvent<double>(
      drake::systems::TriggerType::kTimed, uucallback));
}

}  // namespace biped
}  // namespace my_drake_examples
