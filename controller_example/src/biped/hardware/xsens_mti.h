#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>

#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
#include <xscommon/journaller.h>

#include <list>
#include <string>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <vector>

namespace my_drake_examples {
namespace biped {

class XsensMti final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(XsensMti)

  XsensMti();

  ~XsensMti();

 private:
  void DoCalcNextUpdateTime(const drake::systems::Context<double>& context,
                            drake::systems::CompositeEventCollection<double>* events,
                            double* time) const override;
  
  drake::systems::EventStatus Update(const drake::systems::Context<double>&,
                                     drake::systems::State<double>* state) const;

  XsControl* control_;
  XsDevice* device_;
  XsCallback* callback_;
  XsPortInfo mtPort_;

  // mutable std::mutex received_message_mutex_;

  // mutable std::condition_variable received_message_condition_variable_;

  // XsDataPacket packet;

  // int received_message_count_{0};

  int magic_number_{};
};

}  // namespace biped
}  // namespace my_drake_examples
